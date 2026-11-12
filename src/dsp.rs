//! Signal processing helpers: FFT magnitude, FIR NLMS identification and the
//! SVD-based path compression used before uploading paths to the device.

use anyhow::{Result, bail};
use nalgebra::DMatrix;
use rustfft::{FftPlanner, num_complex::Complex};

/// Magnitude spectrum in dB, single-sided: returns `(magdb, freqs)`.
pub fn easy_fourier(x: &[f64], fs: f64) -> (Vec<f64>, Vec<f64>) {
    let n = x.len();
    if n == 0 {
        return (vec![], vec![]);
    }
    let mut buf: Vec<Complex<f64>> = x.iter().map(|v| Complex::new(*v, 0.0)).collect();
    FftPlanner::new().plan_fft_forward(n).process(&mut buf);
    let half = n / 2;
    let mag = buf[..half]
        .iter()
        .map(|c| 20.0 * (2.0 * c.norm() / n as f64).log10())
        .collect();
    let freqs = (0..half).map(|k| k as f64 / n as f64 * fs).collect();
    (mag, freqs)
}

/// FIR filter identified by the normalized LMS algorithm.
pub struct FirNlms {
    pub n: usize,
    pub mu: f64,
    pub psi: f64,
    pub avg_window: Option<usize>,
    pub ww: Vec<f64>,
    pub wwavg: Vec<f64>,
    pub sqerror: Vec<f64>,
}

impl FirNlms {
    pub fn new(
        memorysize: usize,
        stepsize: f64,
        regularization: f64,
        avg_window: Option<usize>,
    ) -> Self {
        Self {
            n: memorysize,
            mu: stepsize,
            psi: regularization,
            avg_window,
            ww: vec![],
            wwavg: vec![],
            sqerror: vec![],
        }
    }

    pub fn run(&mut self, input: &[f64], output: &[f64]) -> Result<()> {
        if input.len() != output.len() {
            bail!("Input and output must be vectors with same length.");
        }
        let n = self.n;
        let range = input.len();
        let mut xx = vec![0.0; n];
        let mut ww = vec![0.0; n];
        let mut wwavg = vec![0.0; n];
        self.sqerror = vec![0.0; range];
        for k in 0..range {
            xx.copy_within(0..n - 1, 1);
            xx[0] = input[k];
            let y: f64 = xx.iter().zip(&ww).map(|(a, b)| a * b).sum();
            let e = output[k] - y;
            let energy: f64 = xx.iter().map(|a| a * a).sum();
            let g = self.mu * e / (energy + self.psi);
            for (w, x) in ww.iter_mut().zip(&xx) {
                *w += g * x;
            }
            self.sqerror[k] = e * e;
            if let Some(win) = self.avg_window
                && k + win >= range
            {
                for (a, w) in wwavg.iter_mut().zip(&ww) {
                    *a += w;
                }
            }
        }
        if let Some(win) = self.avg_window {
            for a in wwavg.iter_mut() {
                *a /= win as f64;
            }
            self.wwavg = wwavg;
        }
        self.ww = ww;
        Ok(())
    }
}

/// Pads `w` with zeros to a multiple of `r` and reshapes it column-major into
/// a matrix with `min(r, len/r)` rows (port of `format_W`).
pub fn format_w(w: &[f64], r: usize) -> DMatrix<f64> {
    let n = w.len();
    let rem = n % r;
    let padded_len = if rem != 0 { n + (r - rem) } else { n };
    let c = padded_len / r;
    let rows = r.min(c).max(1);
    let cols = padded_len / rows;
    let mut data = vec![0.0; rows * cols];
    data[..n].copy_from_slice(w);
    DMatrix::from_column_slice(rows, cols, &data)
}

/// Full SVD: returns (U thin, singular values sorted descending, full Vᵀ (cols × cols)).
pub fn full_svd(w: &DMatrix<f64>) -> (DMatrix<f64>, Vec<f64>, DMatrix<f64>) {
    let (rows, cols) = w.shape();
    let svd = w.clone().svd(true, true);
    let u = svd.u.unwrap();
    let vt = svd.v_t.unwrap();
    let k = svd.singular_values.len();
    let mut order: Vec<usize> = (0..k).collect();
    order.sort_by(|&a, &b| {
        svd.singular_values[b]
            .partial_cmp(&svd.singular_values[a])
            .unwrap()
    });
    let s: Vec<f64> = order.iter().map(|&i| svd.singular_values[i]).collect();
    let mut u_sorted = DMatrix::zeros(rows, k);
    let mut basis: Vec<Vec<f64>> = Vec::with_capacity(cols);
    for (j, &i) in order.iter().enumerate() {
        u_sorted.set_column(j, &u.column(i));
        basis.push(vt.row(i).iter().copied().collect());
    }
    // Complete Vᵀ to an orthonormal basis of R^cols (Gram-Schmidt on unit vectors).
    let mut e = 0;
    while basis.len() < cols && e < cols {
        let mut v = vec![0.0; cols];
        v[e] = 1.0;
        for _ in 0..2 {
            for b in &basis {
                let d: f64 = v.iter().zip(b).map(|(x, y)| x * y).sum();
                for (x, y) in v.iter_mut().zip(b) {
                    *x -= d * y;
                }
            }
        }
        let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm > 1e-8 {
            basis.push(v.iter().map(|x| x / norm).collect());
        }
        e += 1;
    }
    let vt_full = DMatrix::from_fn(cols, cols, |r, c| basis[r][c]);
    (u_sorted, s, vt_full)
}

/// Parameters for SVD path compression.
#[derive(Clone, Copy, Debug)]
pub struct CompressionParams {
    pub mem: usize,
    pub branches: usize,
    pub sparsity: usize,
    /// Use the basis of the other path (feedback basis for sec / secondary basis for fbk).
    pub use_other_basis: bool,
}

pub struct CompressedPath {
    pub weights: Vec<f64>,
    pub leading_bytes: [u8; 5],
    pub singular_values: Vec<f64>,
}

/// Port of `getCompressedPath`. `is_sec` selects the secondary path.
pub fn compress_path(
    secpath: &[f64],
    fbkpath: &[f64],
    is_sec: bool,
    p: CompressionParams,
) -> Result<CompressedPath> {
    if p.sparsity == 0 || p.branches == 0 {
        bail!("Branches and sparsity must be positive.");
    }
    let c_chosen = p.mem.div_ceil(p.sparsity);
    let fmt = |path: &[f64], own: bool| -> DMatrix<f64> {
        // The secondary path is reshaped with `sparsity` rows, the feedback path with `C` rows.
        let use_sec_shape = if own { is_sec } else { !is_sec };
        if use_sec_shape {
            format_w(path, p.sparsity)
        } else {
            format_w(path, c_chosen)
        }
    };
    let (w, us, vt) = if !p.use_other_basis {
        let w = if is_sec {
            fmt(secpath, true)
        } else {
            fmt(fbkpath, true)
        };
        let (u, s, vt) = full_svd(&w);
        let rows = w.nrows();
        let mut us = DMatrix::zeros(rows, c_chosen);
        for i in 0..rows.min(c_chosen).min(s.len()) {
            us.set_column(i, &(u.column(i) * s[i]));
        }
        (w, us, vt)
    } else {
        // Basis taken from the other path; reshape it the same way as this path.
        let waux = if is_sec {
            format_w(fbkpath, p.sparsity)
        } else {
            format_w(secpath, c_chosen)
        };
        let (_, _, vt) = full_svd(&waux);
        let w = if is_sec {
            fmt(secpath, true)
        } else {
            fmt(fbkpath, true)
        };
        if w.ncols() != vt.nrows() {
            bail!("Secondary and feedback paths must have the same length to share a basis.");
        }
        let us = &w * vt.transpose();
        (w, us, vt)
    };
    let b = p.branches;
    if b > vt.nrows() || b > us.ncols() {
        bail!("Number of branches ({b}) is too large for the chosen sparsity.");
    }
    let mut weights = Vec::with_capacity(b * (vt.ncols() + us.nrows()));
    for i in 0..b {
        weights.extend(vt.row(i).iter().copied());
    }
    for i in 0..b {
        weights.extend(us.column(i).iter().copied());
    }
    let singular_values = (0..us.ncols())
        .map(|j| us.column(j).norm_squared())
        .collect();
    let rows = w.nrows();
    let leading_bytes = [
        (p.mem >> 8) as u8,
        (p.mem & 0xFF) as u8,
        b as u8,
        rows as u8,
        c_chosen as u8,
    ];
    Ok(CompressedPath {
        weights,
        leading_bytes,
        singular_values,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourier_of_sine() {
        let fs = 250.0;
        let n = 250;
        let x: Vec<f64> = (0..n)
            .map(|k| (2.0 * std::f64::consts::PI * 10.0 * k as f64 / fs).sin())
            .collect();
        let (mag, freqs) = easy_fourier(&x, fs);
        assert_eq!(mag.len(), 125);
        let imax = mag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!((freqs[imax] - 10.0).abs() < 1e-9);
        assert!(mag[imax].abs() < 1e-6); // amplitude 1 -> 0 dB
    }

    #[test]
    fn nlms_identifies_fir() {
        let h = [0.5, -0.3, 0.2];
        let mut state: u64 = 12345;
        let x: Vec<f64> = (0..5000)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((state >> 33) as f64 / (1u64 << 31) as f64) - 0.5
            })
            .collect();
        let d: Vec<f64> = (0..x.len())
            .map(|n| {
                (0..3)
                    .map(|k| if n >= k { h[k] * x[n - k] } else { 0.0 })
                    .sum()
            })
            .collect();
        let mut f = FirNlms::new(5, 0.5, 1e-6, Some(100));
        f.run(&x, &d).unwrap();
        for k in 0..3 {
            assert!((f.wwavg[k] - h[k]).abs() < 1e-3, "{:?}", f.wwavg);
        }
        assert!(f.wwavg[3].abs() < 1e-3);
    }

    #[test]
    fn format_w_is_column_major() {
        let w: Vec<f64> = (1..=7).map(|v| v as f64).collect();
        let m = format_w(&w, 3);
        assert_eq!(m.shape(), (3, 3));
        assert_eq!(m[(0, 0)], 1.0);
        assert_eq!(m[(1, 0)], 2.0);
        assert_eq!(m[(0, 1)], 4.0);
        assert_eq!(m[(0, 2)], 7.0);
        assert_eq!(m[(2, 2)], 0.0);
    }

    #[test]
    fn full_svd_reconstructs() {
        let w = DMatrix::from_fn(3, 5, |r, c| ((r * 7 + c * 3) % 5) as f64 - 2.0);
        let (u, s, vt) = full_svd(&w);
        assert_eq!(vt.shape(), (5, 5));
        let ortho = &vt * vt.transpose();
        assert!((ortho - DMatrix::identity(5, 5)).norm() < 1e-9);
        let mut rec = DMatrix::zeros(3, 5);
        for i in 0..s.len() {
            rec += u.column(i) * vt.row(i) * s[i];
        }
        assert!((rec - &w).norm() < 1e-9);
        assert!(s.windows(2).all(|p| p[0] >= p[1]));
    }

    #[test]
    fn compression_full_rank_reconstructs_path() {
        // With branches == rows, the compressed representation is exact.
        let sec: Vec<f64> = (0..100)
            .map(|k| ((k as f64) * 0.37).sin() * (-(k as f64) / 30.0).exp())
            .collect();
        let fbk = sec.clone();
        let p = CompressionParams {
            mem: 100,
            branches: 10,
            sparsity: 10,
            use_other_basis: false,
        };
        let c = compress_path(&sec, &fbk, true, p).unwrap();
        assert_eq!(c.leading_bytes, [0, 100, 10, 10, 10]);
        let (cw, rw) = c.weights.split_at(10 * 10);
        // W[r, c] = sum_b R_w[b][r] * C_w[b][c]; path index = c * rows + r
        for r in 0..10 {
            for col in 0..10 {
                let v: f64 = (0..10).map(|b| rw[b * 10 + r] * cw[b * 10 + col]).sum();
                assert!((v - sec[col * 10 + r]).abs() < 1e-9);
            }
        }
    }
}
