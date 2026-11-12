//! Cantilever beam simulator with acceleration outputs (port of `CantileverBeam.py`).
//! Used only by the developer debug mode, where the host simulates the plant.

use nalgebra::{DMatrix, DVector};
use rand::RngExt;
use rand_distr::StandardNormal;

#[derive(Clone, Debug)]
pub struct BeamParams {
    pub npoints: usize,
    pub width: f64,
    pub thickness: f64,
    pub length: f64,
    pub density: f64,
    pub elasticmod: f64,
    pub tsampling: f64,
    pub nmodes: usize,
    pub damp: Vec<f64>,
    pub forcescaler: f64,
    pub noisestd: f64,
}

impl Default for BeamParams {
    fn default() -> Self {
        Self {
            npoints: 60,
            width: 0.05,
            thickness: 0.00575,
            length: 0.58,
            density: 7900.0,
            elasticmod: 2e11,
            tsampling: 0.004,
            nmodes: 5,
            damp: vec![0.002, 0.002, 0.001, 0.001, 0.001],
            forcescaler: 1.0,
            noisestd: 0.0,
        }
    }
}

pub struct CantileverBeam {
    pub ts: f64,
    pub fs: f64,
    pub npoints: usize,
    pub nmodes: usize,
    pub length: f64,
    pub forcescaler: f64,
    pub forcescaler1: f64,
    pub magnetdist: f64,
    pub noisestd: f64,
    pub accel_in_g: bool,
    pub vmod: DMatrix<f64>,
    pub freqs_hz: Vec<f64>,
    wd: Vec<f64>,
    aiir: Vec<[f64; 2]>,
    biir: Vec<[f64; 3]>,
    rotvelmultiplier: f64,
    pub f: DVector<f64>,
    pub x: DVector<f64>,
    pub a: DVector<f64>,
    xiir: Vec<[f64; 3]>,
    yiir: Vec<[f64; 3]>,
    bufdesloc: DVector<f64>,
    bufvel: [DVector<f64>; 2],
    pub rotvel: DVector<f64>,
}

const MASS: f64 = 1.0;

impl CantileverBeam {
    pub fn new(p: BeamParams) -> Self {
        let n = p.npoints;
        let deltax = p.length / (n as f64 - 1.0);
        let (vmod, freqs_hz) = Self::modes_and_freqs(&p, deltax);
        let wn: Vec<f64> = freqs_hz
            .iter()
            .map(|f| 2.0 * std::f64::consts::PI * f)
            .collect();
        let zeta = &p.damp;
        let wd: Vec<f64> = (0..p.nmodes)
            .map(|k| wn[k] * (1.0 - zeta[k] * zeta[k]).sqrt())
            .collect();
        let ts = p.tsampling;
        let mut aiir = vec![[0.0; 2]; p.nmodes];
        let mut biir = vec![[0.0; 3]; p.nmodes];
        for k in 0..p.nmodes {
            let e = (-zeta[k] * wn[k] * ts).exp();
            biir[k] = [0.0, e * (wd[k] * ts).sin(), 0.0];
            aiir[k] = [
                -2.0 * e * (wd[k] * ts).cos(),
                (-2.0 * zeta[k] * wn[k] * ts).exp(),
            ];
        }
        let mut beam = Self {
            ts,
            fs: 1.0 / ts,
            npoints: n,
            nmodes: p.nmodes,
            length: p.length,
            forcescaler: p.forcescaler,
            forcescaler1: p.forcescaler,
            magnetdist: 1e-3,
            noisestd: p.noisestd,
            accel_in_g: false,
            vmod,
            freqs_hz,
            wd,
            aiir,
            biir,
            rotvelmultiplier: (1.0 / deltax) * 180.0 / std::f64::consts::PI,
            f: DVector::zeros(n),
            x: DVector::zeros(n),
            a: DVector::zeros(n),
            xiir: vec![[0.0; 3]; p.nmodes],
            yiir: vec![[0.0; 3]; p.nmodes],
            bufdesloc: DVector::zeros(n),
            bufvel: [DVector::zeros(n), DVector::zeros(n)],
            rotvel: DVector::zeros(n),
        };
        beam.reset();
        beam
    }

    fn modes_and_freqs(p: &BeamParams, deltax: f64) -> (DMatrix<f64>, Vec<f64>) {
        let n = p.npoints;
        let inertia = p.width * p.thickness.powi(3) / 12.0;
        let beam_mass = p.density * p.width * p.thickness * p.length;
        let mut mdiag = vec![beam_mass / n as f64; n];
        mdiag[0] /= 2.0;
        let mut a = DMatrix::<f64>::zeros(n, n);
        for r in 0..n {
            for c in 0..=r {
                let k = (n - r) as f64;
                let l = (n - c) as f64;
                let v = (deltax.powi(3) / (6.0 * p.elasticmod * inertia))
                    * (3.0 * k * k * l - k.powi(3));
                a[(r, c)] = v;
                a[(c, r)] = v;
            }
        }
        let kmat = a
            .try_inverse()
            .expect("flexibility matrix must be invertible");
        // M is diagonal, so its Cholesky factor is diag(sqrt(m)).
        let linv = DMatrix::from_diagonal(&DVector::from_iterator(
            n,
            mdiag.iter().map(|m| 1.0 / m.sqrt()),
        ));
        let mut kt = &linv * &kmat * &linv;
        kt = (&kt + kt.transpose()) * 0.5;
        let eig = kt.symmetric_eigen();
        let mut idx: Vec<usize> = (0..n).collect();
        idx.sort_by(|&i, &j| eig.eigenvalues[i].partial_cmp(&eig.eigenvalues[j]).unwrap());
        let u = &linv * &eig.eigenvectors;
        let mut vmod = DMatrix::<f64>::zeros(n, p.nmodes);
        let mut freqs = vec![0.0; p.nmodes];
        for k in 0..p.nmodes {
            for r in 0..n {
                vmod[(r, k)] = u[(n - 1 - r, idx[k])];
            }
            freqs[k] = eig.eigenvalues[idx[k]].sqrt() / 2.0 / std::f64::consts::PI;
        }
        (vmod, freqs)
    }

    pub fn configforcescaler(&mut self, forcescl: f64, magnetdist: f64) {
        self.forcescaler1 = forcescl;
        self.forcescaler = forcescl / (magnetdist * 1000.0).powi(2);
        self.magnetdist = magnetdist;
    }

    pub fn setforce(&mut self, pos: usize, val: f64) {
        self.f[pos] = self.forcescaler * val;
    }

    pub fn setforcenl(&mut self, pos: usize, val: f64) {
        self.f[pos] = self.forcescaler1 * val / ((self.magnetdist + self.x[pos]) * 1000.0).powi(2);
    }

    pub fn getaccel(&self, pos: usize) -> f64 {
        let noise: f64 = rand::rng().sample::<f64, _>(StandardNormal) * self.noisestd;
        let a = self.a[pos] + noise;
        if self.accel_in_g { a / 9.80665 } else { a }
    }

    pub fn reset(&mut self) {
        let n = self.npoints;
        self.f = DVector::zeros(n);
        self.x = DVector::zeros(n);
        self.a = DVector::zeros(n);
        self.xiir = vec![[0.0; 3]; self.nmodes];
        self.yiir = vec![[0.0; 3]; self.nmodes];
        self.bufdesloc = DVector::zeros(n);
        self.bufvel = [DVector::zeros(n), DVector::zeros(n)];
        self.rotvel = DVector::zeros(n);
    }

    pub fn update(&mut self) {
        self.bufvel[1] = self.bufvel[0].clone();
        self.bufdesloc = self.x.clone();
        let mut x = DVector::zeros(self.npoints);
        for k in 0..self.nmodes {
            let mode = self.vmod.column(k);
            self.xiir[k] = [mode.dot(&self.f), self.xiir[k][0], self.xiir[k][1]];
            let b = &self.biir[k];
            let a = &self.aiir[k];
            let xi = &self.xiir[k];
            let y0 = b[0] * xi[0] + b[1] * xi[1] + b[2] * xi[2]
                - (a[0] * self.yiir[k][0] + a[1] * self.yiir[k][1]);
            self.yiir[k] = [y0, self.yiir[k][0], self.yiir[k][1]];
            x += mode * (self.ts / (MASS * self.wd[k]) * y0);
        }
        self.x = x;
        self.bufvel[0] = (&self.x - &self.bufdesloc) * self.fs;
        self.a = (&self.bufvel[0] - &self.bufvel[1]) * self.fs;
        for i in 1..self.npoints {
            self.rotvel[i] = (self.bufvel[0][i] - self.bufvel[0][i - 1]) * self.rotvelmultiplier;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_mode_frequency_is_plausible() {
        // Analytical first-mode frequency of a cantilever: f1 = (1.875^2 / 2π) sqrt(EI / (ρ A L^4))
        let p = BeamParams {
            nmodes: 2,
            ..Default::default()
        };
        let beam = CantileverBeam::new(p.clone());
        let i = p.width * p.thickness.powi(3) / 12.0;
        let area = p.width * p.thickness;
        let f1 = 1.875f64.powi(2) / (2.0 * std::f64::consts::PI)
            * (p.elasticmod * i / (p.density * area * p.length.powi(4))).sqrt();
        assert!(
            (beam.freqs_hz[0] - f1).abs() / f1 < 0.05,
            "{} vs {}",
            beam.freqs_hz[0],
            f1
        );
        assert!(beam.freqs_hz[1] > beam.freqs_hz[0]);
    }

    #[test]
    fn beam_responds_to_force() {
        let mut beam = CantileverBeam::new(BeamParams {
            nmodes: 2,
            ..Default::default()
        });
        beam.setforce(30, 1.0);
        for _ in 0..10 {
            beam.update();
        }
        assert!(beam.x.iter().any(|v| v.abs() > 0.0));
    }
}
