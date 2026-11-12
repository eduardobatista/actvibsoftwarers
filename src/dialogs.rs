//! Tool dialogs: path modeling, data viewer, additional setup and workdir
//! manager (port of `dialogs.py`).

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use crossbeam_channel::{Receiver, bounded};
use egui::{Color32, Ui};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use serde::{Deserialize, Serialize};

use crate::dataman::Shared;
use crate::driver::proto;
use crate::dsp::{CompressionParams, FirNlms, compress_path, easy_fourier};
use crate::figures::{BLUE, RED};
use crate::table::{Column, Table, fmt_f64};

/// A computation running on a worker thread, polled from the UI.
pub struct BgTask<T> {
    rx: Receiver<T>,
}

impl<T: Send + 'static> BgTask<T> {
    pub fn spawn(ctx: &egui::Context, f: impl FnOnce() -> T + Send + 'static) -> Self {
        let (tx, rx) = bounded(1);
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let _ = tx.send(f());
            ctx.request_repaint();
        });
        Self { rx }
    }

    pub fn poll(&self) -> Option<T> {
        self.rx.try_recv().ok()
    }
}

pub struct PlotSeries {
    pub name: String,
    pub color: Color32,
    pub width: f32,
    pub points: Vec<[f64; 2]>,
}

pub fn ps(name: &str, color: Color32, width: f32, points: Vec<[f64; 2]>) -> PlotSeries {
    PlotSeries {
        name: name.into(),
        color,
        width,
        points,
    }
}

pub fn indexed(y: &[f64]) -> Vec<[f64; 2]> {
    y.iter().enumerate().map(|(i, v)| [i as f64, *v]).collect()
}

pub fn xy(x: &[f64], y: &[f64]) -> Vec<[f64; 2]> {
    x.iter()
        .zip(y)
        .filter(|(a, b)| a.is_finite() && b.is_finite())
        .map(|(a, b)| [*a, *b])
        .collect()
}

/// Auto-ranging plot with legend.
pub fn simple_plot(
    ui: &mut Ui,
    id: impl egui::AsIdSalt,
    height: f32,
    xlabel: &str,
    series: &[PlotSeries],
) {
    let mut plot = Plot::new(id).height(height).x_axis_label(xlabel);
    if series.iter().any(|s| !s.name.is_empty()) {
        plot = plot.legend(Legend::default());
    }
    plot.show(ui, |pui| {
        for s in series {
            pui.line(
                Line::new(s.name.clone(), PlotPoints::new(s.points.clone()))
                    .color(s.color)
                    .width(s.width),
            );
        }
    });
}

pub fn pick_feather(dir: Option<&Path>) -> Option<PathBuf> {
    let mut d = rfd::FileDialog::new().add_filter("feather", &["feather"]);
    if let Some(dir) = dir.filter(|p| p.is_dir()) {
        d = d.set_directory(dir);
    } else if let Some(home) = directories::UserDirs::new() {
        d = d.set_directory(home.home_dir());
    }
    d.pick_file()
}

// ------------------------------------------------------------ Path modeling

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PathModelingOptions {
    pub starttime: f64,
    pub endtime: f64,
    pub memsize: usize,
    pub stepsize: f64,
    pub penalization: f64,
    pub averaging: usize,
    pub source: usize,
    pub svdchecksec: bool,
    pub svdcheckfbk: bool,
    pub svdbranchessec: usize,
    pub svdbranchesfbk: usize,
    pub svdsparsitysec: usize,
    pub svdsparsityfbk: usize,
    pub lastfile: String,
}

impl Default for PathModelingOptions {
    fn default() -> Self {
        Self {
            starttime: 0.0,
            endtime: 0.0,
            memsize: 1000,
            stepsize: 0.05,
            penalization: 1e-2,
            averaging: 250,
            source: 0,
            svdchecksec: false,
            svdcheckfbk: false,
            svdbranchessec: 5,
            svdbranchesfbk: 5,
            svdsparsitysec: 10,
            svdsparsityfbk: 10,
            lastfile: String::new(),
        }
    }
}

enum PmPlot {
    None,
    Paths,
    Check { wsec: Vec<f64>, wfbk: Vec<f64> },
    SValues { name: String, values: Vec<f64> },
}

/// Secondary and feedback path weights.
type PathPair = (Vec<f64>, Vec<f64>);

pub struct PathModelingDialog {
    pub open: bool,
    pub opts: PathModelingOptions,
    step_text: String,
    pen_text: String,
    endtime_max: f64,
    use_feedback_basis: bool,
    use_secondary_basis: bool,
    paths: Option<(Vec<f64>, Vec<f64>)>,
    fft: Option<[(Vec<f64>, Vec<f64>); 2]>,
    plot: PmPlot,
    status: String,
    upload_enabled: bool,
    progress: Arc<Mutex<(usize, usize)>>,
    upload_task: Option<BgTask<Result<()>>>,
    check_task: Option<BgTask<Result<PathPair>>>,
}

impl PathModelingDialog {
    pub fn new(opts: Option<PathModelingOptions>, shared: &Shared) -> Self {
        let data = shared.data.read().unwrap();
        let endtime = data.time.last().copied().unwrap_or(0.0).ceil();
        let starttime = if endtime > 120.0 {
            20.0
        } else if endtime > 60.0 {
            10.0
        } else if endtime > 10.0 {
            2.0
        } else {
            0.0
        };
        let mut o = PathModelingOptions {
            starttime,
            endtime,
            ..Default::default()
        };
        if let Some(saved) = opts {
            o = PathModelingOptions {
                endtime: saved.endtime.min(endtime),
                ..saved
            };
            if saved.endtime == 0.0 {
                o.endtime = endtime;
            }
        }
        o.starttime = o.starttime.min(endtime);
        o.endtime = o.endtime.min(endtime);
        Self {
            open: true,
            step_text: fmt_f64(o.stepsize),
            pen_text: fmt_f64(o.penalization),
            opts: o,
            endtime_max: endtime,
            use_feedback_basis: false,
            use_secondary_basis: false,
            paths: None,
            fft: None,
            plot: PmPlot::None,
            status: String::new(),
            upload_enabled: false,
            progress: Arc::new(Mutex::new((0, 1))),
            upload_task: None,
            check_task: None,
        }
    }

    /// Options to persist (stepsize/penalization parsed from the text fields).
    pub fn options_to_save(&self) -> PathModelingOptions {
        let mut o = self.opts.clone();
        o.stepsize = self.step_text.trim().parse().unwrap_or(o.stepsize);
        o.penalization = self.pen_text.trim().parse().unwrap_or(o.penalization);
        o
    }

    fn set_paths(&mut self, sec: Vec<f64>, fbk: Vec<f64>, fs: f64) {
        let f1 = easy_fourier(&sec, fs);
        let f2 = easy_fourier(&fbk, fs);
        self.fft = Some([(f1.1, f1.0), (f2.1, f2.0)]);
        self.paths = Some((sec, fbk));
        self.plot = PmPlot::Paths;
    }

    fn open_file(&mut self) {
        if let Some(p) = pick_feather(None) {
            self.opts.lastfile = p.to_string_lossy().into_owned();
            match Table::read_feather(&p) {
                Ok(t) if t.has("ctrl") => {
                    let end = t
                        .f64_col("time")
                        .ok()
                        .and_then(|v| v.last().copied())
                        .unwrap_or(0.0);
                    self.endtime_max = end;
                    self.opts.endtime = end;
                    self.opts.starttime = self.opts.starttime.min(end);
                    self.opts.source = 1;
                }
                Ok(_) => self.status = "File not valid (Error 1).".into(),
                Err(e) => self.status = format!("Error: {e}"),
            }
        }
    }

    fn compression(&self, is_sec: bool) -> CompressionParams {
        CompressionParams {
            mem: self.opts.memsize,
            branches: if is_sec {
                self.opts.svdbranchessec
            } else {
                self.opts.svdbranchesfbk
            },
            sparsity: if is_sec {
                self.opts.svdsparsitysec
            } else {
                self.opts.svdsparsityfbk
            },
            use_other_basis: if is_sec {
                self.use_feedback_basis
            } else {
                self.use_secondary_basis
            },
        }
    }

    fn run_modeling(&mut self, shared: &Shared) -> Result<()> {
        self.paths = None;
        self.upload_enabled = false;
        let psi: f64 = self
            .pen_text
            .trim()
            .parse()
            .map_err(|_| anyhow!("Invalid penalization"))?;
        let mu: f64 = self
            .step_text
            .trim()
            .parse()
            .map_err(|_| anyhow!("Invalid step size"))?;
        let n = self.opts.memsize;
        let navg = self.opts.averaging;
        let (x, dfbk, dsec, fs);
        if self.opts.source == 0 {
            let data = shared.data.read().unwrap();
            if !(data.ctrlmode && !data.taskisctrl && !data.is_empty()) {
                bail!("Path modeling recording not found");
            }
            let (s, e) = (self.opts.starttime, self.opts.endtime);
            let idx: Vec<usize> = (0..data.len())
                .filter(|&i| data.time[i] >= s && data.time[i] <= e)
                .collect();
            x = idx.iter().map(|&i| data.dac[1][i]).collect::<Vec<_>>();
            dfbk = idx.iter().map(|&i| data.xref[i]).collect::<Vec<_>>();
            dsec = idx.iter().map(|&i| data.xerr[i]).collect::<Vec<_>>();
            fs = 1.0 / data.samplingperiod;
        } else {
            if self.opts.lastfile.is_empty() {
                bail!("No file selected (Error 2)");
            }
            let t = Table::read_feather(Path::new(&self.opts.lastfile))?;
            if !t.has("ctrl") {
                bail!("File not valid (Error 1)");
            }
            let time = t.f64_col("time")?;
            let end = time.last().copied().unwrap_or(0.0);
            self.endtime_max = end;
            self.opts.endtime = self.opts.endtime.min(end);
            let (s, e) = (self.opts.starttime, self.opts.endtime);
            let mask: Vec<bool> = time.iter().map(|v| *v >= s && *v <= e).collect();
            let pick = |name: &str| -> Result<Vec<f64>> {
                Ok(t.f64_col(name)?
                    .into_iter()
                    .zip(&mask)
                    .filter(|(_, m)| **m)
                    .map(|(v, _)| v)
                    .collect())
            };
            x = pick("ctrl")?;
            dfbk = pick("ref")?;
            dsec = pick("err")?;
            fs = 1.0 / shared.data.read().unwrap().samplingperiod;
        }
        let mut f1 = FirNlms::new(n, mu, psi, Some(navg));
        f1.run(&x, &dfbk)?;
        let mut f2 = FirNlms::new(n, mu, psi, Some(navg));
        f2.run(&x, &dsec)?;
        self.set_paths(f2.wwavg, f1.wwavg, fs);
        self.upload_enabled = true;
        Ok(())
    }

    fn save_paths(&mut self) {
        let Some((sec, fbk)) = &self.paths else {
            self.status = "No paths in memory to be saved.".into();
            return;
        };
        self.status.clear();
        let Some(p) = rfd::FileDialog::new()
            .add_filter("feather", &["feather"])
            .set_file_name("paths.feather")
            .save_file()
        else {
            return;
        };
        let p = if p.extension().is_none() {
            p.with_extension("feather")
        } else {
            p
        };
        let mut t = Table::new();
        t.push("wsec", Column::F64(sec.clone()));
        t.push("wfbk", Column::F64(fbk.clone()));
        self.status = match t.write_feather(&p) {
            Ok(()) => "File saved successfully!".into(),
            Err(e) => format!("Error: {e}"),
        };
    }

    fn load_paths(&mut self, fs: f64) {
        let Some(p) = pick_feather(None) else { return };
        let r = (|| -> Result<(Vec<f64>, Vec<f64>)> {
            let t = Table::read_feather(&p)?;
            if t.names.first().map(String::as_str) != Some("wsec")
                || t.names.get(1).map(String::as_str) != Some("wfbk")
            {
                bail!("Invalid dataframe.");
            }
            Ok((t.f64_col("wsec")?, t.f64_col("wfbk")?))
        })();
        match r {
            Ok((s, f)) => {
                self.set_paths(s, f, fs);
                self.upload_enabled = true;
                self.status = "Paths successfully loaded into memory.".into();
            }
            Err(e) => self.status = format!("Error: {e}"),
        }
    }

    fn start_check(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        self.status = "Wait... May take some time...".into();
        let sh = shared.clone();
        self.check_task = Some(BgTask::spawn(ctx, move || {
            sh.with_link(|link| {
                proto::handshake(link)?;
                let paths = proto::read_paths(link)?;
                proto::stop_readings(link)?;
                Ok(paths)
            })
        }));
    }

    fn start_upload(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        self.status.clear();
        let Some((sec, fbk)) = self.paths.clone() else {
            self.status = "Error: Path data not found in memory.".into();
            return;
        };
        let prep = |is_sec: bool, checked: bool| -> Result<(u8, Vec<f64>, Option<Vec<u8>>)> {
            let plain = if is_sec {
                (b's', sec.clone())
            } else {
                (b'f', fbk.clone())
            };
            if checked {
                let c = compress_path(&sec, &fbk, is_sec, self.compression(is_sec))?;
                Ok((
                    if is_sec { b'S' } else { b'F' },
                    c.weights,
                    Some(c.leading_bytes.to_vec()),
                ))
            } else {
                Ok((plain.0, plain.1, None))
            }
        };
        let jobs = match (
            prep(true, self.opts.svdchecksec),
            prep(false, self.opts.svdcheckfbk),
        ) {
            (Ok(a), Ok(b)) => [a, b],
            (Err(e), _) | (_, Err(e)) => {
                self.status = format!("Error: {e}");
                return;
            }
        };
        self.upload_enabled = false;
        let sh = shared.clone();
        let progress = self.progress.clone();
        self.upload_task = Some(BgTask::spawn(ctx, move || {
            sh.close_link();
            std::thread::sleep(Duration::from_millis(200));
            sh.with_link(|link| {
                for (tipo, data, leading) in &jobs {
                    proto::handshake(link)?;
                    let p = progress.clone();
                    proto::upload_path(link, *tipo, data, leading.as_deref(), |a, b| {
                        *p.lock().unwrap() = (a, b.max(1));
                    })?;
                }
                proto::stop_readings(link)?;
                proto::handshake(link)?;
                proto::write_flash(link)?;
                proto::stop_readings(link)?;
                Ok(())
            })
        }));
    }

    fn plot_svalues(&mut self, is_sec: bool) {
        let r = (|| -> Result<Vec<f64>> {
            let (sec, fbk) = self
                .paths
                .as_ref()
                .ok_or_else(|| anyhow!("Path data not found in memory."))?;
            Ok(compress_path(sec, fbk, is_sec, self.compression(is_sec))?.singular_values)
        })();
        match r {
            Ok(values) => {
                let name = if is_sec { "sec" } else { "fbk" };
                self.plot = PmPlot::SValues {
                    name: format!("S-values of {name} path"),
                    values,
                };
                self.status = "S-values plotted.".into();
            }
            Err(e) => self.status = format!("Error: {e}"),
        }
    }

    pub fn show(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        if let Some(t) = &self.upload_task {
            if let Some(r) = t.poll() {
                self.status = match r {
                    Ok(()) => "Successfully uploaded and written in flash!".into(),
                    Err(e) => format!("Error: {e}"),
                };
                self.upload_task = None;
                self.upload_enabled = true;
            } else {
                ctx.request_repaint_after(Duration::from_millis(100));
            }
        }
        if let Some(t) = &self.check_task
            && let Some(r) = t.poll()
        {
            match r {
                Ok((wsec, wfbk)) => {
                    self.plot = PmPlot::Check { wsec, wfbk };
                    self.status = "Paths successfully read.".into();
                }
                Err(e) => self.status = format!("Error: {e}"),
            }
            self.check_task = None;
        }
        let fs = 1.0 / shared.data.read().unwrap().samplingperiod;
        let busy = self.upload_task.is_some() || self.check_task.is_some();
        let mut open = self.open;
        egui::Window::new("Path Modeling")
            .open(&mut open)
            .default_size([900.0, 620.0])
            .show(ctx, |ui| {
                ui.horizontal_top(|ui| {
                    ui.vertical(|ui| {
                        ui.set_width(330.0);
                        self.controls_ui(ui, ctx, shared, fs, busy);
                    });
                    ui.separator();
                    ui.vertical(|ui| self.plots_ui(ui));
                });
            });
        self.open = open;
    }

    fn controls_ui(
        &mut self,
        ui: &mut Ui,
        ctx: &egui::Context,
        shared: &Arc<Shared>,
        fs: f64,
        busy: bool,
    ) {
        use egui::DragValue;
        ui.group(|ui| {
            ui.strong("Modeling");
            egui::Grid::new("pm_grid").num_columns(2).show(ui, |ui| {
                ui.label("Source:");
                crate::panels::combo(
                    ui,
                    "pmsource",
                    &mut self.opts.source,
                    &["Present Recording", "File"],
                    None,
                );
                ui.end_row();
                ui.label("File:");
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.opts.lastfile).desired_width(200.0),
                    );
                    if ui.button("...").clicked() {
                        self.open_file();
                    }
                });
                ui.end_row();
                ui.label("Start time:");
                ui.add(
                    DragValue::new(&mut self.opts.starttime)
                        .range(0.0..=self.endtime_max)
                        .speed(0.1)
                        .suffix(" s"),
                );
                ui.end_row();
                ui.label("End time:");
                ui.add(
                    DragValue::new(&mut self.opts.endtime)
                        .range(0.0..=self.endtime_max)
                        .speed(0.1)
                        .suffix(" s"),
                );
                ui.end_row();
                ui.label("Memory size:");
                ui.add(DragValue::new(&mut self.opts.memsize).range(1..=3000));
                ui.end_row();
                ui.label("NLMS step-size:");
                ui.text_edit_singleline(&mut self.step_text);
                ui.end_row();
                ui.label("NLMS penalization:");
                ui.text_edit_singleline(&mut self.pen_text);
                ui.end_row();
                ui.label("Weight averaging:");
                ui.add(DragValue::new(&mut self.opts.averaging).range(1..=1000));
                ui.end_row();
            });
            if ui.add_enabled(!busy, egui::Button::new("Run...")).clicked() {
                self.status = match self.run_modeling(shared) {
                    Ok(()) => "Success! Paths are available in memory for upload.".into(),
                    Err(e) => format!("Error: {e}."),
                };
            }
        });
        ui.group(|ui| {
            ui.strong("SVD Compression");
            egui::Grid::new("svd_grid").num_columns(1).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.opts.svdchecksec, "Secondary path with:");
                    ui.label("Branches:");
                    ui.add(DragValue::new(&mut self.opts.svdbranchessec).range(1..=10));
                    ui.label("Sparsity:");
                    ui.add(DragValue::new(&mut self.opts.svdsparsitysec).range(1..=1000));
                });
                ui.end_row();
                ui.horizontal(|ui| {
                    if ui
                        .checkbox(&mut self.use_feedback_basis, "Use feedback path basis.")
                        .changed()
                        && self.use_feedback_basis
                    {
                        self.use_secondary_basis = false;
                    }
                    if ui.button("Singular Values").clicked() {
                        self.plot_svalues(true);
                    }
                });
                ui.end_row();
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.opts.svdcheckfbk, "Feedback path with:");
                    ui.label("Branches:");
                    ui.add(DragValue::new(&mut self.opts.svdbranchesfbk).range(1..=10));
                    ui.label("Sparsity:");
                    ui.add(DragValue::new(&mut self.opts.svdsparsityfbk).range(1..=1000));
                });
                ui.end_row();
                ui.horizontal(|ui| {
                    if ui
                        .checkbox(&mut self.use_secondary_basis, "Use secondary path basis.")
                        .changed()
                        && self.use_secondary_basis
                    {
                        self.use_feedback_basis = false;
                    }
                    if ui.button("Singular Values").clicked() {
                        self.plot_svalues(false);
                    }
                });
                ui.end_row();
            });
        });
        if ui
            .add_enabled(!busy, egui::Button::new("Save paths to file..."))
            .clicked()
        {
            self.save_paths();
        }
        if ui
            .add_enabled(!busy, egui::Button::new("Load paths from file ..."))
            .clicked()
        {
            self.load_paths(fs);
        }
        if ui
            .add_enabled(
                self.upload_enabled && !busy,
                egui::Button::new("Upload and Record in Flash..."),
            )
            .clicked()
        {
            self.start_upload(ctx, shared);
        }
        if ui
            .add_enabled(!busy, egui::Button::new("Check Recorded Paths"))
            .clicked()
        {
            self.start_check(ctx, shared);
        }
        let (a, b) = *self.progress.lock().unwrap();
        ui.add(egui::ProgressBar::new(a as f32 / b.max(1) as f32).show_percentage());
        ui.label(&self.status);
    }

    fn plots_ui(&mut self, ui: &mut Ui) {
        let h = ((ui.available_height() - 10.0) / 2.0).max(120.0);
        match &self.plot {
            PmPlot::None => {
                ui.label("Run the modeling or load paths to plot them.");
            }
            PmPlot::Paths => {
                if let (Some((sec, fbk)), Some(fft)) = (&self.paths, &self.fft) {
                    simple_plot(
                        ui,
                        "pm_paths",
                        h,
                        "Coefficient",
                        &[
                            ps("Secondary path", RED, 1.0, indexed(sec)),
                            ps("Feedback path", BLUE, 1.0, indexed(fbk)),
                        ],
                    );
                    simple_plot(
                        ui,
                        "pm_fft",
                        h,
                        "Frequency (Hz)",
                        &[
                            ps("Secondary path (dB)", RED, 1.0, xy(&fft[0].0, &fft[0].1)),
                            ps("Feedback path (dB)", BLUE, 1.0, xy(&fft[1].0, &fft[1].1)),
                        ],
                    );
                }
            }
            PmPlot::Check { wsec, wfbk } => {
                let mut s1 = vec![ps("Sec. Path from Device", RED, 2.0, indexed(wsec))];
                let mut s2 = vec![ps("Feedback Path from Device", BLUE, 2.0, indexed(wfbk))];
                if let Some((sec, fbk)) = &self.paths {
                    s1.push(ps(
                        "Sec. Path from Memory",
                        RED.gamma_multiply(0.5),
                        1.0,
                        indexed(sec),
                    ));
                    s2.push(ps(
                        "Feedback Path from Memory",
                        BLUE.gamma_multiply(0.5),
                        1.0,
                        indexed(fbk),
                    ));
                }
                simple_plot(ui, "pm_check1", h, "Coefficient", &s1);
                simple_plot(ui, "pm_check2", h, "Coefficient", &s2);
            }
            PmPlot::SValues { name, values } => {
                simple_plot(
                    ui,
                    "pm_svalues",
                    h * 2.0,
                    "Index",
                    &[ps(name, RED, 2.0, indexed(values))],
                );
            }
        }
    }
}

// --------------------------------------------------------------- Data viewer

pub struct DataViewer {
    pub open: bool,
    file_name: String,
    log_text: String,
    status: String,
    series: Option<[Vec<PlotSeries>; 2]>,
}

impl DataViewer {
    pub fn new() -> Self {
        Self {
            open: true,
            file_name: String::new(),
            log_text: String::new(),
            status: String::new(),
            series: None,
        }
    }

    pub fn open_file(&mut self, path: &Path) {
        self.status.clear();
        self.file_name = path.to_string_lossy().into_owned();
        let r = (|| -> Result<()> {
            let t = Table::read_feather(path)?;
            if let (Some(time), Some(log)) = (t.get("time"), t.get("log")) {
                let time = time.as_f64();
                let lines: Vec<String> = (0..t.nrows())
                    .filter_map(|i| match log {
                        Column::Str(v) => {
                            v[i].as_ref().map(|s| format!("{}: {s}", fmt_f64(time[i])))
                        }
                        _ => None,
                    })
                    .collect();
                self.log_text = lines.join("\n");
            } else {
                self.log_text.clear();
            }
            if !t.has("ctrl") {
                self.series = None;
                bail!("File not valid (Error 1).");
            }
            let time = t.f64_col("time")?;
            self.series = Some([
                vec![
                    ps("Perturbation", RED, 1.0, xy(&time, &t.f64_col("perturb")?)),
                    ps("Control", BLUE, 1.0, xy(&time, &t.f64_col("ctrl")?)),
                ],
                vec![
                    ps("Error", RED, 1.0, xy(&time, &t.f64_col("err")?)),
                    ps("Reference", BLUE, 1.0, xy(&time, &t.f64_col("ref")?)),
                ],
            ]);
            Ok(())
        })();
        if let Err(e) = r {
            self.status = if e.to_string().starts_with("File not valid") {
                e.to_string()
            } else {
                format!("Error: {e}")
            };
        }
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        shared: &Shared,
        last_folder: &mut Option<PathBuf>,
    ) {
        // Drag & drop of feather files.
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .collect()
        });
        if let Some(p) = dropped.first() {
            if p.extension()
                .map(|e| e.eq_ignore_ascii_case("feather"))
                .unwrap_or(false)
            {
                self.open_file(p);
            } else {
                self.status = "File is not in feather format.".into();
            }
        }
        let mut open = self.open;
        egui::Window::new("Data Viewer")
            .open(&mut open)
            .default_size([900.0, 620.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("File:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.file_name)
                            .desired_width(400.0)
                            .interactive(false),
                    );
                    if ui.button("...").clicked()
                        && let Some(p) = pick_feather(last_folder.as_deref())
                    {
                        *last_folder = p.parent().map(Path::to_path_buf);
                        self.open_file(&p);
                    }
                    if ui.button("Data In Memory").clicked() {
                        let d = shared.data.read().unwrap();
                        if d.is_empty() {
                            self.status = "Recorded data not found...".into();
                        } else if !d.ctrlmode {
                            self.status = "Only Control Mode data can be viewed for now.".into();
                        } else {
                            let n = d.len();
                            self.series = Some([
                                vec![
                                    ps("Perturbation", RED, 1.0, xy(&d.time, &d.dac[0][..n])),
                                    ps("Control", BLUE, 1.0, xy(&d.time, &d.dac[1][..n])),
                                ],
                                vec![
                                    ps("Error", RED, 1.0, xy(&d.time, &d.xerr[..n])),
                                    ps("Reference", BLUE, 1.0, xy(&d.time, &d.xref[..n])),
                                ],
                            ]);
                            self.status.clear();
                        }
                    }
                });
                ui.label(egui::RichText::new(&self.status).color(Color32::DARK_RED));
                ui.horizontal_top(|ui| {
                    ui.vertical(|ui| {
                        ui.set_width(260.0);
                        ui.label("Log:");
                        egui::ScrollArea::vertical()
                            .id_salt("dv_log")
                            .show(ui, |ui| {
                                ui.add(
                                    egui::TextEdit::multiline(&mut self.log_text.as_str())
                                        .desired_width(250.0)
                                        .font(egui::TextStyle::Monospace),
                                );
                            });
                    });
                    ui.vertical(|ui| {
                        if let Some([s1, s2]) = &self.series {
                            let h = ((ui.available_height() - 10.0) / 2.0).max(120.0);
                            simple_plot(ui, "dv_plot1", h, "Time (s)", s1);
                            simple_plot(ui, "dv_plot2", h, "Time (s)", s2);
                        } else {
                            ui.label(
                                "Open a feather file (or drop one here) to view control data.",
                            );
                        }
                    });
                });
            });
        self.open = open;
    }
}

impl Default for DataViewer {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------- Additional system setup

pub fn coefs_to_string(c: &[f64]) -> String {
    format!(
        "[{}]",
        c.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(" ")
    )
}

pub fn parse_coefs(s: &str) -> Result<Vec<f64>> {
    let inner = s.trim().trim_start_matches('[').trim_end_matches(']');
    let v: Vec<f64> = inner
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
        .map(|t| {
            t.parse::<f64>()
                .map_err(|_| anyhow!("Invalid coefficient '{t}'."))
        })
        .collect::<Result<_>>()?;
    if v.is_empty() {
        bail!("Must be an array.");
    }
    if v.len() > 10 {
        bail!("Maximum order is 9.");
    }
    Ok(v)
}

pub struct AdditionalDialog {
    pub open: bool,
    texts: [String; 4],
    checks: [bool; 4],
    weights: [f64; 2],
    status: String,
    task: Option<BgTask<Result<()>>>,
}

impl AdditionalDialog {
    pub fn new(shared: &Shared) -> Self {
        let cfg = shared.cfg.lock().unwrap();
        Self {
            open: true,
            texts: std::array::from_fn(|k| coefs_to_string(&cfg.predistcoefs[k])),
            checks: cfg.predistenablemap,
            weights: cfg.fusionweights,
            status: String::new(),
            task: None,
        }
    }

    fn check_and_save(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        self.status.clear();
        {
            let mut cfg = shared.cfg.lock().unwrap();
            for k in 0..4 {
                cfg.predistenablemap[k] = self.checks[k];
                match parse_coefs(&self.texts[k]) {
                    Ok(c) => cfg.predistcoefs[k] = c,
                    Err(e) => {
                        cfg.predistenablemap[k] = false;
                        cfg.predistcoefs[k] = vec![1.0, 0.0];
                        self.status = format!("Error: {e}");
                        return;
                    }
                }
            }
            cfg.fusionweights = self.weights;
        }
        let sh = shared.clone();
        self.status = "Writing to device...".into();
        self.task = Some(BgTask::spawn(ctx, move || {
            let cfg = sh.cfg.lock().unwrap().clone();
            sh.with_link(|link| {
                proto::handshake(link)?;
                for k in 0..4 {
                    proto::write_predist_config(link, &cfg, k)?;
                }
                proto::write_fusion_config(link, cfg.fusionweights)?;
                proto::record_additional_configs(link)
            })
        }));
    }

    pub fn show(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        if let Some(t) = &self.task
            && let Some(r) = t.poll()
        {
            self.status = match r {
                Ok(()) => "Coefficients checked, saved and written to device.".into(),
                Err(e) => format!("Error: {e}"),
            };
            self.task = None;
        }
        let mut open = self.open;
        egui::Window::new("Additional System Setup").open(&mut open).default_width(560.0).show(ctx, |ui| {
            ui.label("Predistortion polynomials.");
            ui.label(
                "Input must be an array containing the polynomial coefficients [a(k) a(k-1) ... a(0)], \
                 where y = a(k)*x^k + a(k-1)*x^(k-1) + ... + a(1)*x + a(0). \
                 Thus, for a K-th order polynomial, K+1 coefficients are required (maximum order is 9).",
            );
            egui::Grid::new("predist_grid").num_columns(2).show(ui, |ui| {
                for k in 0..4 {
                    ui.checkbox(&mut self.checks[k], format!("Enable for Output {}:", k + 1));
                    ui.add(egui::TextEdit::singleline(&mut self.texts[k]).desired_width(360.0));
                    ui.end_row();
                }
            });
            ui.separator();
            ui.label("Sensor fusion weights:");
            ui.horizontal(|ui| {
                ui.label("Weight for Sensor 1:");
                ui.add(egui::DragValue::new(&mut self.weights[0]).range(0.0..=1000.0).speed(0.01).fixed_decimals(3));
                ui.label("Weight for Sensor 2:");
                ui.add(egui::DragValue::new(&mut self.weights[1]).range(0.0..=1000.0).speed(0.01).fixed_decimals(3));
            });
            ui.horizontal(|ui| {
                if ui.add_enabled(self.task.is_none(), egui::Button::new("Check and Save")).clicked() {
                    self.check_and_save(ctx, shared);
                }
                ui.label(&self.status);
            });
        });
        self.open = open;
    }
}

// ----------------------------------------------------------- Workdir manager

pub struct WorkdirManager {
    pub open: bool,
    workdir: PathBuf,
    table: Option<Table>,
    selected: Vec<bool>,
    status: String,
}

impl WorkdirManager {
    pub fn new(workdir: &Path) -> Self {
        let mut s = Self {
            open: true,
            workdir: workdir.to_path_buf(),
            table: None,
            selected: vec![],
            status: String::new(),
        };
        s.reload();
        s
    }

    fn metafile(&self) -> PathBuf {
        self.workdir.join("metacontrol.feather")
    }

    fn reload(&mut self) {
        let mf = self.metafile();
        if mf.exists() {
            match Table::read_feather(&mf) {
                Ok(t) => {
                    self.selected = vec![false; t.nrows()];
                    self.table = Some(t);
                }
                Err(e) => self.status = format!("Error: {e}"),
            }
        }
    }

    fn delete_selected(&mut self) -> Result<()> {
        let Some(t) = self.table.as_mut() else {
            return Ok(());
        };
        let rows: Vec<usize> = self
            .selected
            .iter()
            .enumerate()
            .filter(|(_, s)| **s)
            .map(|(i, _)| i)
            .collect();
        if rows.is_empty() {
            return Ok(());
        }
        if let Some(Column::Str(names)) = t.get("filename") {
            for &r in &rows {
                if let Some(Some(f)) = names.get(r) {
                    let p = self.workdir.join(f);
                    if p.exists() {
                        std::fs::remove_file(&p)?;
                    }
                }
            }
        }
        t.drop_rows(&rows);
        t.write_feather(&self.workdir.join("metacontrol.feather"))?;
        self.selected = vec![false; t.nrows()];
        Ok(())
    }

    pub fn show(&mut self, ctx: &egui::Context) {
        let mut open = self.open;
        egui::Window::new("Workdir Manager")
            .open(&mut open)
            .default_size([800.0, 400.0])
            .show(ctx, |ui| {
                ui.label(format!("Workdir: {}", self.workdir.display()));
                match &self.table {
                    None => {
                        ui.label("No metacontrol.feather found in the working directory.");
                    }
                    Some(t) => {
                        egui::ScrollArea::both()
                            .max_height(ui.available_height() - 40.0)
                            .show(ui, |ui| {
                                egui::Grid::new("wd_grid").striped(true).show(ui, |ui| {
                                    ui.label("");
                                    for n in &t.names {
                                        ui.strong(n);
                                    }
                                    ui.end_row();
                                    for r in 0..t.nrows() {
                                        ui.checkbox(&mut self.selected[r], "");
                                        for c in &t.columns {
                                            ui.label(c.cell_string(r));
                                        }
                                        ui.end_row();
                                    }
                                });
                            });
                        if ui.button("Delete data of the selected rows").clicked()
                            && let Err(e) = self.delete_selected()
                        {
                            self.status = format!("Error: {e}");
                        }
                    }
                }
                ui.label(&self.status);
            });
        self.open = open;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coefficient_parsing() {
        assert_eq!(parse_coefs("[1.0  0.5 -2]").unwrap(), vec![1.0, 0.5, -2.0]);
        assert_eq!(coefs_to_string(&[1.0, 0.0]), "[1.0 0.0]");
        assert!(parse_coefs("[]").is_err());
        assert!(parse_coefs("[1 2 3 4 5 6 7 8 9 10 11]").is_err());
        assert!(parse_coefs("[a b]").is_err());
    }
}
