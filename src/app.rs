//! Main window (port of `mainwindow.py`).

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Result, anyhow, bail};
use crossbeam_channel::{Receiver, Sender, unbounded};
use egui::{Color32, RichText, Ui};

use crate::automator::{ActionOutcome, Automator, Command};
use crate::dataman::{DataEvent, LogKind, Shared, start_acquisition};
use crate::dialogs::{
    AdditionalDialog, DataViewer, PathModelingDialog, PathModelingOptions, WorkdirManager,
};
use crate::driver::{BAUD_RATES, GeneratorParams, list_ports, port_vid};
use crate::figures::{CtrlFig, MainFig, OutFig};
use crate::panels::{
    ADC_MODELS, ADC_RANGES, ALGORITHMS, AdcPanel, ControlPanel, GEN_TYPES, GeneratorPanel,
    IMU_ADDRESSES, IMU_BUSES, IMU_TYPES, IMUS, ImuPanel, OUTPUTS, RATES_1015, RATES_1115, SENSORS,
    STEP_FIELD_ID,
};
use crate::settings::Settings;
use crate::table::{Column, Table};
use crate::updater::UpdaterDialog;

pub const MPU_RATES: [u32; 4] = [250, 333, 500, 1000];
pub const LSM_RATES: [u32; 4] = [104, 208, 416, 832];
pub const MAX_TIMES: [u32; 3] = [10, 20, 30];
const MAX_NOTE_LENGTH: usize = 1000;

enum Confirm {
    ResetUnsaved,
    QuitUnsaved,
}

pub struct ActVibApp {
    shared: Arc<Shared>,
    tx: Sender<DataEvent>,
    rx: Receiver<DataEvent>,
    // Setup
    port: Option<String>,
    fsampling: u32,
    tsampling_us: u32,
    maxtime: u32,
    baud: u32,
    workdir: PathBuf,
    last_data_folder: Option<PathBuf>,
    last_saved_ext: Option<String>,
    // Panels
    mode: usize,
    imu: [ImuPanel; 3],
    gens: [GeneratorPanel; 4],
    ctrl: ControlPanel,
    adc: AdcPanel,
    mainfig: MainFig,
    ctrlfig: CtrlFig,
    outfig: OutFig,
    notes: String,
    sensor_tab: usize,
    output_tab: usize,
    // State
    status: Option<String>,
    loglist: Vec<(f64, String)>,
    mode_locked: bool,
    start_busy: bool,
    flagsaveplus: bool,
    gen_applied: [Option<GeneratorParams>; 4],
    debug_mode: bool,
    // Automator
    automator: Automator,
    automator_wait_stop: bool,
    automator_wait_start: bool,
    // Dialogs
    path_modeling: Option<PathModelingDialog>,
    path_modeling_opts: Option<PathModelingOptions>,
    data_viewer: Option<DataViewer>,
    additional: Option<AdditionalDialog>,
    workdir_manager: Option<WorkdirManager>,
    updater: Option<UpdaterDialog>,
    confirm: Option<Confirm>,
    error_modal: Option<String>,
    allow_close: bool,
}

fn home() -> PathBuf {
    directories::UserDirs::new()
        .map(|u| u.home_dir().to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."))
}

impl ActVibApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.set_visuals(egui::Visuals::light());
        let s = Settings::load();
        let shared = Shared::new();
        let (tx, rx) = unbounded();
        {
            let mut cfg = shared.cfg.lock().unwrap();
            cfg.predistenablemap = s.predist_enable;
            cfg.predistcoefs = s.predist_coefs.clone();
            if cfg.predistcoefs.iter().any(|c| c.is_empty()) {
                cfg.predistcoefs = std::array::from_fn(|_| vec![1.0, 0.0]);
            }
            cfg.fusionweights = s.fusion_weights;
        }
        let workdir = s
            .workdir
            .clone()
            .filter(|p| p.is_dir())
            .unwrap_or_else(home);
        let mut app = Self {
            shared,
            tx,
            rx,
            port: None,
            fsampling: 250,
            tsampling_us: 4000,
            maxtime: 10,
            baud: crate::driver::DEFAULT_BAUD,
            workdir,
            last_data_folder: s.last_data_folder.clone(),
            last_saved_ext: s.last_saved_ext.clone(),
            mode: s.mode.min(1),
            imu: s.imu.clone(),
            gens: s.gens.clone(),
            ctrl: s.ctrl.clone(),
            adc: s.adc.clone(),
            mainfig: s.mainfig.clone(),
            ctrlfig: s.ctrlfig.clone(),
            outfig: s.outfig.clone(),
            notes: String::new(),
            sensor_tab: s.sensor_tab.min(3),
            output_tab: s.output_tab.min(3),
            status: None,
            loglist: vec![],
            mode_locked: false,
            start_busy: false,
            flagsaveplus: false,
            gen_applied: Default::default(),
            debug_mode: std::env::var("ACTVIB_DEBUG_MODE")
                .map(|v| v == "1")
                .unwrap_or(false),
            automator: Automator::new(),
            automator_wait_stop: false,
            automator_wait_start: false,
            path_modeling: None,
            path_modeling_opts: s.path_modeling.clone(),
            data_viewer: None,
            additional: None,
            workdir_manager: None,
            updater: None,
            confirm: None,
            error_modal: None,
            allow_close: false,
        };
        // Port: saved one if present, else a single CP210x/CH340 port, else the only port.
        let ports = list_ports();
        if let Some(p) = s
            .port
            .as_ref()
            .filter(|p| ports.iter().any(|i| &i.port_name == *p))
        {
            app.set_port(p.clone());
        } else {
            let preferred: Vec<_> = ports
                .iter()
                .filter(|p| matches!(port_vid(p), Some(0x10C4 | 0x1A86)))
                .collect();
            if preferred.len() == 1 {
                app.set_port(preferred[0].port_name.clone());
            } else if ports.len() == 1 {
                app.set_port(ports[0].port_name.clone());
            }
        }
        app.apply_sampling(s.fsampling, false);
        app.maxtime = if MAX_TIMES.contains(&s.maxtime) {
            s.maxtime
        } else {
            10
        };
        app.reset_data();
        app.baud = if BAUD_RATES.contains(&s.baud) {
            s.baud
        } else {
            crate::driver::DEFAULT_BAUD
        };
        app.shared.set_baud(app.baud);
        app.change_generator_config();
        app.change_imu_config();
        app.change_adc_config();
        app.config_control();
        app.mainfig.show_adc = app.shared.cfg.lock().unwrap().adc_active();
        app
    }

    fn running(&self) -> bool {
        self.shared.is_running()
    }

    fn settings(&self) -> Settings {
        let cfg = self.shared.cfg.lock().unwrap();
        Settings {
            port: self.port.clone(),
            fsampling: self.fsampling,
            maxtime: self.maxtime,
            baud: self.baud,
            workdir: Some(self.workdir.clone()),
            last_data_folder: self.last_data_folder.clone(),
            last_saved_ext: self.last_saved_ext.clone(),
            predist_enable: cfg.predistenablemap,
            predist_coefs: cfg.predistcoefs.clone(),
            fusion_weights: cfg.fusionweights,
            mode: self.mode,
            imu: self.imu.clone(),
            gens: self.gens.clone(),
            ctrl: self.ctrl.clone(),
            adc: self.adc.clone(),
            mainfig: self.mainfig.clone(),
            ctrlfig: self.ctrlfig.clone(),
            outfig: self.outfig.clone(),
            path_modeling: self
                .path_modeling
                .as_ref()
                .map(|p| p.options_to_save())
                .or(self.path_modeling_opts.clone()),
            sensor_tab: self.sensor_tab,
            output_tab: self.output_tab,
        }
    }

    fn set_status(&mut self, msg: impl Into<String>) {
        self.status = Some(msg.into());
    }

    // ------------------------------------------------------ configuration

    fn control_on(&self) -> bool {
        self.mode == 1
    }

    /// Whether generator panel `k` is enabled given the control configuration.
    fn gen_panel_enabled(&self, k: usize) -> bool {
        if !self.control_on() {
            return true;
        }
        let task_ctrl = self.ctrl.is_task_control();
        if k == self.ctrl.control_channel {
            !task_ctrl
        } else if k == self.ctrl.perturb_channel {
            task_ctrl
        } else {
            false
        }
    }

    fn output_tab_title(&self, k: usize) -> String {
        if self.control_on() {
            if k == self.ctrl.control_channel {
                return format!("Output {} (Control)", k + 1);
            } else if k == self.ctrl.perturb_channel {
                return format!("Output {} (Perturb.)", k + 1);
            }
        }
        format!("Output {}", k + 1)
    }

    fn config_control(&mut self) {
        let params = self.ctrl.params();
        {
            let mut cfg = self.shared.cfg.lock().unwrap();
            cfg.set_control_mode(self.control_on(), self.ctrl.task as u8, false);
            if cfg.control_mode {
                cfg.control_channel = self.ctrl.control_channel as u8;
                cfg.perturb_channel = self.ctrl.perturb_channel as u8;
                cfg.set_control_config(&params);
                cfg.debug_mode = self.debug_mode;
            }
        }
        self.change_generator_config();
    }

    fn change_generator_config(&mut self) {
        let params: Vec<GeneratorParams> = (0..4)
            .map(|k| self.gens[k].params(self.gen_panel_enabled(k)))
            .collect();
        let mut cfg = self.shared.cfg.lock().unwrap();
        for (k, p) in params.into_iter().enumerate() {
            if self.gen_applied[k].as_ref() != Some(&p) {
                cfg.set_generator_config(k, &p);
                self.gen_applied[k] = Some(p);
            }
        }
    }

    fn change_imu_config(&mut self) {
        let bytes: Vec<[u8; 3]> = self.imu.iter_mut().map(|i| i.config_bytes()).collect();
        let mut cfg = self.shared.cfg.lock().unwrap();
        for (k, b) in bytes.into_iter().enumerate() {
            cfg.set_imu_config(k, b);
        }
    }

    fn change_adc_config(&mut self) {
        self.shared
            .cfg
            .lock()
            .unwrap()
            .set_adc_config(self.adc.config());
    }

    fn plot_config(&mut self) {
        self.ctrlfig.sensor_ids = [self.ctrl.ref_sensor, self.ctrl.err_sensor];
        let enabled: Vec<bool> = (0..4)
            .map(|k| self.gens[k].is_on(self.gen_panel_enabled(k)))
            .collect();
        self.outfig.dacenable = if self.control_on() && self.ctrl.is_task_control() {
            [true, true, false, false]
        } else {
            [enabled[0], enabled[1], enabled[2], enabled[3]]
        };
    }

    fn config_alg_on(&mut self) {
        let mut cfg = self.shared.cfg.lock().unwrap();
        if self.ctrl.alg_on {
            cfg.ctrlmu = self.ctrl.mu() as f32;
            cfg.ctrlfi = self.ctrl.psi() as f32;
        }
        cfg.set_alg_on(self.ctrl.alg_on, self.ctrl.t_alg_on as f64, false);
    }

    // ------------------------------------------------------------ actions

    fn init_checks(&self) -> Result<()> {
        if self.port.is_none() {
            bail!("Select a serial port before starting the acquisition.");
        }
        if self.control_on() {
            if !self.imu[self.ctrl.imu_err].is_enabled() {
                bail!("Error IMU is disabled.");
            }
            if !self.imu[self.ctrl.imu_ref].is_enabled() {
                bail!("Reference IMU is disabled.");
            }
        }
        Ok(())
    }

    /// Start/Stop button. Returns false if the acquisition could not start.
    fn b_init(&mut self, stoptime: f64, ctx: &egui::Context) -> bool {
        if self.running() {
            self.start_busy = true;
            self.shared
                .stop_flag
                .store(true, std::sync::atomic::Ordering::SeqCst);
            self.set_status("Stopping...");
            return true;
        }
        if let Err(e) = self.init_checks() {
            self.set_status(format!("Error: {e}"));
            return false;
        }
        self.flagsaveplus = false;
        self.config_control();
        self.change_generator_config();
        self.change_imu_config();
        self.change_adc_config();
        self.plot_config();
        self.mainfig.show_adc = self.shared.cfg.lock().unwrap().adc_active();
        self.mode_locked = true;
        self.start_busy = true;
        let c = ctx.clone();
        start_acquisition(
            self.shared.clone(),
            stoptime,
            self.tx.clone(),
            Arc::new(move || c.request_repaint()),
        );
        true
    }

    fn reset_data(&mut self) {
        self.shared
            .data
            .write()
            .unwrap()
            .reset(self.tsampling_us as f64 / 1e6, self.maxtime);
    }

    fn b_reset(&mut self, ignore_unsaved: bool) {
        if self.running() {
            self.set_status("Reading must be stopped before erasing data.");
            return;
        }
        let saved = self.shared.data.read().unwrap().flagsaved;
        if !saved && !ignore_unsaved {
            self.confirm = Some(Confirm::ResetUnsaved);
            return;
        }
        self.do_reset();
    }

    fn do_reset(&mut self) {
        self.reset_data();
        self.status = None;
        self.mode_locked = false;
    }

    fn setup_allowed(&mut self) -> bool {
        let d = self.shared.data.read().unwrap();
        let ok = !self.running() && d.flagsaved;
        drop(d);
        if !ok {
            self.error_modal =
                Some("Operation not allowed when running or with unsaved data.".into());
        }
        ok
    }

    fn set_max_time(&mut self, minutes: u32) {
        if self.setup_allowed() {
            self.maxtime = minutes;
            self.reset_data();
            self.status = None;
        }
    }

    fn set_baud_rate(&mut self, baud: u32) {
        if self.setup_allowed() {
            self.baud = baud;
            self.shared.set_baud(baud);
            self.status = None;
        }
    }

    fn apply_sampling(&mut self, fsampling: u32, reset: bool) {
        let f = if MPU_RATES.contains(&fsampling) || LSM_RATES.contains(&fsampling) {
            fsampling
        } else {
            250
        };
        self.fsampling = f;
        self.tsampling_us = (1e6 / f as f64).round() as u32;
        if reset {
            self.reset_data();
        }
        // Disable IMUs that do not support this sampling rate.
        let incompatible = if MPU_RATES.contains(&f) { 2 } else { 1 };
        for imu in self.imu.iter_mut() {
            if imu.imu_type == incompatible {
                imu.imu_type = 0;
            }
        }
    }

    fn set_sampling(&mut self, fsampling: u32) {
        if self.setup_allowed() {
            self.apply_sampling(fsampling, true);
            self.change_imu_config();
            self.status = None;
        }
    }

    fn set_port(&mut self, port: String) {
        if self.running() {
            self.set_status("Not allowed when running.");
            return;
        }
        self.port = Some(port.clone());
        self.shared.set_port(Some(port));
        self.status = None;
    }

    fn full_loglist(&self) -> Vec<(f64, String)> {
        let mut l = Vec::new();
        if !self.notes.is_empty() {
            l.push((0.0, self.notes.clone()));
        }
        l.extend(self.loglist.iter().cloned());
        l
    }

    fn save_to(&mut self, path: &Path) -> Result<()> {
        let cfg = self.shared.cfg.lock().unwrap().clone();
        let logs = self.full_loglist();
        let mut d = self.shared.data.write().unwrap();
        d.save(path, &cfg, &logs)?;
        d.flagsaved = true;
        Ok(())
    }

    fn save_file(&mut self, fname: Option<PathBuf>) -> Result<()> {
        if self.running() {
            bail!("Readings must be stopped before saving file.");
        }
        let path = match fname {
            Some(p) => p,
            None => {
                let csv_first = self.last_saved_ext.as_deref() == Some(".csv");
                let mut dlg = rfd::FileDialog::new()
                    .set_title("Save File")
                    .set_directory(home());
                dlg = if csv_first {
                    dlg.add_filter("CSV file", &["csv"])
                        .add_filter("Feather file", &["feather"])
                } else {
                    dlg.add_filter("Feather file", &["feather"])
                        .add_filter("CSV file", &["csv"])
                };
                let Some(p) = dlg.save_file() else {
                    return Ok(());
                };
                let ext = p.extension().map(|e| e.to_string_lossy().to_lowercase());
                match ext.as_deref() {
                    Some("csv") => {
                        self.last_saved_ext = Some(".csv".into());
                        p
                    }
                    Some("feather") => {
                        self.last_saved_ext = Some(".feather".into());
                        p
                    }
                    _ => {
                        let e = if csv_first { "csv" } else { "feather" };
                        self.last_saved_ext = Some(format!(".{e}"));
                        PathBuf::from(format!("{}.{e}", p.display()))
                    }
                }
            }
        };
        self.save_to(&path)?;
        self.set_status(format!("File {} saved successfully.", path.display()));
        Ok(())
    }

    fn millis() -> u128 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0)
    }

    /// "Save and Update Metadata in Workdir".
    fn save_plus(&mut self) -> Result<()> {
        if self.flagsaveplus {
            bail!("Data already saved...");
        }
        if self.running() {
            bail!("Readings must be stopped before saving file.");
        }
        let mut fname = format!("{}.feather", Self::millis());
        let mut ct = 0;
        while self.workdir.join(&fname).exists() {
            fname = format!("{}{ct}.feather", Self::millis());
            ct += 1;
            if ct > 10 {
                bail!("10 file creation attempts and file still exists.");
            }
        }
        let (empty, ctrlmode) = {
            let d = self.shared.data.read().unwrap();
            (d.is_empty(), d.ctrlmode)
        };
        if empty {
            bail!("No data to save.");
        }
        if ctrlmode {
            let mfile = self.workdir.join("metacontrol.feather");
            let mut meta = if mfile.exists() {
                Table::read_feather(&mfile)?
            } else {
                Table::new()
            };
            let g = &self.gens[self.ctrl.perturb_channel];
            let st = |v: &str| Column::Str(vec![Some(v.to_string())]);
            meta.append_row(vec![
                ("filename", st(&fname)),
                ("alg", st(ALGORITHMS[self.ctrl.algorithm])),
                ("mu", Column::F64(vec![self.ctrl.mu()])),
                ("psi", Column::F64(vec![self.ctrl.psi()])),
                ("memctrl", Column::I64(vec![self.ctrl.mem as i64])),
                ("tinicio", Column::I64(vec![self.ctrl.t_alg_on as i64])),
                ("tipoperturb", st(GEN_TYPES[g.gen_type])),
                ("amplperturb", Column::F64(vec![g.ampl])),
                ("freqperturb", Column::F64(vec![g.freq])),
                ("sensorref", st(SENSORS[self.ctrl.ref_sensor])),
                ("sensorerro", st(SENSORS[self.ctrl.err_sensor])),
            ])?;
            meta.write_feather(&mfile)?;
        }
        self.save_to(&self.workdir.join(&fname))?;
        self.flagsaveplus = true;
        self.set_status("Data successfully saved.");
        Ok(())
    }

    fn k_space(&mut self, ctx: &egui::Context) {
        if !self.running() {
            self.b_init(3600.0, ctx);
        } else if self.control_on() && self.ctrl.is_task_control() && !self.ctrl.alg_on {
            self.ctrl.alg_on = true;
            self.config_alg_on();
        } else {
            self.b_init(3600.0, ctx);
        }
    }

    fn process_log(&mut self, t: f64, kind: LogKind) {
        if t == 0.0 && kind == LogKind::Started {
            self.loglist.clear();
        }
        match kind {
            LogKind::Started => {
                self.loglist.push((t, "Started".into()));
                for (k, imu) in self.imu.iter().enumerate() {
                    self.loglist
                        .push((t, format!("IMU{}|{}", k + 1, imu.log_string())));
                }
                self.loglist
                    .push((t, format!("ADC|{}", self.adc.log_string())));
                for k in 0..4 {
                    let en = self.gen_panel_enabled(k);
                    self.loglist
                        .push((t, format!("Gen{}|{}", k + 1, self.gens[k].log_string(en))));
                }
                self.loglist.push((
                    t,
                    format!("Ctrl|{}", self.ctrl.log_string(self.control_on())),
                ));
            }
            LogKind::Stopped => self.loglist.push((t, "Stopped".into())),
            LogKind::Gen(k) => {
                let en = self.gen_panel_enabled(k);
                self.loglist
                    .push((t, format!("Gen{}|{}", k + 1, self.gens[k].log_string(en))));
            }
            LogKind::Alg => self.loglist.push((t, self.ctrl.alg_on_log_string())),
        }
    }

    fn handle_events(&mut self) {
        while let Ok(ev) = self.rx.try_recv() {
            match ev {
                DataEvent::Status(s) => self.status = s,
                DataEvent::Started => {
                    self.start_busy = false;
                    if self.automator_wait_start {
                        self.automator_wait_start = false;
                        self.automator.resume();
                    }
                }
                DataEvent::Stopped => {
                    self.start_busy = false;
                    self.ctrl.alg_on = false;
                    let err = self.shared.data.read().unwrap().errorlevel > 0;
                    if err && self.automator.running {
                        self.automator
                            .print_error("Acquisition error: stopping the automator.");
                        self.automator.stop();
                    } else if self.automator_wait_stop {
                        self.automator_wait_stop = false;
                        self.automator.resume();
                    }
                }
                DataEvent::Log(t, k) => self.process_log(t, k),
            }
        }
    }

    // ---------------------------------------------------------- automator

    fn find_index(items: &[&str], value: &str) -> Option<usize> {
        let v = value.trim();
        items.iter().position(|i| i.eq_ignore_ascii_case(v))
    }

    fn opt(opts: &[String], i: usize) -> Result<&str> {
        opts.get(i)
            .map(String::as_str)
            .ok_or_else(|| anyhow!("missing parameter {}", i + 1))
    }

    fn num<T: std::str::FromStr>(opts: &[String], i: usize) -> Result<T> {
        let s = Self::opt(opts, i)?;
        s.trim()
            .parse::<T>()
            .or_else(|_| {
                s.trim()
                    .parse::<f64>()
                    .map_err(|_| ())
                    .and_then(|f| f.to_string().parse::<T>().map_err(|_| ()))
            })
            .map_err(|_| anyhow!("invalid number '{s}'"))
    }

    fn process_action(&mut self, cmd: Command, ctx: &egui::Context) -> ActionOutcome {
        let (name, opts) = cmd;
        let mut msg = format!("{:.0}: {name}", self.automator.elapsed());
        if !opts.is_empty() {
            msg += &format!(" - {opts:?}");
        }
        let r: Result<ActionOutcome> = (|| {
            Ok(match name.as_str() {
                "Reset" => {
                    self.b_reset(true);
                    ActionOutcome::Resume
                }
                "ConfigOutput" => {
                    let idx = Self::num::<usize>(&opts, 0)?.wrapping_sub(1);
                    if idx >= 4 {
                        bail!("Invalid output number");
                    }
                    let en = matches!(
                        Self::opt(&opts, 1)?.to_lowercase().as_str(),
                        "enabled" | "true"
                    );
                    self.gens[idx].enable = en;
                    if en {
                        let t = Self::opt(&opts, 2)?.to_string();
                        let a = Self::num::<f64>(&opts, 3)?;
                        let f = Self::num::<f64>(&opts, 4)?;
                        self.gens[idx].set_config(&t, a, f);
                    }
                    self.change_generator_config();
                    ActionOutcome::Resume
                }
                "ConfigIMU" => {
                    let idx = Self::num::<usize>(&opts, 0)?.wrapping_sub(1);
                    if idx >= 3 {
                        bail!("Invalid IMU number");
                    }
                    let t = Self::find_index(&IMU_TYPES, Self::opt(&opts, 1)?)
                        .ok_or_else(|| anyhow!("Invalid IMU type"))?;
                    let imu = &mut self.imu[idx];
                    imu.imu_type = t;
                    if t != 0 {
                        if let Some(a) = Self::find_index(&IMU_ADDRESSES, Self::opt(&opts, 2)?) {
                            imu.address = a;
                        }
                        if let Some(b) = Self::find_index(&IMU_BUSES, Self::opt(&opts, 3)?) {
                            imu.bus = b;
                        }
                        imu.acc_range = Self::num::<usize>(&opts, 4)?.min(3);
                        imu.gyro_range = Self::num::<usize>(&opts, 5)?.min(4);
                        if let Ok(f) = Self::num::<usize>(&opts, 7) {
                            imu.filter = f;
                        }
                    }
                    imu.normalize();
                    self.change_imu_config();
                    ActionOutcome::Resume
                }
                "ConfigADC" => {
                    let m = Self::find_index(&ADC_MODELS, Self::opt(&opts, 0)?)
                        .ok_or_else(|| anyhow!("Invalid ADC Model"))?;
                    self.adc.model = m;
                    let ch = Self::opt(&opts, 1)?.trim();
                    self.adc.channel = if ch.eq_ignore_ascii_case("off") {
                        0
                    } else {
                        ch.parse::<usize>()?.min(4)
                    };
                    if let Some(r) = Self::find_index(
                        &ADC_RANGES,
                        &format!("±{} V", Self::opt(&opts, 2)?.trim()),
                    ) {
                        self.adc.range = r;
                    }
                    let rate = format!("{} SPS", Self::opt(&opts, 3)?.trim());
                    if m == 0 {
                        if let Some(r) = Self::find_index(&RATES_1115, &rate) {
                            self.adc.rate1115 = r;
                        }
                    } else if let Some(r) = Self::find_index(&RATES_1015, &rate) {
                        self.adc.rate1015 = r;
                    }
                    self.change_adc_config();
                    ActionOutcome::Resume
                }
                "ControlChannels" => {
                    let set = |items: &[&str], v: &str, target: &mut usize| {
                        if let Some(i) = Self::find_index(items, v) {
                            *target = i;
                        }
                    };
                    set(
                        &OUTPUTS,
                        Self::opt(&opts, 0)?,
                        &mut self.ctrl.perturb_channel,
                    );
                    set(
                        &OUTPUTS,
                        Self::opt(&opts, 1)?,
                        &mut self.ctrl.control_channel,
                    );
                    set(&IMUS, Self::opt(&opts, 2)?, &mut self.ctrl.imu_ref);
                    set(&SENSORS, Self::opt(&opts, 3)?, &mut self.ctrl.ref_sensor);
                    set(&IMUS, Self::opt(&opts, 4)?, &mut self.ctrl.imu_err);
                    set(&SENSORS, Self::opt(&opts, 5)?, &mut self.ctrl.err_sensor);
                    if self.ctrl.perturb_channel == self.ctrl.control_channel {
                        bail!("Perturbation and control channels must differ");
                    }
                    self.config_control();
                    ActionOutcome::Resume
                }
                "SetControlMode" => {
                    self.mode = 1;
                    self.ctrl.task = 0;
                    let a = Self::opt(&opts, 0)?;
                    if let Some(i) = Self::find_index(&ALGORITHMS, a) {
                        self.ctrl.algorithm = i;
                    } else if let Ok(i) = a.trim().parse::<usize>() {
                        self.ctrl.algorithm = i.min(ALGORITHMS.len() - 1);
                    }
                    self.ctrl.t_alg_on = Self::num::<u32>(&opts, 1)?.min(99);
                    self.ctrl.mem = Self::num::<u16>(&opts, 2)?.clamp(1, 1000);
                    self.ctrl.step = Self::opt(&opts, 3)?.trim().to_string();
                    self.ctrl.penalization = Self::opt(&opts, 4)?.trim().to_string();
                    self.config_control();
                    ActionOutcome::Resume
                }
                "SetPathModeling" => {
                    self.mode = 1;
                    self.ctrl.task = 1;
                    self.config_control();
                    ActionOutcome::Resume
                }
                "SetReadingMode" => {
                    self.mode = 0;
                    self.config_control();
                    ActionOutcome::Resume
                }
                "Wait" | "Waiting" => {
                    if self.running() {
                        self.automator_wait_stop = true;
                        ActionOutcome::Wait
                    } else {
                        ActionOutcome::Resume
                    }
                }
                "Start" | "StartWait" | "StartNotify" => {
                    let stoptime = Self::num::<f64>(&opts, 0)?;
                    if self.running() {
                        bail!("Acquisition already running");
                    }
                    if !self.b_init(stoptime, ctx) {
                        bail!("{}", self.status.clone().unwrap_or_default());
                    }
                    match name.as_str() {
                        "StartWait" => {
                            self.automator_wait_stop = true;
                            ActionOutcome::Wait
                        }
                        "StartNotify" => {
                            self.automator_wait_start = true;
                            ActionOutcome::Wait
                        }
                        _ => ActionOutcome::Resume,
                    }
                }
                "Stop" => {
                    if self.running() {
                        self.automator_wait_stop = true;
                        self.b_init(0.0, ctx);
                        ActionOutcome::Wait
                    } else {
                        ActionOutcome::Resume
                    }
                }
                "AlgOn" => {
                    self.ctrl.alg_on = Self::opt(&opts, 0)?.trim().eq_ignore_ascii_case("true");
                    self.config_alg_on();
                    ActionOutcome::Resume
                }
                "Print" => {
                    let text = opts.join(", ");
                    self.automator.print(text);
                    ActionOutcome::Resume
                }
                "SaveFile" => {
                    let p = self.workdir.join(Self::opt(&opts, 0)?);
                    self.save_file(Some(p))?;
                    ActionOutcome::Resume
                }
                "SetWorkDir" => {
                    let p = home().join(Self::opt(&opts, 0)?);
                    std::fs::create_dir_all(&p)?;
                    self.workdir = p;
                    ActionOutcome::Resume
                }
                other => bail!("Unknown command '{other}'"),
            })
        })();
        match r {
            Ok(o) => {
                self.automator.print(msg);
                o
            }
            Err(e) => {
                self.automator.print(msg);
                self.automator.print_error(&format!("Error: {e}"));
                ActionOutcome::Abort
            }
        }
    }

    fn automator_stop(&mut self, ctx: &egui::Context) {
        self.automator.stop();
        self.automator_wait_start = false;
        self.automator_wait_stop = false;
        if self.running() {
            self.b_init(0.0, ctx);
        }
    }

    fn run_automator(&mut self, ctx: &egui::Context) {
        // Several immediate actions may run in one frame.
        for _ in 0..64 {
            let Some(cmd) = self.automator.next_action() else {
                break;
            };
            let outcome = self.process_action(cmd, ctx);
            if outcome == ActionOutcome::Abort {
                self.automator_stop(ctx);
                break;
            }
            self.automator.complete(outcome);
        }
    }

    // ----------------------------------------------------------------- UI

    fn menu_bar(&mut self, ui: &mut Ui, ctx: &egui::Context) {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("Save...").clicked() {
                    ui.close();
                    if let Err(e) = self.save_file(None) {
                        self.set_status(e.to_string());
                    }
                }
                if ui.button("Save and Update Metadata in Workdir").clicked() {
                    ui.close();
                    if let Err(e) = self.save_plus() {
                        self.set_status(e.to_string());
                    }
                }
                if ui.button("Define Workdir").clicked() {
                    ui.close();
                    if let Some(d) = rfd::FileDialog::new()
                        .set_title("Choose the working directory (workdir)")
                        .set_directory(&self.workdir)
                        .pick_folder()
                    {
                        self.workdir = d;
                        self.set_status(format!(
                            "Working directory set to: {}",
                            self.workdir.display()
                        ));
                    }
                }
                if ui.button("Workdir Manager").clicked() {
                    ui.close();
                    if !self.running() {
                        self.workdir_manager = Some(WorkdirManager::new(&self.workdir));
                    }
                }
                ui.separator();
                if ui.button("Quit").clicked() {
                    ui.close();
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            });
            ui.menu_button("Tools", |ui| {
                if ui.button("Automator...").clicked() {
                    ui.close();
                    self.automator.open = true;
                }
                if ui.button("Data Viewer...").clicked() {
                    ui.close();
                    if !self.running() {
                        self.data_viewer = Some(DataViewer::new());
                    }
                }
                if ui.button("Path modeling...").clicked() {
                    ui.close();
                    if self.port.is_none() {
                        self.set_status("Select a serial port before opening Path Modeling.");
                    } else if !self.running() {
                        self.path_modeling = Some(PathModelingDialog::new(
                            self.path_modeling_opts.clone(),
                            &self.shared,
                        ));
                    }
                }
                if ui.button("Firmware/Software Updater...").clicked() {
                    ui.close();
                    if self.running() {
                        self.set_status("Stop data acquisition before updating.");
                    } else {
                        self.updater = Some(UpdaterDialog::new(self.port.clone()));
                    }
                }
            });
            ui.menu_button("Setup", |ui| {
                ui.menu_button("Port", |ui| {
                    let ports = list_ports();
                    if ports.is_empty() {
                        ui.add_enabled(false, egui::Button::new("No serial ports found"));
                    }
                    for p in ports {
                        let sel = self.port.as_deref() == Some(p.port_name.as_str());
                        if ui
                            .add(egui::Button::selectable(sel, &p.port_name))
                            .clicked()
                        {
                            ui.close();
                            self.set_port(p.port_name.clone());
                        }
                    }
                });
                ui.menu_button("Sampling Rate", |ui| {
                    for (label, rates) in [("MPU6050:", MPU_RATES), ("LSM6DS3:", LSM_RATES)] {
                        ui.add_enabled(false, egui::Label::new(label));
                        for r in rates {
                            if ui
                                .add(egui::Button::selectable(
                                    self.fsampling == r,
                                    format!("{r} Hz"),
                                ))
                                .clicked()
                            {
                                ui.close();
                                self.set_sampling(r);
                            }
                        }
                    }
                });
                ui.menu_button("Max Time", |ui| {
                    for m in MAX_TIMES {
                        if ui
                            .add(egui::Button::selectable(
                                self.maxtime == m,
                                format!("{m} min"),
                            ))
                            .clicked()
                        {
                            ui.close();
                            self.set_max_time(m);
                        }
                    }
                });
                ui.menu_button("Serial Baud Rate", |ui| {
                    for b in BAUD_RATES {
                        if ui
                            .add(egui::Button::selectable(self.baud == b, b.to_string()))
                            .clicked()
                        {
                            ui.close();
                            self.set_baud_rate(b);
                        }
                    }
                });
                if ui.button("Additional system setup...").clicked() {
                    ui.close();
                    if !self.running() {
                        self.additional = Some(AdditionalDialog::new(&self.shared));
                    }
                }
            });
        });
    }

    fn top_bar(&mut self, ui: &mut Ui) {
        let running = self.running();
        let (readtime, maxtime, realtime, calctime, errorflag) = {
            let d = self.shared.data.read().unwrap();
            (d.readtime, d.maxtime, d.realtime, d.calctime, d.errorflag)
        };
        ui.horizontal(|ui| {
            ui.label("Mode:");
            ui.add_enabled_ui(!running && !self.mode_locked, |ui| {
                if crate::panels::combo(
                    ui,
                    "mainmode",
                    &mut self.mode,
                    &["Measurement", "Control"],
                    None,
                ) {
                    self.config_control();
                }
            });
            ui.separator();
            ui.label("Elapsed Time:");
            ui.monospace(format!("{} s / {} s", readtime as i64, maxtime as i64));
            ui.separator();
            ui.label("Time Delay:");
            ui.monospace(format!("{realtime:.3} s"));
            ui.separator();
            ui.label("Sample Time:");
            ui.monospace(format!(
                "{}/{}/{} us",
                calctime[0], calctime[1], calctime[2]
            ));
            ui.separator();
            ui.label("Error Flag:");
            let txt = RichText::new(format!(" {errorflag:x} ")).monospace();
            if errorflag > 0 {
                ui.label(txt.background_color(Color32::from_rgb(255, 150, 150)));
            } else {
                ui.label(txt);
            }
            if let Some(p) = &self.port {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.weak(format!("{p} @ {} | {} Hz", self.baud, self.fsampling));
                });
            }
        });
    }

    fn tabs(ui: &mut Ui, current: &mut usize, titles: &[String]) {
        ui.horizontal_wrapped(|ui| {
            for (i, t) in titles.iter().enumerate() {
                if ui.add(egui::Button::selectable(*current == i, t)).clicked() {
                    *current = i;
                }
            }
        });
    }

    fn left_column(&mut self, ui: &mut Ui) {
        let running = self.running();
        // Outputs
        ui.group(|ui| {
            let titles: Vec<String> = (0..4).map(|k| self.output_tab_title(k)).collect();
            Self::tabs(ui, &mut self.output_tab, &titles);
            ui.separator();
            let k = self.output_tab;
            let en = self.gen_panel_enabled(k);
            if self.gens[k].ui(ui, en) {
                self.change_generator_config();
                self.plot_config();
            }
        });
        // Control
        ui.group(|ui| {
            ui.strong("Control");
            let r = self.ctrl.ui(ui, self.control_on(), running);
            if r.config_changed {
                self.config_control();
                self.plot_config();
            }
            if r.alg_on_toggled {
                self.config_alg_on();
            }
        });
        // Notes
        ui.horizontal(|ui| {
            ui.label("Notes:");
            let r = ui.add(
                egui::TextEdit::multiline(&mut self.notes)
                    .desired_rows(2)
                    .desired_width(f32::INFINITY),
            );
            if r.changed() && self.notes.chars().count() > MAX_NOTE_LENGTH {
                self.notes = self.notes.chars().take(MAX_NOTE_LENGTH).collect();
            }
        });
        // Output figure
        let d = self.shared.data.read().unwrap();
        let h = (ui.available_height() - 4.0).max(100.0);
        self.outfig.ui(ui, &d, h);
    }

    fn right_column(&mut self, ui: &mut Ui, ctx: &egui::Context) {
        let running = self.running();
        ui.group(|ui| {
            let titles: Vec<String> =
                vec!["IMU 1".into(), "IMU 2".into(), "IMU 3".into(), "ADC".into()];
            Self::tabs(ui, &mut self.sensor_tab, &titles);
            ui.separator();
            let t = self.sensor_tab;
            if t < 3 {
                if self.imu[t].ui(ui, t, running) {
                    self.change_imu_config();
                }
            } else if self.adc.ui(ui, running) {
                self.change_adc_config();
            }
        });
        let buttons_h = 36.0;
        let avail = ui.available_height() - buttons_h;
        ui.allocate_ui(egui::vec2(ui.available_width(), avail.max(150.0)), |ui| {
            let d = self.shared.data.read().unwrap();
            if self.control_on() {
                self.ctrlfig.ui(ui, &d);
            } else {
                self.mainfig.ui(ui, &d);
            }
        });
        ui.horizontal(|ui| {
            let start_en = !self.start_busy && !(self.automator.running && running);
            let big = |t: &str| {
                egui::Button::new(RichText::new(t).size(16.0)).min_size(egui::vec2(140.0, 28.0))
            };
            if ui.add_enabled(start_en, big("Start/Stop")).clicked() {
                self.b_init(3600.0, ctx);
            }
            if ui.add_enabled(!running, big("Reset Data")).clicked() {
                self.b_reset(false);
            }
        });
    }

    fn shortcuts(&mut self, ctx: &egui::Context) {
        use egui::{Key, KeyboardShortcut, Modifiers};
        let save2 = KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, Key::S);
        let save = KeyboardShortcut::new(Modifiers::COMMAND, Key::S);
        let focus = KeyboardShortcut::new(Modifiers::ALT, Key::P);
        if ctx.input_mut(|i| i.consume_shortcut(&save2)) {
            if let Err(e) = self.save_plus() {
                self.set_status(e.to_string());
            }
            self.b_reset(false);
        } else if ctx.input_mut(|i| i.consume_shortcut(&save))
            && let Err(e) = self.save_file(None)
        {
            self.set_status(e.to_string());
        }
        if ctx.input_mut(|i| i.consume_shortcut(&focus)) {
            ctx.memory_mut(|m| m.request_focus(egui::Id::unique(STEP_FIELD_ID)));
        }
        let no_text_focus = !ctx.egui_wants_keyboard_input();
        if no_text_focus
            && self.confirm.is_none()
            && ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Space))
        {
            self.k_space(ctx);
        }
    }

    fn modals(&mut self, ctx: &egui::Context) {
        if let Some(msg) = self.error_modal.clone() {
            egui::Modal::new(egui::Id::unique("error_modal")).show(ctx, |ui| {
                ui.heading("Error");
                ui.label(msg);
                if ui.button("OK").clicked() {
                    self.error_modal = None;
                }
            });
        }
        let Some(kind) = &self.confirm else { return };
        let (title, text) = match kind {
            Confirm::ResetUnsaved => ("Warning!", "Data is not saved and will be erased. Confirm?"),
            Confirm::QuitUnsaved => (
                "Quitting",
                "There is unsaved data. Do you really want to quit?",
            ),
        };
        let mut answer = None;
        egui::Modal::new(egui::Id::unique("confirm_modal")).show(ctx, |ui| {
            ui.heading(title);
            ui.label(text);
            ui.horizontal(|ui| {
                if ui.button("Yes").clicked() {
                    answer = Some(true);
                }
                if ui.button("No").clicked() {
                    answer = Some(false);
                }
            });
        });
        if let Some(a) = answer {
            let kind = self.confirm.take().unwrap();
            match (kind, a) {
                (Confirm::ResetUnsaved, true) => self.do_reset(),
                (Confirm::ResetUnsaved, false) => self.set_status("Canceled..."),
                (Confirm::QuitUnsaved, true) => {
                    self.allow_close = true;
                    self.persist();
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
                (Confirm::QuitUnsaved, false) => {}
            }
        }
    }

    fn persist(&self) {
        if let Err(e) = self.settings().save() {
            eprintln!("[actvib] could not save settings: {e}");
        }
    }

    fn handle_close(&mut self, ctx: &egui::Context) {
        if !ctx.input(|i| i.viewport().close_requested()) || self.allow_close {
            return;
        }
        if self.updater.as_ref().is_some_and(|u| u.running) {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.set_status("Wait for the firmware update to finish before closing.");
        } else if self.running() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.set_status("Stop the readings before closing...");
        } else if !self.shared.data.read().unwrap().flagsaved {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.confirm = Some(Confirm::QuitUnsaved);
        } else {
            self.allow_close = true;
            self.persist();
            self.shared.close_link();
        }
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        if let Some(d) = self.path_modeling.as_mut() {
            d.show(ctx, &self.shared);
            if !d.open {
                self.path_modeling_opts = Some(d.options_to_save());
                self.path_modeling = None;
            }
        }
        if let Some(d) = self.data_viewer.as_mut() {
            d.show(ctx, &self.shared, &mut self.last_data_folder);
            if !d.open {
                self.data_viewer = None;
            }
        }
        if let Some(d) = self.additional.as_mut() {
            d.show(ctx, &self.shared);
            if !d.open {
                self.additional = None;
            }
        }
        if let Some(d) = self.workdir_manager.as_mut() {
            d.show(ctx);
            if !d.open {
                self.workdir_manager = None;
            }
        }
        let running = self.running();
        if let Some(d) = self.updater.as_mut() {
            d.show(ctx, &self.shared, running);
            if !d.open {
                self.updater = None;
            }
        }
        if self.automator.show(ctx) {
            self.automator_stop(ctx);
        }
    }
}

impl eframe::App for ActVibApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_events();
        self.run_automator(&ctx);
        self.shortcuts(&ctx);
        self.handle_close(&ctx);

        egui::Panel::top("menu").show(ui, |ui| self.menu_bar(ui, &ctx));
        egui::Panel::top("topbar").show(ui, |ui| self.top_bar(ui));
        egui::Panel::bottom("statusbar").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(self.status.clone().unwrap_or_default());
            });
        });
        let left_w = (ui.available_width() * 0.45).clamp(380.0, 620.0);
        egui::Panel::left("left")
            .resizable(true)
            .default_size(left_w)
            .min_size(340.0)
            .show(ui, |ui| {
                self.left_column(ui);
            });
        egui::CentralPanel::default().show(ui, |ui| self.right_column(ui, &ctx));

        self.dialogs(&ctx);
        self.modals(&ctx);

        if self.running() || self.start_busy {
            ctx.request_repaint_after(Duration::from_millis(33));
        }
    }

    fn on_exit(&mut self) {
        if !self.allow_close {
            self.persist();
        }
        self.shared.close_link();
    }
}
