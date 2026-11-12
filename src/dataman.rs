//! Data manager: acquisition buffers, the acquisition thread and data export
//! (port of `dataman.py`).

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Result, anyhow};
use crossbeam_channel::Sender;

use crate::driver::{DebugSim, DriverConfig, Link, Reading, proto};
use crate::table::{Column, Table};

/// Interval (in acquisition time) between device syncs: generator updates,
/// algorithm on/off and stop-time checks.
pub const PLOT_UPDATE_SEC: f64 = 1.0;

/// Kinds of log entries emitted by the acquisition thread; the UI turns them
/// into descriptive strings using the current panel state.
#[derive(Clone, Debug, PartialEq)]
pub enum LogKind {
    Started,
    Stopped,
    Gen(usize),
    Alg,
}

#[derive(Clone, Debug)]
pub enum DataEvent {
    Status(Option<String>),
    Started,
    Stopped,
    Log(f64, LogKind),
}

/// Recorded samples.
pub struct DataStore {
    pub samplingperiod: f64,
    /// Maximum recording time in seconds.
    pub maxtime: f64,
    pub wsize: usize,
    pub time: Vec<f64>,
    pub acc: [[Vec<f64>; 3]; 3],
    pub gyro: [[Vec<f64>; 3]; 3],
    pub dac: [Vec<f64>; 4],
    pub adc: [Vec<f64>; 4],
    pub xref: Vec<f64>,
    pub xerr: Vec<f64>,
    pub flagsaved: bool,
    pub readtime: f64,
    pub realtime: f64,
    pub ctrlmode: bool,
    pub taskisctrl: bool,
    pub errorlevel: u8,
    pub globalstarttime: f64,
    pub calctime: [u16; 3],
    pub errorflag: u8,
}

impl DataStore {
    pub fn new(samplingperiod: f64, maxtime_min: u32) -> Self {
        let maxtime = maxtime_min as f64 * 60.0;
        Self {
            samplingperiod,
            maxtime,
            wsize: (maxtime / samplingperiod) as usize,
            time: vec![],
            acc: Default::default(),
            gyro: Default::default(),
            dac: Default::default(),
            adc: Default::default(),
            xref: vec![],
            xerr: vec![],
            flagsaved: true,
            readtime: 0.0,
            realtime: 0.0,
            ctrlmode: false,
            taskisctrl: false,
            errorlevel: 0,
            globalstarttime: 0.0,
            calctime: [0; 3],
            errorflag: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.time.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time.is_empty()
    }

    /// Clears data, possibly with a new sampling period (s) and maximum time (min).
    pub fn reset(&mut self, samplingperiod: f64, maxtime_min: u32) {
        *self = DataStore::new(samplingperiod, maxtime_min);
    }

    fn push(&mut self, t: f64, r: &Reading, cfg: &DriverConfig) {
        self.time.push(t);
        if cfg.control_mode {
            self.xref.push(r.xref);
            self.xerr.push(r.xerro);
            self.dac[0].push(r.dacout[0]);
            self.dac[1].push(r.dacout[1]);
            self.dac[2].push(0.0);
            self.dac[3].push(0.0);
            for j in 0..3 {
                for k in 0..3 {
                    self.acc[j][k].push(0.0);
                    self.gyro[j][k].push(0.0);
                }
            }
        } else {
            self.xref.push(0.0);
            self.xerr.push(0.0);
            for j in 0..3 {
                let en = cfg.imu_enable[j];
                for k in 0..3 {
                    self.acc[j][k].push(if en { r.acc[j][k] } else { 0.0 });
                    self.gyro[j][k].push(if en { r.gyro[j][k] } else { 0.0 });
                }
            }
            for k in 0..4 {
                self.dac[k].push(r.dacout[k]);
            }
        }
        for k in 0..4 {
            self.adc[k].push(r.adcin[k]);
        }
        self.calctime = r.calctime;
        self.errorflag = r.errorflag;
    }

    /// Builds the table to export, merging the log entries into a `log` column
    /// the same way the Python version does.
    pub fn to_table(&self, cfg: &DriverConfig, loglist: &[(f64, String)]) -> Table {
        let n = self.len();
        let mut t = Table::new();
        t.push("time", Column::F64(self.time[..n].to_vec()));
        if self.ctrlmode {
            t.push("perturb", Column::F64(self.dac[0][..n].to_vec()));
            t.push("ctrl", Column::F64(self.dac[1][..n].to_vec()));
            t.push("ref", Column::F64(self.xref[..n].to_vec()));
            t.push("err", Column::F64(self.xerr[..n].to_vec()));
        } else {
            for k in 0..4 {
                t.push(
                    format!("dac{}", k + 1),
                    Column::F64(self.dac[k][..n].to_vec()),
                );
            }
            if cfg.adc_active() {
                for k in 0..4 {
                    t.push(
                        format!("adc{}.{}", k + 1, cfg.adcsel as i32 + 1),
                        Column::F64(self.adc[k][..n].to_vec()),
                    );
                }
            }
            for k in 0..3 {
                if cfg.imu_enable[k] {
                    let i = k + 1;
                    for (axis, name) in ["x", "y", "z"].iter().enumerate() {
                        t.push(
                            format!("imu{i}acc{name}"),
                            Column::F64(self.acc[k][axis][..n].to_vec()),
                        );
                    }
                    for (axis, name) in ["x", "y", "z"].iter().enumerate() {
                        t.push(
                            format!("imu{i}gyro{name}"),
                            Column::F64(self.gyro[k][axis][..n].to_vec()),
                        );
                    }
                }
            }
        }
        if !loglist.is_empty() && n > 0 {
            let mut log: Vec<Option<String>> = vec![None; n];
            log[0] = Some(" ".into());
            let mut idx = 0usize;
            for (tstamp, msg) in loglist {
                while idx < n && self.time[idx] < *tstamp {
                    idx += 1;
                }
                if idx >= n {
                    // Past the last sample: attach to the last row.
                    let last = &mut log[n - 1];
                    *last = Some(match last.take() {
                        Some(s) if s.trim().is_empty() => msg.clone(),
                        Some(s) => format!("{s}; {msg}"),
                        None => msg.clone(),
                    });
                    continue;
                }
                log[idx] = Some(msg.clone());
                idx += 1;
            }
            t.push("log", Column::Str(log));
        }
        t
    }

    pub fn save(&self, path: &Path, cfg: &DriverConfig, loglist: &[(f64, String)]) -> Result<()> {
        let t = self.to_table(cfg, loglist);
        let name = path.to_string_lossy().to_lowercase();
        if name.ends_with("feather") {
            t.write_feather(path)
        } else if name.ends_with("csv") {
            t.write_csv(path)
        } else {
            Err(anyhow!("Unknown file extension (use .feather or .csv)."))
        }
    }
}

/// Serial port selection.
#[derive(Clone, Debug)]
pub struct PortSettings {
    pub port: Option<String>,
    pub baud: u32,
}

/// State shared between the UI and worker threads.
pub struct Shared {
    pub cfg: Mutex<DriverConfig>,
    pub port: Mutex<PortSettings>,
    /// Open serial link (kept open between acquisitions, like the Python version).
    pub link: Mutex<Option<Link>>,
    pub data: RwLock<DataStore>,
    pub running: AtomicBool,
    pub stop_flag: AtomicBool,
}

impl Shared {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            cfg: Mutex::new(DriverConfig::default()),
            port: Mutex::new(PortSettings {
                port: None,
                baud: crate::driver::DEFAULT_BAUD,
            }),
            link: Mutex::new(None),
            data: RwLock::new(DataStore::new(4e-3, 10)),
            running: AtomicBool::new(false),
            stop_flag: AtomicBool::new(false),
        })
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    /// Takes the open link, or opens (and resets) the selected port.
    pub fn acquire_link(&self) -> Result<Link> {
        if let Some(l) = self.link.lock().unwrap().take() {
            return Ok(l);
        }
        let ps = self.port.lock().unwrap().clone();
        let port = ps
            .port
            .ok_or_else(|| anyhow!("Select a serial port first."))?;
        let mut link = Link::open_serial(&port, ps.baud)?;
        link.reset_device()?;
        Ok(link)
    }

    pub fn release_link(&self, link: Link) {
        *self.link.lock().unwrap() = Some(link);
    }

    pub fn close_link(&self) {
        *self.link.lock().unwrap() = None;
    }

    /// Changes the baud rate (applied immediately to an open link).
    pub fn set_baud(&self, baud: u32) {
        self.port.lock().unwrap().baud = baud;
        if let Some(l) = self.link.lock().unwrap().as_mut() {
            let _ = l.set_baud(baud);
        }
    }

    pub fn set_port(&self, port: Option<String>) {
        let mut ps = self.port.lock().unwrap();
        if ps.port != port {
            ps.port = port;
            drop(ps);
            self.close_link();
        }
    }

    /// Runs `f` with a link (opening it if necessary), then keeps the link open.
    /// On error, the link is dropped so it will be reopened next time.
    pub fn with_link<T>(&self, f: impl FnOnce(&mut Link) -> Result<T>) -> Result<T> {
        let mut link = self.acquire_link()?;
        let r = f(&mut link);
        if r.is_ok() {
            self.release_link(link);
        }
        r
    }
}

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Starts the acquisition thread.
pub fn start_acquisition(
    shared: Arc<Shared>,
    stoptime: f64,
    tx: Sender<DataEvent>,
    notify: Arc<dyn Fn() + Send + Sync>,
) {
    shared.stop_flag.store(false, Ordering::SeqCst);
    shared.running.store(true, Ordering::SeqCst);
    let _ = tx.send(DataEvent::Status(None));
    std::thread::Builder::new()
        .name("acquisition".into())
        .spawn(move || {
            let send = |e: DataEvent| {
                let _ = tx.send(e);
                notify();
            };
            let mut link_slot: Option<Link> = None;
            let result = run_acquisition(&shared, stoptime, &send, &mut link_slot);
            match result {
                Ok(()) => {
                    if let Some(l) = link_slot.take() {
                        shared.release_link(l);
                    }
                }
                Err(err) => {
                    if let Some(mut l) = link_slot.take() {
                        let _ = proto::stop_readings(&mut l);
                    }
                    shared.data.write().unwrap().errorlevel = 1;
                    shared.running.store(false, Ordering::SeqCst);
                    send(DataEvent::Status(Some(format!("Error: {err}"))));
                    send(DataEvent::Stopped);
                }
            }
            shared.running.store(false, Ordering::SeqCst);
            shared.stop_flag.store(false, Ordering::SeqCst);
            notify();
        })
        .expect("failed to spawn acquisition thread");
}

fn run_acquisition(
    shared: &Shared,
    stoptime: f64,
    send: &dyn Fn(DataEvent),
    link_slot: &mut Option<Link>,
) -> Result<()> {
    let mut cfg = shared.cfg.lock().unwrap().clone();
    let (samplingperiod, wsize) = {
        let mut d = shared.data.write().unwrap();
        d.errorlevel = 0;
        d.ctrlmode = cfg.control_mode;
        d.taskisctrl = cfg.task_is_control;
        (d.samplingperiod, d.wsize)
    };
    let mut debug = if cfg.debug_mode {
        Some(DebugSim::new())
    } else {
        None
    };

    send(DataEvent::Status(Some("Opening port...".into())));
    *link_slot = Some(shared.acquire_link()?);
    let link = link_slot.as_mut().unwrap();
    send(DataEvent::Status(Some("Handshaking...".into())));
    if let Err(e) = proto::handshake(link) {
        *link_slot = None; // pyserial closes the port when the handshake fails
        return Err(e);
    }
    send(DataEvent::Status(Some("Writing configs...".into())));
    proto::write_sampling_micros(link, (samplingperiod * 1e6).round() as u32)?;
    for k in 0..3u8 {
        proto::write_imu_config(link, k, &cfg.imuconfigdata[k as usize])?;
        if cfg.imu_enable[k as usize] {
            proto::init_hardware(link, k)?;
        }
    }
    {
        // Lock the shared config only around each command, never across sleeps,
        // so the UI stays responsive while the device is configured.
        proto::write_adc_config(link, &mut shared.cfg.lock().unwrap())?;
        for k in 0..4 {
            proto::write_generator_config(link, &mut shared.cfg.lock().unwrap(), k)?;
            std::thread::sleep(Duration::from_millis(100));
        }
        if cfg.control_mode {
            let snapshot = shared.cfg.lock().unwrap().clone();
            proto::write_control_config(link, &snapshot)?;
            shared.cfg.lock().unwrap().set_alg_on(false, 0.0, true);
            proto::start_control(link)?;
        } else {
            proto::start_readings(link)?;
        }
        cfg = shared.cfg.lock().unwrap().clone();
    }
    send(DataEvent::Status(None));
    send(DataEvent::Started);

    let starttime = (now_secs() * 1000.0).round() / 1000.0;
    let start_instant = Instant::now();
    let timedelta = {
        let mut d = shared.data.write().unwrap();
        d.flagsaved = false;
        if d.is_empty() {
            d.globalstarttime = starttime;
        }
        starttime - d.globalstarttime
    };
    send(DataEvent::Log(timedelta, LogKind::Started));

    let mut ctreadings: u64 = 0;
    let mut lastfigrefresh = 0.0;
    let mut readtime = timedelta;
    let mut reading = Reading::default();
    loop {
        if shared.stop_flag.load(Ordering::SeqCst) {
            break;
        }
        if shared.data.read().unwrap().len() >= wsize {
            break;
        }
        if let Some(sim) = debug.as_mut() {
            sim.talk(link, &cfg)?;
        }
        proto::get_reading(link, &cfg, &mut reading)?;
        readtime = ctreadings as f64 * samplingperiod + timedelta;
        {
            let mut d = shared.data.write().unwrap();
            d.push(readtime, &reading, &cfg);
            d.readtime = readtime;
        }
        ctreadings += 1;
        if readtime - lastfigrefresh >= PLOT_UPDATE_SEC {
            lastfigrefresh = readtime;
            shared.data.write().unwrap().realtime =
                readtime - start_instant.elapsed().as_secs_f64();
            let mut logs = Vec::new();
            {
                let mut scfg = shared.cfg.lock().unwrap();
                if scfg.algonchanged && readtime >= scfg.algontime {
                    proto::write_alg_on(link, &mut scfg)?;
                    logs.push(LogKind::Alg);
                }
                for k in 0..4 {
                    if !scfg.gen_config_written[k] {
                        proto::write_generator_config(link, &mut scfg, k)?;
                        logs.push(LogKind::Gen(k));
                    }
                }
                // Keep acquisition-defining settings fixed; refresh output scaling.
                cfg.dclevel = scfg.dclevel;
                cfg.iampscaler = scfg.iampscaler;
            }
            for l in logs {
                send(DataEvent::Log(readtime, l));
            }
            if readtime >= stoptime {
                shared.stop_flag.store(true, Ordering::SeqCst);
                send(DataEvent::Status(Some("Stopping...".into())));
            }
        }
    }
    proto::stop_readings(link)?;
    shared.running.store(false, Ordering::SeqCst);
    send(DataEvent::Stopped);
    send(DataEvent::Log(readtime, LogKind::Stopped));
    send(DataEvent::Status(Some("Stopped.".into())));
    shared.data.write().unwrap().readtime = ctreadings as f64 * samplingperiod + timedelta;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_merge_matches_python_semantics() {
        let mut d = DataStore::new(1.0, 1);
        let mut cfg = DriverConfig::default();
        cfg.control_mode = true;
        d.ctrlmode = true;
        let r = Reading::default();
        for t in 0..5 {
            d.push(t as f64, &r, &cfg);
        }
        let logs = vec![
            (0.0, "notes".to_string()),
            (0.0, "Started".to_string()),
            (2.5, "Gen1|x".to_string()),
            (4.0, "Stopped".to_string()),
        ];
        let t = d.to_table(&cfg, &logs);
        let log = t.get("log").unwrap();
        assert_eq!(log.cell_string(0), "notes");
        assert_eq!(log.cell_string(1), "Started");
        assert_eq!(log.cell_string(2), "");
        assert_eq!(log.cell_string(3), "Gen1|x");
        assert_eq!(log.cell_string(4), "Stopped");
        assert_eq!(
            t.names,
            vec!["time", "perturb", "ctrl", "ref", "err", "log"]
        );
    }

    #[test]
    fn measurement_columns() {
        let mut d = DataStore::new(4e-3, 1);
        let mut cfg = DriverConfig::default();
        cfg.set_imu_config(1, [1, 0x68, 0]);
        cfg.set_adc_config([0x03, 0, 0]);
        d.push(0.0, &Reading::default(), &cfg);
        let t = d.to_table(&cfg, &[]);
        assert_eq!(
            t.names,
            vec![
                "time",
                "dac1",
                "dac2",
                "dac3",
                "dac4",
                "adc1.1",
                "adc2.1",
                "adc3.1",
                "adc4.1",
                "imu2accx",
                "imu2accy",
                "imu2accz",
                "imu2gyrox",
                "imu2gyroy",
                "imu2gyroz"
            ]
        );
    }
}

#[cfg(test)]
mod acquisition_tests {
    use super::*;
    use crate::driver::mock::{MockPort, link};

    fn packet(dac0: i16, calc: u16) -> Vec<u8> {
        // Measurement packet with no IMU and ADC off: 3-byte header + 13 bytes.
        let mut p = vec![0x0F, 0x0F, 0x0F];
        p.extend_from_slice(&dac0.to_be_bytes());
        p.extend_from_slice(&[0, 0, 0, 0]);
        p.extend_from_slice(&calc.to_be_bytes());
        p.extend_from_slice(&[0, 0, 0, 0]);
        p.push(0);
        p
    }

    #[test]
    fn full_measurement_run_against_simulated_firmware() {
        let shared = Shared::new();
        let port = MockPort::default();
        *shared.link.lock().unwrap() = Some(link(&port));
        // 1 s sampling, 1 min max -> buffer of 60 samples ends the run.
        shared.data.write().unwrap().reset(1.0, 1);
        // Firmware replies, in protocol order.
        port.feed(b"k"); // handshake
        port.feed(&[b'm', 0x42, 0x40]); // sampling 1_000_000 us echo
        port.feed(b"KIKIKI"); // three IMU configs
        port.feed(b"KA\0\0\0\0"); // ADC config
        port.feed(&[0xAA; 10]); // bytes consumed by the start-readings check
        for k in 0..100 {
            port.feed(&packet(k as i16, 4000));
        }
        let (tx, rx) = crossbeam_channel::unbounded();
        start_acquisition(shared.clone(), 3600.0, tx, Arc::new(|| {}));
        let mut events = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Ok(e) = rx.recv_timeout(Duration::from_millis(100)) {
                let stop = matches!(e, DataEvent::Status(Some(ref s)) if s == "Stopped.");
                events.push(e);
                if stop {
                    break;
                }
            }
            assert!(
                Instant::now() < deadline,
                "acquisition did not finish: {events:?}"
            );
        }
        let d = shared.data.read().unwrap();
        assert_eq!(d.len(), 60);
        assert_eq!(d.time[1] - d.time[0], 1.0);
        assert_eq!(d.calctime[0], 4000);
        assert!(!d.flagsaved);
        assert_eq!(d.errorlevel, 0);
        assert!(events.iter().any(|e| matches!(e, DataEvent::Started)));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, DataEvent::Log(_, LogKind::Started)))
        );
        assert!(events.iter().any(|e| matches!(e, DataEvent::Stopped)));
        let w = port.written();
        assert_eq!(&w[..4], &[b'h', b'm', 0x42, 0x40]);
        assert_eq!(&w[4..9], &[b'I', 0, 0, 0, 0]);
        assert_eq!(w.iter().filter(|b| **b == b'G').count(), 4);
        assert!(w.contains(&b's'));
        assert_eq!(*w.last().unwrap(), b't');
        // The link is kept open for the next acquisition.
        assert!(shared.link.lock().unwrap().is_some());
        assert!(!shared.is_running());
    }

    #[test]
    fn handshake_failure_reports_error() {
        let shared = Shared::new();
        let port = MockPort::default();
        *shared.link.lock().unwrap() = Some(link(&port));
        let (tx, rx) = crossbeam_channel::unbounded();
        start_acquisition(shared.clone(), 3600.0, tx, Arc::new(|| {}));
        let mut got_error = false;
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            if let Ok(DataEvent::Status(Some(s))) = rx.recv_timeout(Duration::from_millis(100)) {
                if s.starts_with("Error:") {
                    got_error = true;
                    break;
                }
            }
        }
        assert!(got_error);
        std::thread::sleep(Duration::from_millis(50));
        assert!(!shared.is_running());
        assert_eq!(shared.data.read().unwrap().errorlevel, 1);
    }
}
