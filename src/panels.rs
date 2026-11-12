//! Configuration panels: IMU, signal generators, control and ADC
//! (port of `panels.py` and the corresponding `.ui` forms).

use egui::{DragValue, Ui};
use serde::{Deserialize, Serialize};

use crate::driver::{ControlParams, GeneratorParams};
use crate::table::fmt_f64;

/// Combo box with optionally disabled items. Returns true if the selection changed.
pub fn combo(
    ui: &mut Ui,
    id: impl egui::AsIdSalt,
    idx: &mut usize,
    items: &[&str],
    item_enabled: Option<&[bool]>,
) -> bool {
    let mut changed = false;
    let selected = items.get(*idx).copied().unwrap_or("");
    egui::ComboBox::from_id_salt(id)
        .selected_text(selected)
        .show_ui(ui, |ui| {
            for (i, it) in items.iter().enumerate() {
                let en = item_enabled
                    .map(|e| e.get(i).copied().unwrap_or(true))
                    .unwrap_or(true);
                if ui
                    .add_enabled(en, egui::Button::selectable(*idx == i, *it))
                    .clicked()
                    && *idx != i
                {
                    *idx = i;
                    changed = true;
                }
            }
        });
    changed
}

// ---------------------------------------------------------------- IMU panel

pub const IMU_TYPES: [&str; 3] = ["Disabled", "MPU6050", "LSM6DS3"];
pub const IMU_ADDRESSES: [&str; 6] = ["0x68", "0x69", "0x6A", "0x6B", "CS5", "CS33"];
pub const IMU_BUSES: [&str; 3] = ["I2C-1", "I2C-2", "VSPI"];
pub const ACC_RANGES: [&str; 4] = [
    "±2g (±19,6m/s²)",
    "±4g (±39,2m/s²)",
    "±8g (±78,4m/s²)",
    "±16g (±156,9m/s²)",
];
pub const GYRO_RANGES: [&str; 5] = ["±125º/s", "±250º/s", "±500º/s", "±1000º/s", "±2000º/s"];
pub const MPU_FILTERS: [&str; 7] = [
    "260 Hz", "184 Hz", "94 Hz", "44 Hz", "21 Hz", "10 Hz", "5 Hz",
];
pub const LSM_FILTERS: [&str; 5] = ["Auto", "400 Hz", "200 Hz", "100 Hz", "50 Hz"];

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
#[derive(Default)]
pub struct ImuPanel {
    pub imu_type: usize,
    pub address: usize,
    pub bus: usize,
    pub acc_range: usize,
    pub gyro_range: usize,
    pub filter: usize,
}

impl ImuPanel {
    pub fn is_enabled(&self) -> bool {
        self.imu_type > 0
    }

    pub fn filters(&self) -> &'static [&'static str] {
        if self.imu_type == 1 {
            &MPU_FILTERS
        } else {
            &LSM_FILTERS
        }
    }

    fn address_items_enabled(&self) -> [bool; 6] {
        match self.imu_type {
            1 => [true, true, false, false, false, false],
            2 if self.bus == 2 => [false, false, false, false, true, true],
            2 => [false, false, true, true, false, false],
            _ => [true; 6],
        }
    }

    /// Enforces the valid address/range/bus combinations (port of `addresshighlight`).
    pub fn normalize(&mut self) {
        match self.imu_type {
            1 => {
                if self.gyro_range == 0 {
                    self.gyro_range = 1;
                }
                if self.address > 1 {
                    self.address = 0;
                }
                if self.bus == 2 {
                    self.bus = 0;
                }
            }
            2 => {
                if self.bus == 2 {
                    if self.address < 4 {
                        self.address = 4;
                    }
                } else if self.address < 2 || self.address > 3 {
                    self.address = 2;
                }
            }
            _ => {}
        }
        if self.filter >= self.filters().len() {
            self.filter = 0;
        }
    }

    /// Three configuration bytes sent with the `I` command.
    pub fn config_bytes(&mut self) -> [u8; 3] {
        let imutype = (self.imu_type == 2) as u8;
        let enabled = (self.imu_type > 0) as u8;
        let b0 = ((self.bus as u8) << 2) + (imutype << 1) + enabled;
        let b1 = if self.bus < 2 {
            u8::from_str_radix(&IMU_ADDRESSES[self.address.min(5)][2..], 16).unwrap_or(0)
        } else {
            if self.address < 4 {
                self.address = 4;
            }
            IMU_ADDRESSES[self.address.min(5)][2..]
                .parse::<u8>()
                .unwrap_or(0)
        };
        let gyro = match self.imu_type {
            1 => {
                if self.gyro_range < 1 {
                    self.gyro_range = 1;
                }
                self.gyro_range as u8
            }
            2 => self.gyro_range as u8,
            _ => 0,
        };
        let b2 = ((self.filter as u8) << 5) + (gyro << 2) + self.acc_range as u8;
        [b0, b1, b2]
    }

    pub fn log_string(&self) -> String {
        let t = IMU_TYPES[self.imu_type];
        if self.imu_type == 0 {
            return t.to_string();
        }
        [
            t,
            IMU_ADDRESSES[self.address],
            IMU_BUSES[self.bus],
            ACC_RANGES[self.acc_range],
            GYRO_RANGES[self.gyro_range],
            self.filters().get(self.filter).copied().unwrap_or(""),
        ]
        .join("|")
    }

    /// Draws the panel. `running` disables everything.
    pub fn ui(&mut self, ui: &mut Ui, id: usize, running: bool) -> bool {
        let mut changed = false;
        let en = !running && self.imu_type > 0;
        egui::Grid::new(("imu_grid", id))
            .num_columns(4)
            .spacing([6.0, 4.0])
            .show(ui, |ui| {
                ui.label("Type:");
                ui.add_enabled_ui(!running, |ui| {
                    if combo(ui, ("imutype", id), &mut self.imu_type, &IMU_TYPES, None) {
                        self.normalize();
                        changed = true;
                    }
                });
                ui.label("Bus:");
                ui.add_enabled_ui(en, |ui| {
                    let bus_en = [true, true, self.imu_type != 1];
                    if combo(ui, ("imubus", id), &mut self.bus, &IMU_BUSES, Some(&bus_en)) {
                        self.normalize();
                        changed = true;
                    }
                });
                ui.end_row();
                ui.label("Addr:");
                ui.add_enabled_ui(en, |ui| {
                    let items = self.address_items_enabled();
                    changed |= combo(
                        ui,
                        ("imuaddr", id),
                        &mut self.address,
                        &IMU_ADDRESSES,
                        Some(&items),
                    );
                });
                ui.label("AccR:");
                ui.add_enabled_ui(en, |ui| {
                    changed |= combo(ui, ("imuacc", id), &mut self.acc_range, &ACC_RANGES, None);
                });
                ui.end_row();
                ui.label("GyroR:");
                ui.add_enabled_ui(en, |ui| {
                    let g_en = [self.imu_type != 1, true, true, true, true];
                    changed |= combo(
                        ui,
                        ("imugyro", id),
                        &mut self.gyro_range,
                        &GYRO_RANGES,
                        Some(&g_en),
                    );
                });
                ui.label("Filter:");
                ui.add_enabled_ui(en, |ui| {
                    let f = self.filters();
                    changed |= combo(ui, ("imufilt", id), &mut self.filter, f, None);
                });
                ui.end_row();
            });
        changed
    }
}

// ---------------------------------------------------------- Generator panel

pub const GEN_TYPES: [&str; 5] = ["Noise", "Harmonic", "Chirp", "Step", "Square"];

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct GeneratorPanel {
    #[serde(skip)]
    pub id: usize,
    pub enable: bool,
    pub gen_type: usize,
    pub ampl: f64,
    pub freq: f64,
    pub dclevel: u16,
    pub chirp_tinicio: u8,
    pub chirp_deltai: f64,
    pub chirp_tfim: u8,
    pub chirp_deltaf: f64,
    pub chirp_a2: f64,
}

impl Default for GeneratorPanel {
    fn default() -> Self {
        Self::new(0)
    }
}

impl GeneratorPanel {
    pub fn new(id: usize) -> Self {
        Self {
            id,
            enable: true,
            gen_type: 0,
            ampl: 0.0,
            freq: 10.0,
            dclevel: if id < 2 { 1668 } else { 128 },
            chirp_tinicio: 10,
            chirp_deltai: 10.0,
            chirp_tfim: 100,
            chirp_deltaf: 10.0,
            chirp_a2: 0.0,
        }
    }

    pub fn max_dclevel(&self) -> u16 {
        if self.id < 2 { 2048 } else { 128 }
    }

    pub fn clamp(&mut self) {
        self.dclevel = self.dclevel.min(self.max_dclevel());
        self.ampl = self.ampl.clamp(0.0, 1.0);
        self.freq = self.freq.clamp(0.0, 125.0);
        self.chirp_deltai = self.chirp_deltai.clamp(0.1, 25.5);
        self.chirp_deltaf = self.chirp_deltaf.clamp(0.1, 25.5);
        self.chirp_a2 = self.chirp_a2.clamp(0.0, 1.0);
        if self.gen_type >= GEN_TYPES.len() {
            self.gen_type = 0;
        }
    }

    /// `panel_enabled` reflects control-mode channel assignment.
    pub fn is_on(&self, panel_enabled: bool) -> bool {
        panel_enabled && self.enable
    }

    pub fn params(&self, panel_enabled: bool) -> GeneratorParams {
        if self.is_on(panel_enabled) {
            GeneratorParams {
                tipo: self.gen_type as u8,
                amp: self.ampl,
                freq: self.freq,
                dclevel: self.dclevel,
                chirpconf: (self.gen_type == 2).then_some([
                    self.chirp_tinicio as f64,
                    self.chirp_deltai,
                    self.chirp_tfim as f64,
                    self.chirp_deltaf,
                    self.chirp_a2,
                ]),
                pwmduty: 0.0,
            }
        } else {
            GeneratorParams {
                tipo: 0,
                amp: 0.0,
                freq: 0.0,
                dclevel: self.dclevel,
                chirpconf: None,
                pwmduty: 0.0,
            }
        }
    }

    pub fn log_string(&self, panel_enabled: bool) -> String {
        if !self.is_on(panel_enabled) {
            return "Disabled".into();
        }
        let mut info = vec![
            "Enabled".to_string(),
            GEN_TYPES[self.gen_type].to_string(),
            fmt_f64(self.ampl),
            fmt_f64(self.freq),
            self.dclevel.to_string(),
        ];
        if self.gen_type == 2 {
            info.push(format!(
                "{},{},{},{},{}",
                self.chirp_tinicio,
                fmt_f64(self.chirp_deltai),
                self.chirp_tfim,
                fmt_f64(self.chirp_deltaf),
                fmt_f64(self.chirp_a2)
            ));
        }
        info.join("|")
    }

    /// Sets type (by name or index), amplitude and frequency (used by the automator).
    pub fn set_config(&mut self, signaltype: &str, amp: f64, freq: f64) {
        if let Some(i) = GEN_TYPES
            .iter()
            .position(|t| t.eq_ignore_ascii_case(signaltype))
        {
            self.gen_type = i;
        } else if let Ok(i) = signaltype.parse::<usize>() {
            self.gen_type = i.min(GEN_TYPES.len() - 1);
        }
        self.ampl = amp;
        self.freq = freq;
        self.clamp();
    }

    pub fn ui(&mut self, ui: &mut Ui, panel_enabled: bool) -> bool {
        let mut changed = false;
        let t = self.gen_type;
        let freq_en = matches!(t, 1 | 2 | 4);
        let chirp_full = t == 2;
        let step = t == 3;
        let id = self.id;
        ui.add_enabled_ui(panel_enabled, |ui| {
            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut self.enable, "Enable").changed();
                changed |= combo(ui, ("gentype", id), &mut self.gen_type, &GEN_TYPES, None);
                ui.label("DCLevel:");
                let max = self.max_dclevel();
                changed |= ui
                    .add(DragValue::new(&mut self.dclevel).range(0..=max))
                    .changed();
            });
            ui.horizontal(|ui| {
                ui.label("Ampl.:");
                changed |= ui
                    .add(
                        DragValue::new(&mut self.ampl)
                            .range(0.0..=1.0)
                            .speed(0.005)
                            .fixed_decimals(3),
                    )
                    .changed();
                ui.label("Freq.:");
                changed |= ui
                    .add_enabled(
                        freq_en,
                        DragValue::new(&mut self.freq)
                            .range(0.0..=125.0)
                            .speed(0.1)
                            .fixed_decimals(2),
                    )
                    .changed();
            });
        });
        ui.horizontal(|ui| {
            ui.label("ST:");
            changed |= ui
                .add_enabled(
                    panel_enabled && (chirp_full || step),
                    DragValue::new(&mut self.chirp_tinicio).range(0..=255),
                )
                .changed();
            ui.label("DI:");
            changed |= ui
                .add_enabled(
                    panel_enabled && chirp_full,
                    DragValue::new(&mut self.chirp_deltai)
                        .range(0.1..=25.5)
                        .speed(0.1)
                        .fixed_decimals(1),
                )
                .changed();
            ui.label("ET:");
            changed |= ui
                .add_enabled(
                    panel_enabled && (chirp_full || step),
                    DragValue::new(&mut self.chirp_tfim).range(0..=255),
                )
                .changed();
            ui.label("DF:");
            changed |= ui
                .add_enabled(
                    panel_enabled && chirp_full,
                    DragValue::new(&mut self.chirp_deltaf)
                        .range(0.1..=25.5)
                        .speed(0.1)
                        .fixed_decimals(1),
                )
                .changed();
            ui.label("A2:");
            changed |= ui
                .add_enabled(
                    panel_enabled && chirp_full,
                    DragValue::new(&mut self.chirp_a2)
                        .range(0.0..=1.0)
                        .speed(0.005)
                        .fixed_decimals(3),
                )
                .changed();
        });
        changed
    }
}

// ------------------------------------------------------------ Control panel

pub const OUTPUTS: [&str; 4] = ["Output 1", "Output 2", "Output 3", "Output 4"];
pub const IMUS: [&str; 3] = ["IMU 1", "IMU 2", "IMU 3"];
pub const SENSORS: [&str; 7] = [
    "X Accel",
    "Y Accel",
    "Z Accel",
    "X Gyro",
    "Y Gyro",
    "Z Gyro",
    "Z Accel + X Gyro",
];
pub const TASKS: [&str; 2] = ["Control", "Path Modelling"];
pub const ALGORITHMS: [&str; 4] = [
    "FxNLMS",
    "FxNLMS with full buffers",
    "CVA-FxNLMS",
    "CVA-FxNLMS with full buffers",
];

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct ControlPanel {
    pub perturb_channel: usize,
    pub control_channel: usize,
    pub imu_ref: usize,
    pub ref_sensor: usize,
    pub imu_err: usize,
    pub err_sensor: usize,
    pub task: usize,
    #[serde(skip)]
    pub alg_on: bool,
    pub t_alg_on: u32,
    pub algorithm: usize,
    pub mem: u16,
    pub step: String,
    pub penalization: String,
    #[serde(skip)]
    old_mu: f64,
    #[serde(skip)]
    old_psi: f64,
}

impl Default for ControlPanel {
    fn default() -> Self {
        Self {
            perturb_channel: 1,
            control_channel: 0,
            imu_ref: 0,
            ref_sensor: 0,
            imu_err: 1,
            err_sensor: 0,
            task: 0,
            alg_on: false,
            t_alg_on: 0,
            algorithm: 0,
            mem: 100,
            step: "0.01".into(),
            penalization: "1e-4".into(),
            old_mu: 0.01,
            old_psi: 1e-4,
        }
    }
}

/// What the control panel asks the main window to do.
#[derive(Default)]
pub struct ControlPanelResponse {
    pub config_changed: bool,
    pub alg_on_toggled: bool,
}

pub const STEP_FIELD_ID: &str = "ctrl_step_field";

impl ControlPanel {
    pub fn is_task_control(&self) -> bool {
        self.task == 0
    }

    fn parse_or(s: &str, default: f64) -> f64 {
        s.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .unwrap_or(default)
    }

    pub fn mu(&self) -> f64 {
        Self::parse_or(&self.step, 0.01)
    }

    pub fn psi(&self) -> f64 {
        Self::parse_or(&self.penalization, 0.01)
    }

    pub fn params(&mut self) -> ControlParams {
        if self.penalization.trim().parse::<f64>().is_err() {
            self.penalization = "0.01".into();
        }
        if self.step.trim().parse::<f64>().is_err() {
            self.step = "0.01".into();
        }
        ControlParams {
            alg: self.algorithm as u8,
            mem: self.mem,
            mu: self.mu() as f32,
            fi: self.psi() as f32,
            refimuid: self.imu_ref as u8,
            errimuid: self.imu_err as u8,
            refid: self.ref_sensor as u8,
            erroid: self.err_sensor as u8,
            ctrltask: self.task as u8,
        }
    }

    pub fn log_string(&self, control_on: bool) -> String {
        if !control_on {
            return "Disabled".into();
        }
        [
            "Enabled".to_string(),
            TASKS[self.task].into(),
            OUTPUTS[self.perturb_channel].into(),
            OUTPUTS[self.control_channel].into(),
            IMUS[self.imu_ref].into(),
            SENSORS[self.ref_sensor].into(),
            IMUS[self.imu_err].into(),
            SENSORS[self.err_sensor].into(),
            if self.alg_on {
                "AlgOn".into()
            } else {
                "AlgOff".into()
            },
            self.t_alg_on.to_string(),
            ALGORITHMS[self.algorithm].into(),
            self.mem.to_string(),
            self.step.clone(),
            self.penalization.clone(),
        ]
        .join("|")
    }

    pub fn alg_on_log_string(&self) -> String {
        if self.alg_on {
            format!(
                "AlgOn|{}|{}|{}",
                self.step, self.penalization, self.t_alg_on
            )
        } else {
            "AlgOff".into()
        }
    }

    fn fix_channels(&mut self, control_changed: bool) {
        if self.control_channel == self.perturb_channel {
            if control_changed {
                self.perturb_channel = (self.control_channel + 1) % 4;
            } else {
                self.control_channel = (self.perturb_channel + 1) % 4;
            }
        }
    }

    fn validated_float(
        ui: &mut Ui,
        text: &mut String,
        old: &mut f64,
        enabled: bool,
        id: Option<&str>,
    ) -> bool {
        let mut te = egui::TextEdit::singleline(text).desired_width(70.0);
        if let Some(id) = id {
            te = te.id(egui::Id::unique(id));
        }
        let r = ui.add_enabled(enabled, te);
        if r.lost_focus() {
            match text.trim().parse::<f64>() {
                Ok(v) if (0.0..=1000.0).contains(&v) => {
                    *old = v;
                    return true;
                }
                _ => *text = fmt_f64(*old),
            }
        }
        false
    }

    /// `control_on`: main mode is Control. `running`: acquisition in progress.
    pub fn ui(&mut self, ui: &mut Ui, control_on: bool, running: bool) -> ControlPanelResponse {
        let mut resp = ControlPanelResponse::default();
        let task_ctrl = self.is_task_control();
        let cmps_en = control_on && !running;
        let alg_en = control_on && task_ctrl && !running;
        let algon_en = control_on && task_ctrl && running;
        let params_en = control_on && task_ctrl && !self.alg_on;
        egui::Grid::new("ctrl_grid")
            .num_columns(4)
            .spacing([6.0, 4.0])
            .show(ui, |ui| {
                ui.label("Task:");
                ui.add_enabled_ui(cmps_en, |ui| {
                    resp.config_changed |= combo(ui, "ctrltask", &mut self.task, &TASKS, None);
                });
                ui.label("Algorithm:");
                ui.add_enabled_ui(alg_en, |ui| {
                    resp.config_changed |=
                        combo(ui, "ctrlalg", &mut self.algorithm, &ALGORITHMS, None);
                });
                ui.end_row();

                ui.label("Perturb.:");
                ui.add_enabled_ui(cmps_en, |ui| {
                    if combo(ui, "ctrlperturb", &mut self.perturb_channel, &OUTPUTS, None) {
                        self.fix_channels(false);
                        resp.config_changed = true;
                    }
                });
                ui.label("Control:");
                ui.add_enabled_ui(cmps_en, |ui| {
                    if combo(ui, "ctrlcontrol", &mut self.control_channel, &OUTPUTS, None) {
                        self.fix_channels(true);
                        resp.config_changed = true;
                    }
                });
                ui.end_row();

                ui.label("Ref.:");
                ui.add_enabled_ui(cmps_en, |ui| {
                    ui.horizontal(|ui| {
                        resp.config_changed |=
                            combo(ui, "ctrlimuref", &mut self.imu_ref, &IMUS, None);
                        resp.config_changed |=
                            combo(ui, "ctrlref", &mut self.ref_sensor, &SENSORS, None);
                    });
                });
                ui.label("Error:");
                ui.add_enabled_ui(cmps_en, |ui| {
                    ui.horizontal(|ui| {
                        resp.config_changed |=
                            combo(ui, "ctrlimuerr", &mut self.imu_err, &IMUS, None);
                        resp.config_changed |=
                            combo(ui, "ctrlerr", &mut self.err_sensor, &SENSORS, None);
                    });
                });
                ui.end_row();

                ui.label("Memory size:");
                resp.config_changed |= ui
                    .add_enabled(alg_en, DragValue::new(&mut self.mem).range(1..=1000))
                    .changed();
                ui.label("Step size:");
                resp.config_changed |= Self::validated_float(
                    ui,
                    &mut self.step,
                    &mut self.old_mu,
                    params_en,
                    Some(STEP_FIELD_ID),
                );
                ui.end_row();

                ui.label("Penalization:");
                resp.config_changed |= Self::validated_float(
                    ui,
                    &mut self.penalization,
                    &mut self.old_psi,
                    params_en,
                    None,
                );
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(algon_en, egui::Checkbox::new(&mut self.alg_on, "On ≥"))
                        .changed()
                    {
                        resp.alg_on_toggled = true;
                    }
                });
                ui.add_enabled(
                    params_en,
                    DragValue::new(&mut self.t_alg_on)
                        .range(0..=99)
                        .suffix(" s"),
                );
                ui.end_row();
            });
        resp
    }
}

// ---------------------------------------------------------------- ADC panel

pub const ADC_MODELS: [&str; 2] = ["ADS1115", "ADS1015"];
pub const ADC_RANGES: [&str; 6] = [
    "±6.144 V",
    "±4.096 V",
    "±2.048 V",
    "±1.024 V",
    "±0.512 V",
    "±0.256 V",
];
pub const RATES_1115: [&str; 8] = [
    "8 SPS", "16 SPS", "32 SPS", "64 SPS", "128 SPS", "250 SPS", "475 SPS", "860 SPS",
];
pub const RATES_1015: [&str; 7] = [
    "128 SPS", "250 SPS", "490 SPS", "920 SPS", "1600 SPS", "2400 SPS", "3300 SPS",
];

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct AdcPanel {
    pub model: usize,
    /// 0 = Off, 1..=4 = channel
    pub channel: usize,
    pub range: usize,
    pub rate1115: usize,
    pub rate1015: usize,
}

impl AdcPanel {
    pub fn config(&self) -> [u8; 3] {
        let is1115 = self.model == 0;
        let mut flagon = if is1115 { 0 } else { 16 };
        if (1..=4).contains(&self.channel) {
            flagon += 1 << (self.channel - 1);
        }
        [
            flagon,
            self.range as u8,
            if is1115 {
                self.rate1115 as u8
            } else {
                self.rate1015 as u8
            },
        ]
    }

    pub fn log_string(&self) -> String {
        let bits: Vec<&str> = (1..=4)
            .map(|c| if self.channel == c { "1" } else { "0" })
            .collect();
        let rate = if self.model == 1 {
            RATES_1015[self.rate1015]
        } else {
            RATES_1115[self.rate1115]
        };
        format!(
            "{}|{}|{}|{}",
            ADC_MODELS[self.model],
            bits.join("|"),
            ADC_RANGES[self.range],
            rate
        )
    }

    pub fn ui(&mut self, ui: &mut Ui, running: bool) -> bool {
        let mut changed = false;
        ui.add_enabled_ui(!running, |ui| {
            egui::Grid::new("adc_grid")
                .num_columns(2)
                .spacing([6.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Model:");
                    changed |= combo(ui, "adcmodel", &mut self.model, &ADC_MODELS, None);
                    ui.end_row();
                    ui.label("Channel:");
                    ui.horizontal(|ui| {
                        changed |= ui.radio_value(&mut self.channel, 0, "Off").changed();
                        for c in 1..=4 {
                            changed |= ui
                                .radio_value(&mut self.channel, c, format!("{c}"))
                                .changed();
                        }
                    });
                    ui.end_row();
                    ui.label("Range:");
                    changed |= combo(ui, "adcrange", &mut self.range, &ADC_RANGES, None);
                    ui.end_row();
                    ui.label("Rate:");
                    if self.model == 0 {
                        changed |= combo(ui, "adcrate1115", &mut self.rate1115, &RATES_1115, None);
                    } else {
                        changed |= combo(ui, "adcrate1015", &mut self.rate1015, &RATES_1015, None);
                    }
                    ui.end_row();
                });
        });
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_bytes() {
        let mut p = ImuPanel {
            imu_type: 1,
            address: 1,
            bus: 1,
            acc_range: 2,
            gyro_range: 0,
            filter: 3,
        };
        assert_eq!(p.config_bytes(), [0b101, 0x69, (3 << 5) | (1 << 2) | 2]);
        let mut p = ImuPanel {
            imu_type: 2,
            address: 0,
            bus: 2,
            acc_range: 0,
            gyro_range: 4,
            filter: 0,
        };
        p.normalize();
        assert_eq!(p.config_bytes(), [0b1011, 5, 4 << 2]);
        let mut p = ImuPanel::default();
        assert_eq!(p.config_bytes(), [0, 0x68, 0]);
    }

    #[test]
    fn adc_config_bits() {
        let a = AdcPanel {
            model: 1,
            channel: 3,
            range: 2,
            rate1115: 0,
            rate1015: 5,
        };
        assert_eq!(a.config(), [16 + 4, 2, 5]);
        assert_eq!(a.log_string(), "ADS1015|0|0|1|0|±2.048 V|2400 SPS");
    }

    #[test]
    fn generator_params_when_disabled() {
        let mut g = GeneratorPanel::new(0);
        g.gen_type = 1;
        g.ampl = 0.5;
        assert_eq!(g.params(true).tipo, 1);
        let off = g.params(false);
        assert_eq!((off.tipo, off.amp, off.dclevel), (0, 0.0, 1668));
        assert_eq!(g.log_string(true), "Enabled|Harmonic|0.5|10.0|1668");
    }

    #[test]
    fn control_channels_never_collide() {
        let mut c = ControlPanel {
            perturb_channel: 1,
            control_channel: 1,
            ..Default::default()
        };
        c.fix_channels(true);
        assert_eq!((c.control_channel, c.perturb_channel), (1, 2));
    }
}
