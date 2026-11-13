//! Real-time plots (port of `BaseFigQtGraph.py` and `figures.py`).

use egui::{Color32, Ui};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use serde::{Deserialize, Serialize};

use crate::dataman::DataStore;

pub const RED: Color32 = Color32::from_rgb(230, 0, 0);
pub const BLUE: Color32 = Color32::from_rgb(0, 0, 230);
pub const GREEN: Color32 = Color32::from_rgb(0, 170, 0);
pub const YELLOW: Color32 = Color32::from_rgb(210, 180, 0);
pub const PENS: [Color32; 4] = [RED, BLUE, GREEN, YELLOW];

/// Per-plot view configuration (time window and Y range), persisted.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PlotCfg {
    pub janelax: f64,
    pub miny: f64,
    pub maxy: f64,
    pub autoy: bool,
    #[serde(skip)]
    pub apply_y: bool,
    #[serde(skip)]
    pub view_all: bool,
    #[serde(skip)]
    pub ylim_text: [String; 2],
}

impl PlotCfg {
    pub fn with_y(miny: f64, maxy: f64) -> Self {
        Self {
            miny,
            maxy,
            ..Default::default()
        }
    }
}

impl Default for PlotCfg {
    fn default() -> Self {
        Self {
            janelax: 5.0,
            miny: 0.0,
            maxy: 256.0,
            autoy: false,
            apply_y: true,
            view_all: false,
            ylim_text: Default::default(),
        }
    }
}

/// Number of points in the time window and the matching x-axis values
/// (`np.linspace(-window, 0, npts)[-n:]`).
pub fn window_x(window: f64, ts: f64, n: usize) -> Vec<f64> {
    let npts = (window / ts) as usize;
    if npts == 0 {
        return vec![];
    }
    let step = if npts > 1 {
        window / (npts - 1) as f64
    } else {
        0.0
    };
    (npts - n..npts)
        .map(|j| -window + j as f64 * step)
        .collect()
}

/// Returns `(limi, limf)` indices of the data visible in the time window.
pub fn window_range(total: usize, window: f64, ts: f64) -> (usize, usize) {
    let npts = (window / ts) as usize;
    (total.saturating_sub(npts), total)
}

/// Min/max decimation to keep plotting fast for long windows.
fn decimate(x: &[f64], y: &[f64], max_points: usize) -> Vec<[f64; 2]> {
    let n = x.len().min(y.len());
    if n <= max_points {
        return (0..n).map(|i| [x[i], y[i]]).collect();
    }
    let buckets = max_points / 2;
    let size = n.div_ceil(buckets);
    let mut out = Vec::with_capacity(buckets * 2);
    for b in (0..n).step_by(size) {
        let e = (b + size).min(n);
        let (mut imin, mut imax) = (b, b);
        for i in b..e {
            if y[i] < y[imin] {
                imin = i;
            }
            if y[i] > y[imax] {
                imax = i;
            }
        }
        let (a, c) = if imin < imax {
            (imin, imax)
        } else {
            (imax, imin)
        };
        out.push([x[a], y[a]]);
        if c != a {
            out.push([x[c], y[c]]);
        }
    }
    out
}

pub struct Series {
    pub name: String,
    pub color: Color32,
    pub points: Vec<[f64; 2]>,
}

pub fn series(name: &str, color: Color32, x: &[f64], y: &[f64]) -> Series {
    Series {
        name: name.to_string(),
        color,
        points: decimate(x, y, 4000),
    }
}

/// Draws a time-window plot with the standard context menu.
/// `extra_menu` adds plot-specific controls (sensor selection, XYZ toggles).
pub fn time_plot(
    ui: &mut Ui,
    id: &str,
    cfg: &mut PlotCfg,
    height: f32,
    ylabel: &str,
    lines: Vec<Series>,
    extra_menu: impl FnOnce(&mut Ui),
) {
    let w = cfg.janelax;
    let pad = 0.01 * w;
    if cfg.view_all {
        cfg.view_all = false;
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for s in &lines {
            for p in &s.points {
                lo = lo.min(p[1]);
                hi = hi.max(p[1]);
            }
        }
        if lo.is_finite() && hi.is_finite() {
            if hi - lo < 1e-12 {
                lo -= 0.5;
                hi += 0.5;
            }
            cfg.miny = lo;
            cfg.maxy = hi;
            cfg.apply_y = true;
        }
    }
    let apply_y = std::mem::take(&mut cfg.apply_y);
    let (miny, maxy) = (cfg.miny, cfg.maxy);
    let ypad = 0.01 * (maxy - miny).abs();
    let has_names = lines.iter().any(|s| !s.name.is_empty());
    let mut plot = Plot::new(id)
        .height(height)
        .allow_drag([false, true])
        .allow_zoom([false, true])
        .allow_scroll([false, true])
        .allow_boxed_zoom(false)
        .allow_double_click_reset(false)
        .auto_bounds([false, false])
        .x_axis_label("Time (s)")
        .y_axis_label(ylabel)
        .y_axis_min_width(40.0);
    if has_names {
        plot = plot.legend(Legend::default());
    }
    let resp = plot.show(ui, |pui| {
        pui.set_plot_bounds_x(-w - pad..=pad);
        if apply_y {
            pui.set_plot_bounds_y(miny - ypad..=maxy + ypad);
        }
        for s in lines {
            pui.line(
                Line::new(s.name, PlotPoints::new(s.points))
                    .color(s.color)
                    .width(1.0),
            );
        }
        pui.plot_bounds()
    });
    if !apply_y {
        let b = resp.inner;
        let (lo, hi) = (b.min()[1], b.max()[1]);
        let span = hi - lo;
        // Undo the 1% padding to store the nominal range.
        cfg.miny = lo + span * 0.01 / 1.02;
        cfg.maxy = hi - span * 0.01 / 1.02;
    }
    let menu_id = egui::Id::unique((id, "menu_open"));
    // Close only on clicks outside the menu: the default (close on any click)
    // shuts the menu as soon as the sensor combo box, a checkbox or a text
    // field inside it is clicked.
    egui::Popup::context_menu(&resp.response)
        .close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)
        .show(|ui| {
        let was_open = ui
            .ctx()
            .data(|d| d.get_temp::<bool>(menu_id))
            .unwrap_or(false);
        if !was_open {
            cfg.ylim_text = [format!("{:.3}", cfg.miny), format!("{:.3}", cfg.maxy)];
            ui.ctx().data_mut(|d| d.insert_temp(menu_id, true));
        }
        ui.set_min_width(220.0);
        extra_menu(ui);
        ui.horizontal(|ui| {
            ui.label("Window:");
            let mut win = cfg.janelax.round() as i32;
            if ui
                .add(egui::DragValue::new(&mut win).range(1..=10).suffix(" s"))
                .changed()
            {
                cfg.janelax = win as f64;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Y Axis:");
            let r1 = ui.add(egui::TextEdit::singleline(&mut cfg.ylim_text[0]).desired_width(60.0));
            ui.label("to");
            let r2 = ui.add(egui::TextEdit::singleline(&mut cfg.ylim_text[1]).desired_width(60.0));
            if r1.lost_focus() || r2.lost_focus() {
                match (
                    cfg.ylim_text[0].trim().parse::<f64>(),
                    cfg.ylim_text[1].trim().parse::<f64>(),
                ) {
                    (Ok(a), Ok(b)) if a < b => {
                        cfg.miny = a;
                        cfg.maxy = b;
                        cfg.apply_y = true;
                    }
                    _ => cfg.ylim_text = [format!("{:.3}", cfg.miny), format!("{:.3}", cfg.maxy)],
                }
            }
        });
        if ui.button("View All").clicked() {
            cfg.view_all = true;
            ui.close();
        }
    });
    if !resp.response.context_menu_opened() {
        ui.ctx().data_mut(|d| d.remove::<bool>(menu_id));
    }
}

pub const SENSOR_CHOICES: [&str; 7] = [
    "Disabled",
    "IMU1 Accel",
    "IMU2 Accel",
    "IMU3 Accel",
    "IMU1 Gyro",
    "IMU2 Gyro",
    "IMU3 Gyro",
];

/// Sensor selection and XYZ toggles shown in the IMU plots' context menu.
/// The sensor list is a submenu: a ComboBox cannot be used here because egui
/// keeps a single popup open per viewport, so opening the combo's popup closes
/// the context menu (and the combo with it) immediately.
fn sensor_menu(ui: &mut Ui, sc: &mut usize, xyz: &mut [bool; 3]) {
    ui.menu_button(format!("Sensor: {}", SENSOR_CHOICES[*sc]), |ui| {
        for (i, name) in SENSOR_CHOICES.iter().enumerate() {
            if ui.add(egui::Button::selectable(*sc == i, *name)).clicked() {
                *sc = i;
            }
        }
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut xyz[0], "X");
        ui.checkbox(&mut xyz[1], "Y");
        ui.checkbox(&mut xyz[2], "Z");
    });
}

/// Main (measurement) figure: two IMU plots plus the ADC plot.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MainFig {
    pub plots: [PlotCfg; 3],
    pub sensor_choice: [usize; 2],
    pub xyz: [[bool; 3]; 2],
    #[serde(skip)]
    pub show_adc: bool,
}

impl Default for MainFig {
    fn default() -> Self {
        Self {
            plots: [
                PlotCfg::with_y(-20.0, 20.0),
                PlotCfg::with_y(-250.0, 250.0),
                PlotCfg::with_y(-5.0, 5.0),
            ],
            sensor_choice: [1, 4],
            xyz: [[true; 3]; 2],
            show_adc: false,
        }
    }
}

impl MainFig {
    fn ylabel(choice: usize) -> String {
        match choice {
            0 => "Accel. (m/s²)".into(),
            c if c < 4 => format!("IMU {} Accel. (m/s²)", (c - 1) % 3 + 1),
            c => format!("IMU {} Gyro (dg/s)", (c - 1) % 3 + 1),
        }
    }

    pub fn ui(&mut self, ui: &mut Ui, data: &DataStore) {
        let ts = data.samplingperiod;
        let total = data.len();
        let nplots = if self.show_adc { 3 } else { 2 };
        let h = ((ui.available_height() - 8.0) / nplots as f32).max(80.0);
        for k in 0..2 {
            let choice = self.sensor_choice[k];
            let cfg = &mut self.plots[k];
            let (limi, limf) = window_range(total, cfg.janelax, ts);
            let x = window_x(cfg.janelax, ts, limf - limi);
            let mut lines = Vec::new();
            if choice > 0 && limf > limi {
                let imu = (choice - 1) % 3;
                let src = if choice < 4 {
                    &data.acc[imu]
                } else {
                    &data.gyro[imu]
                };
                for axis in 0..3 {
                    if self.xyz[k][axis] {
                        lines.push(series("", PENS[axis], &x, &src[axis][limi..limf]));
                    }
                }
            }
            let sc = &mut self.sensor_choice[k];
            let xyz = &mut self.xyz[k];
            time_plot(
                ui,
                &format!("mainfig{k}"),
                cfg,
                h,
                &Self::ylabel(choice),
                lines,
                |ui| sensor_menu(ui, sc, xyz),
            );
        }
        if self.show_adc {
            let cfg = &mut self.plots[2];
            let (limi, limf) = window_range(total, cfg.janelax, ts);
            let x = window_x(cfg.janelax, ts, limf - limi);
            let lines = if limf > limi {
                vec![series("", RED, &x, &data.adc[0][limi..limf])]
            } else {
                vec![]
            };
            time_plot(ui, "mainfig2", cfg, h, "ADC (Volts)", lines, |_| {});
        }
    }
}

/// Control figure: reference and error signals.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct CtrlFig {
    pub plots: [PlotCfg; 2],
    #[serde(skip)]
    pub sensor_ids: [usize; 2],
}

impl Default for CtrlFig {
    fn default() -> Self {
        Self {
            plots: [PlotCfg::with_y(-1.0, 1.0), PlotCfg::with_y(-1.0, 1.0)],
            sensor_ids: [0, 0],
        }
    }
}

impl CtrlFig {
    pub fn ui(&mut self, ui: &mut Ui, data: &DataStore) {
        let ts = data.samplingperiod;
        let total = data.len();
        let h = ((ui.available_height() - 8.0) / 2.0).max(80.0);
        for k in 0..2 {
            let cfg = &mut self.plots[k];
            let (limi, limf) = window_range(total, cfg.janelax, ts);
            let x = window_x(cfg.janelax, ts, limf - limi);
            let (src, color, name) = if k == 0 {
                (&data.xref, RED, "Reference")
            } else {
                (&data.xerr, BLUE, "Error")
            };
            let unit = if self.sensor_ids[k] % 6 < 3 {
                "Acceleration (m/s²)"
            } else {
                "Gyro (dg/s)"
            };
            let lines = if limf > limi && src.len() >= limf {
                vec![series("", color, &x, &src[limi..limf])]
            } else {
                vec![]
            };
            time_plot(
                ui,
                &format!("ctrlfig{k}"),
                cfg,
                h,
                &format!("{name} {unit}"),
                lines,
                |_| {},
            );
        }
    }
}

/// Output figure: normalized generator outputs.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct OutFig {
    pub plot: PlotCfg,
    #[serde(skip)]
    pub dacenable: [bool; 4],
}

impl Default for OutFig {
    fn default() -> Self {
        Self {
            plot: PlotCfg::with_y(-1.1, 1.1),
            dacenable: [true, false, false, false],
        }
    }
}

impl OutFig {
    pub fn ui(&mut self, ui: &mut Ui, data: &DataStore, height: f32) {
        let ts = data.samplingperiod;
        let total = data.len();
        let cfg = &mut self.plot;
        let (limi, limf) = window_range(total, cfg.janelax, ts);
        let x = window_x(cfg.janelax, ts, limf - limi);
        let mut lines = Vec::new();
        if limf > limi {
            for k in 0..4 {
                let show = if data.ctrlmode {
                    k < 2
                } else {
                    self.dacenable[k]
                };
                if show {
                    lines.push(series(
                        "",
                        PENS[k].gamma_multiply(0.6),
                        &x,
                        &data.dac[k][limi..limf],
                    ));
                }
            }
        }
        time_plot(
            ui,
            "outfig",
            cfg,
            height,
            "Normalized output",
            lines,
            |_| {},
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_axis_matches_linspace_tail() {
        let x = window_x(1.0, 0.25, 2);
        // linspace(-1, 0, 4) = [-1, -2/3, -1/3, 0]
        assert_eq!(x.len(), 2);
        assert!((x[0] + 1.0 / 3.0).abs() < 1e-12);
        assert_eq!(x[1], 0.0);
        assert_eq!(window_range(10, 1.0, 0.25), (6, 10));
        assert_eq!(window_range(2, 1.0, 0.25), (0, 2));
    }

    #[test]
    fn decimation_keeps_extremes() {
        let x: Vec<f64> = (0..10000).map(|i| i as f64).collect();
        let mut y = vec![0.0; 10000];
        y[5000] = 10.0;
        y[7000] = -3.0;
        let d = decimate(&x, &y, 1000);
        assert!(d.len() <= 1000);
        assert!(d.iter().any(|p| p[1] == 10.0));
        assert!(d.iter().any(|p| p[1] == -3.0));
    }
}

#[cfg(test)]
mod menu_tests {
    //! Drives the real plot context menu headlessly: right-click the plot,
    //! pick a sensor from the submenu and toggle an axis checkbox. The menu
    //! must stay open throughout (regression: it closed on the first click).
    use super::*;
    use egui::{Event, PointerButton, pos2};
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;

    struct St {
        cfg: PlotCfg,
        sc: usize,
        xyz: [bool; 3],
    }

    #[test]
    fn sensor_can_be_selected_from_plot_menu() {
        let mut h = Harness::builder()
            .with_size(egui::vec2(800.0, 500.0))
            .with_accessibility_check(false).build_ui_state(
            |ui, st: &mut St| {
                let (cfg, sc, xyz) = (&mut st.cfg, &mut st.sc, &mut st.xyz);
                time_plot(ui, "p", cfg, 300.0, "y", vec![], |ui| sensor_menu(ui, sc, xyz));
            },
            St { cfg: PlotCfg::with_y(-1.0, 1.0), sc: 1, xyz: [true; 3] },
        );
        h.run();
        let p = pos2(400.0, 100.0);
        h.hover_at(p);
        for pressed in [true, false] {
            h.event(Event::PointerButton {
                pos: p,
                button: PointerButton::Secondary,
                pressed,
                modifiers: Default::default(),
            });
        }
        h.run();

        h.get_by_label("Sensor: IMU1 Accel").click();
        h.run();
        h.get_by_label("IMU2 Gyro").click();
        h.run();
        assert_eq!(h.state().sc, 5);
        // The menu is still open and shows the new choice.
        h.get_by_label("Sensor: IMU2 Gyro");

        h.get_by_label("Y").click();
        h.run();
        assert_eq!(h.state().xyz, [true, false, true]);
        h.get_by_label("Sensor: IMU2 Gyro");
    }
}
