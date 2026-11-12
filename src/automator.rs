//! Experiment automator (port of `automator.py`). The sequence runs as a
//! state machine driven by the UI loop instead of a separate thread.
//!
//! Commands (parameters are comma-separated):
//!  - Reset
//!  - ConfigOutput: number (1-4), Enabled/True|False[, type, ampl, freq]
//!  - ConfigIMU: number (1-3), type (Disabled/MPU6050/LSM6DS3)[, addr, bus, accR, gyroR, filter1, filter2]
//!  - ConfigADC: model, channel (Off/1-4), range (V), rate (SPS)
//!  - SetReadingMode / SetPathModeling
//!  - ControlChannels: perturbation, control, refIMU, refSensor, errIMU, errSensor
//!  - SetControlMode: algorithm, algOnTime, memSize, stepSize, penalization
//!  - AlgOn: true/false
//!  - Start / StartWait / StartNotify: stoptime
//!  - Stop, Wait, Delay: seconds, Print: text, SaveFile: name, SetWorkDir: dir

use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use egui::{Color32, RichText};

pub type Command = (String, Vec<String>);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AState {
    Ready,
    Delay(Instant),
    /// Waiting for a resume signal from the main window.
    Waiting,
}

/// What the main window did with an action.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActionOutcome {
    /// Continue immediately with the next command.
    Resume,
    /// Wait for a `resume()` call (e.g. acquisition stopped/started).
    Wait,
    /// Stop the sequence (error already reported).
    Abort,
}

pub struct Automator {
    pub open: bool,
    pub list: Vec<Command>,
    pub running: bool,
    pub idx: usize,
    pub state: AState,
    start: Instant,
    pub messages: Vec<(String, bool)>,
}

fn s(v: &str) -> String {
    v.to_string()
}

impl Default for Automator {
    fn default() -> Self {
        Self::new()
    }
}

impl Automator {
    pub fn new() -> Self {
        let list = vec![
            (s("Reset"), vec![]),
            (s("ConfigOutput"), vec![s("1"), s("False")]),
            (
                s("ConfigOutput"),
                vec![s("2"), s("True"), s("Noise"), s("0.75"), s("10.0")],
            ),
            (s("SetPathModeling"), vec![]),
            (s("Start"), vec![s("10")]),
            (s("Wait"), vec![]),
            (s("Reset"), vec![]),
            (
                s("ConfigOutput"),
                vec![s("1"), s("True"), s("Harmonic"), s("0.25"), s("38.0")],
            ),
            (s("ConfigOutput"), vec![s("2"), s("False")]),
            (
                s("SetControlMode"),
                vec![s("CVA-FxNLMS"), s("5"), s("250"), s("0.001"), s("1e-2")],
            ),
            (s("Start"), vec![s("5")]),
            (s("Delay"), vec![s("2")]),
            (s("AlgOn"), vec![s("True")]),
            (s("Wait"), vec![]),
            (s("Print"), vec![s("Finished the sequence!")]),
        ];
        Self {
            open: false,
            list,
            running: false,
            idx: 0,
            state: AState::Ready,
            start: Instant::now(),
            messages: vec![],
        }
    }

    pub fn elapsed(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    /// Parses a tab-separated command list: `Command<TAB>param1, param2, ...`.
    pub fn parse(text: &str) -> Result<Vec<Command>> {
        let mut out = Vec::new();
        for line in text.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let parts: Vec<&str> = line.split('\t').collect();
            let (cmd, params) = match parts.len() {
                1 => (parts[0], ""),
                2 => (parts[0], parts[1]),
                _ => bail!("Error parsing command list!"),
            };
            let params = if params.trim().is_empty() {
                vec![]
            } else {
                params.split(',').map(|p| p.trim().to_string()).collect()
            };
            out.push((cmd.trim().to_string(), params));
        }
        Ok(out)
    }

    pub fn load_text(&mut self, text: &str) {
        match Self::parse(text) {
            Ok(l) if !l.is_empty() => {
                self.list = l;
                self.idx = 0;
            }
            Ok(_) => self.print_error("Empty command list."),
            Err(e) => self.print_error(&e.to_string()),
        }
    }

    pub fn load_file(&mut self, path: &Path) {
        match std::fs::read_to_string(path) {
            Ok(t) => self.load_text(&t),
            Err(e) => self.print_error(&format!("Error reading file: {e}")),
        }
    }

    pub fn print(&mut self, msg: impl Into<String>) {
        self.messages.push((msg.into(), false));
    }

    pub fn print_error(&mut self, msg: &str) {
        self.messages.push((msg.to_string(), true));
    }

    pub fn start(&mut self) {
        if self.list.is_empty() {
            return;
        }
        self.running = true;
        self.idx = 0;
        self.state = AState::Ready;
        self.start = Instant::now();
    }

    /// Stops the sequence; the caller must also run the "Stopping" action.
    pub fn stop(&mut self) {
        if self.running {
            self.running = false;
            self.state = AState::Ready;
            let msg = format!("{:.0}: Finished!", self.elapsed());
            self.print(msg);
        }
    }

    pub fn resume(&mut self) {
        if self.running && self.state == AState::Waiting {
            self.idx += 1;
            self.state = AState::Ready;
        }
    }

    /// Returns the next command to execute, handling delays internally.
    pub fn next_action(&mut self) -> Option<Command> {
        if !self.running {
            return None;
        }
        if let AState::Delay(until) = self.state {
            if Instant::now() >= until {
                self.idx += 1;
                self.state = AState::Ready;
            } else {
                return None;
            }
        }
        if self.state != AState::Ready {
            return None;
        }
        if self.idx >= self.list.len() {
            self.stop();
            return None;
        }
        let cmd = self.list[self.idx].clone();
        if cmd.0 == "Delay" {
            let secs = cmd
                .1
                .first()
                .and_then(|v| v.parse::<f64>().ok())
                .unwrap_or(0.0)
                .max(0.0);
            let msg = format!("{:.0}: Delay - {:?}", self.elapsed(), cmd.1);
            self.print(msg);
            self.state = AState::Delay(Instant::now() + Duration::from_secs_f64(secs.trunc()));
            return None;
        }
        Some(cmd)
    }

    /// Records the outcome of the action returned by `next_action`.
    pub fn complete(&mut self, outcome: ActionOutcome) {
        match outcome {
            ActionOutcome::Resume => {
                if self.running {
                    self.idx += 1;
                }
            }
            ActionOutcome::Wait => self.state = AState::Waiting,
            ActionOutcome::Abort => self.stop(),
        }
    }

    /// Draws the window. Returns true if "Stop" was pressed.
    pub fn show(&mut self, ctx: &egui::Context) -> bool {
        let mut stop_pressed = false;
        // Ctrl+V pastes a command list when no text field has focus.
        if self.open && !ctx.egui_wants_keyboard_input() {
            let pasted: Option<String> = ctx.input(|i| {
                i.events.iter().find_map(|e| {
                    if let egui::Event::Paste(t) = e {
                        Some(t.clone())
                    } else {
                        None
                    }
                })
            });
            if let Some(t) = pasted
                && !self.running
            {
                self.load_text(&t);
            }
        }
        if let AState::Delay(_) = self.state {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        let mut open = self.open;
        egui::Window::new("Automator")
            .open(&mut open)
            .default_size([640.0, 520.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!self.running, egui::Button::new("Start"))
                        .clicked()
                    {
                        self.messages.clear();
                        self.start();
                    }
                    if ui
                        .add_enabled(self.running, egui::Button::new("Stop"))
                        .clicked()
                    {
                        stop_pressed = true;
                    }
                    if ui
                        .add_enabled(!self.running, egui::Button::new("Load..."))
                        .clicked()
                        && let Some(p) = rfd::FileDialog::new()
                            .add_filter("Command list", &["txt", "tsv"])
                            .pick_file()
                    {
                        self.load_file(&p);
                    }
                    if ui.button("Clear").clicked() {
                        self.messages.clear();
                    }
                });
                ui.small("Paste (Ctrl+V) a tab-separated list: Command<TAB>param1, param2, ...");
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("auto_cmds")
                    .max_height(220.0)
                    .show(ui, |ui| {
                        egui::Grid::new("auto_grid")
                            .striped(true)
                            .num_columns(3)
                            .show(ui, |ui| {
                                ui.strong("");
                                ui.strong("Command");
                                ui.strong("Parameters");
                                ui.end_row();
                                for (i, (c, p)) in self.list.iter().enumerate() {
                                    let done = self.running && i < self.idx;
                                    let current = self.running && i == self.idx;
                                    let color = if done {
                                        Color32::GRAY
                                    } else {
                                        ui.visuals().text_color()
                                    };
                                    ui.label(if current { "▶" } else { "" });
                                    ui.label(RichText::new(c).color(color));
                                    ui.label(RichText::new(p.join(", ")).color(color));
                                    ui.end_row();
                                }
                            });
                    });
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("auto_msgs")
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for (m, err) in &self.messages {
                            if *err {
                                ui.label(RichText::new(m).color(Color32::RED).strong());
                            } else {
                                ui.label(m);
                            }
                        }
                    });
            });
        self.open = open;
        stop_pressed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_tab_separated() {
        let l = Automator::parse("Reset\t\nConfigOutput\t1, True, Noise, 0.5, 10\nStart\t5\n\n")
            .unwrap();
        assert_eq!(l.len(), 3);
        assert_eq!(l[1].1, vec!["1", "True", "Noise", "0.5", "10"]);
        assert!(l[0].1.is_empty());
        assert!(Automator::parse("a\tb\tc").is_err());
    }

    #[test]
    fn sequence_state_machine() {
        let mut a = Automator::new();
        a.list = vec![
            (s("Print"), vec![s("x")]),
            (s("Start"), vec![s("1")]),
            (s("Print"), vec![s("y")]),
        ];
        a.start();
        let c = a.next_action().unwrap();
        assert_eq!(c.0, "Print");
        a.complete(ActionOutcome::Resume);
        let c = a.next_action().unwrap();
        assert_eq!(c.0, "Start");
        a.complete(ActionOutcome::Wait);
        assert!(a.next_action().is_none());
        a.resume();
        assert_eq!(a.next_action().unwrap().1, vec!["y"]);
        a.complete(ActionOutcome::Resume);
        assert!(a.next_action().is_none());
        assert!(!a.running);
    }
}
