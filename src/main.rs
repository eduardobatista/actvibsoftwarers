#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
//! ActVib: monitoring, data acquisition, path modeling and active control of
//! laboratory vibration experiments using an ESP32-based controller.

mod app;
mod automator;
#[allow(dead_code)] // Developer debug-mode simulator; kept complete.
mod beam;
mod dataman;
mod dialogs;
mod driver;
mod dsp;
mod figures;
mod panels;
mod settings;
mod table;
mod updater;

fn main() -> eframe::Result<()> {
    if std::env::args().any(|a| a == "--version") {
        println!("actvib {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    let mut viewport = egui::ViewportBuilder::default()
        .with_title("ActVib")
        .with_app_id("actvib")
        .with_inner_size([1400.0, 900.0])
        .with_min_inner_size([900.0, 600.0])
        .with_drag_and_drop(true);
    if let Ok(icon) = eframe::icon_data::from_png_bytes(include_bytes!("../assets/actvib.png")) {
        viewport = viewport.with_icon(icon);
    }
    let options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };
    eframe::run_native(
        "ActVib",
        options,
        Box::new(|cc| Ok(Box::new(app::ActVibApp::new(cc)))),
    )
}
