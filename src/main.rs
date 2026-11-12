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

/// Shows a native error dialog (the Windows build has no console).
fn report_fatal(msg: &str) {
    eprintln!("[actvib] {msg}");
    rfd::MessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title("ActVib - error")
        .set_description(msg)
        .show();
}

fn main() {
    if std::env::args().any(|a| a == "--version") {
        println!("actvib {}", env!("CARGO_PKG_VERSION"));
        return;
    }
    std::panic::set_hook(Box::new(|info| {
        report_fatal(&format!("ActVib {} crashed:\n\n{info}", env!("CARGO_PKG_VERSION")));
    }));
    let mut viewport = egui::ViewportBuilder::default()
        .with_title("ActVib")
        .with_app_id("actvib")
        .with_inner_size([1400.0, 900.0])
        .with_min_inner_size([900.0, 600.0])
        .with_drag_and_drop(true);
    if let Ok(icon) = eframe::icon_data::from_png_bytes(include_bytes!("../assets/actvib.png")) {
        viewport = viewport.with_icon(icon);
    }
    // Renderer: OpenGL (glow) first; if it cannot start, relaunch with wgpu
    // (DirectX 12 / Vulkan / Metal). A winit event loop can only be created
    // once per process, so the fallback runs in a child process.
    // ACTVIB_RENDERER=glow|wgpu forces one backend (no fallback).
    let forced = std::env::var("ACTVIB_RENDERER").ok();
    let renderer = match forced.as_deref() {
        Some("wgpu") => eframe::Renderer::Wgpu,
        _ => eframe::Renderer::Glow,
    };
    let options = eframe::NativeOptions {
        viewport,
        renderer,
        ..Default::default()
    };
    let Err(e) = eframe::run_native(
        "ActVib",
        options,
        Box::new(|cc| Ok(Box::new(app::ActVibApp::new(cc)))),
    ) else {
        return;
    };
    if forced.is_none() {
        eprintln!("[actvib] OpenGL renderer failed ({e}); retrying with wgpu");
        if let Ok(exe) = std::env::current_exe() {
            match std::process::Command::new(exe)
                .args(std::env::args_os().skip(1))
                .env("ACTVIB_RENDERER", "wgpu")
                .status()
            {
                Ok(status) => std::process::exit(status.code().unwrap_or(1)),
                Err(err) => eprintln!("[actvib] could not relaunch: {err}"),
            }
        }
    }
    report_fatal(&format!(
        "ActVib {} could not start the graphics renderer:\n\n{e}\n\n\
         Make sure the graphics card driver is installed (OpenGL 2.0+, \
         DirectX 12 or Vulkan is required).",
        env!("CARGO_PKG_VERSION")
    ));
    std::process::exit(1);
}
