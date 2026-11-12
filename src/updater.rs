//! Firmware updater (port of `updater.py`): downloads the latest firmware
//! release, validates its manifest and SHA-256 checksums and flashes it with
//! the `espflash` library (replacing the bundled esptool of the Python version).

use std::io::{Cursor, Read};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use crossbeam_channel::{Receiver, Sender, unbounded};
use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::dataman::Shared;

pub const SOFTWARE_RELEASES_URL: &str = "https://github.com/eduardobatista/ActVibSoftware/releases";
pub const FIRMWARE_RELEASE_API: &str =
    "https://api.github.com/repos/eduardobatista/ActVibFirmware/releases/latest";
pub const FIRMWARE_FALLBACK_URL: &str =
    "https://github.com/eduardobatista/ActVibFirmware/archive/refs/heads/beta.zip";
const LEGACY_FIRMWARE_FILES: [(&str, &str); 4] = [
    ("0x1000", "ActVibFirmware.ino.bootloader.bin"),
    ("0x8000", "ActVibFirmware.ino.partitions.bin"),
    ("0xe000", "boot_app0.bin"),
    ("0x10000", "ActVibFirmware.ino.bin"),
];
const MAX_ARCHIVE: u64 = 100 * 1024 * 1024;
const MAX_MANIFEST: u64 = 1024 * 1024;
const MAX_FILE: u64 = 32 * 1024 * 1024;
const USER_AGENT: &str = "ActVibSoftware";

#[derive(Clone, Debug, PartialEq)]
pub struct FlashPlan {
    pub chip: String,
    pub baud: u32,
    pub before: String,
    pub after: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FileSpec {
    pub offset: u32,
    pub name: String,
    pub sha256: Option<String>,
}

pub struct FirmwareImage {
    pub offset: u32,
    pub name: String,
    pub data: Vec<u8>,
}

fn basename(path: &str) -> String {
    path.replace('\\', "/")
        .rsplit('/')
        .next()
        .unwrap_or("")
        .to_string()
}

fn parse_offset(v: &Value) -> Result<u32> {
    let s = match v {
        Value::String(s) => s.clone(),
        Value::Number(n) => n.to_string(),
        _ => bail!("Invalid firmware file entry"),
    };
    let s = s.trim();
    let r = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(h, 16)
    } else if let Some(o) = s.strip_prefix("0o") {
        u32::from_str_radix(o, 8)
    } else if let Some(b) = s.strip_prefix("0b") {
        u32::from_str_radix(b, 2)
    } else {
        s.parse::<u32>()
    };
    r.map_err(|_| anyhow!("Invalid firmware file entry"))
}

/// Validates a firmware `manifest.json` (schema version 1).
pub fn validate_manifest(manifest: &Value) -> Result<(FlashPlan, Vec<FileSpec>)> {
    let obj = manifest
        .as_object()
        .ok_or_else(|| anyhow!("Invalid firmware manifest"))?;
    if obj.get("schema_version").and_then(Value::as_i64) != Some(1) {
        bail!("Unsupported firmware manifest schema");
    }
    if obj.get("chip").and_then(Value::as_str) != Some("esp32") {
        bail!("The firmware manifest does not target ESP32");
    }
    let baud = match obj.get("baud") {
        None => 460_800,
        Some(v) => v
            .as_i64()
            .or_else(|| v.as_str().and_then(|s| s.parse().ok()))
            .ok_or_else(|| anyhow!("Invalid firmware baud rate"))?,
    };
    if !(9600..=3_000_000).contains(&baud) {
        bail!("Invalid firmware baud rate");
    }
    let files = obj
        .get("files")
        .and_then(Value::as_array)
        .filter(|f| !f.is_empty());
    let files = files.ok_or_else(|| anyhow!("The firmware manifest contains no files"))?;
    let mut out = Vec::new();
    let mut names = std::collections::HashSet::new();
    let mut offsets = std::collections::HashSet::new();
    for item in files {
        let offset = parse_offset(
            item.get("offset")
                .ok_or_else(|| anyhow!("Invalid firmware file entry"))?,
        )?;
        let name = basename(
            item.get("name")
                .and_then(Value::as_str)
                .ok_or_else(|| anyhow!("Invalid firmware file entry"))?,
        );
        let sha = item
            .get("sha256")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow!("Invalid firmware file entry"))?;
        if name.is_empty() || name == "." || name == ".." || offset >= 0x100_0000 {
            bail!("Invalid firmware file entry");
        }
        if sha.len() != 64 || !sha.chars().all(|c| c.is_ascii_hexdigit()) {
            bail!("Invalid checksum for {name}");
        }
        if !names.insert(name.clone()) || !offsets.insert(offset) {
            bail!("Duplicate file or offset in firmware manifest");
        }
        out.push(FileSpec {
            offset,
            name,
            sha256: Some(sha.to_string()),
        });
    }
    let before = obj
        .get("before")
        .and_then(Value::as_str)
        .unwrap_or("default-reset")
        .to_string();
    let after = obj
        .get("after")
        .and_then(Value::as_str)
        .unwrap_or("hard-reset")
        .to_string();
    if !["default-reset", "usb-reset", "no-reset", "no-reset-no-sync"].contains(&before.as_str()) {
        bail!("Invalid reset mode in firmware manifest");
    }
    if ![
        "hard-reset",
        "soft-reset",
        "no-reset",
        "no-reset-stub",
        "watchdog-reset",
    ]
    .contains(&after.as_str())
    {
        bail!("Invalid reset mode in firmware manifest");
    }
    Ok((
        FlashPlan {
            chip: "esp32".into(),
            baud: baud as u32,
            before,
            after,
        },
        out,
    ))
}

/// Extracts and verifies the firmware images from a release archive.
/// Returns the plan, the images and whether this is a legacy (manifest-less) archive.
pub fn prepare_firmware(archive: &[u8]) -> Result<(FlashPlan, Vec<FirmwareImage>, bool)> {
    let mut zip = zip::ZipArchive::new(Cursor::new(archive)).context("Invalid firmware archive")?;
    let mut members: Vec<(usize, String, u64)> = Vec::new();
    for i in 0..zip.len() {
        let f = zip.by_index(i)?;
        if !f.is_dir() {
            members.push((i, f.name()?.to_string(), f.size()));
        }
    }
    let manifests: Vec<&(usize, String, u64)> = members
        .iter()
        .filter(|m| basename(&m.1) == "manifest.json")
        .collect();
    if manifests.len() > 1 {
        bail!("Firmware archive contains multiple manifests");
    }
    let (plan, specs, legacy) = if let Some(m) = manifests.first() {
        if m.2 > MAX_MANIFEST {
            bail!("Firmware manifest is larger than 1 MB");
        }
        let mut s = String::new();
        zip.by_index(m.0)?
            .take(MAX_MANIFEST)
            .read_to_string(&mut s)?;
        let v: Value = serde_json::from_str(&s).context("Invalid firmware manifest")?;
        let (plan, specs) = validate_manifest(&v)?;
        (plan, specs, false)
    } else {
        let plan = FlashPlan {
            chip: "esp32".into(),
            baud: 460_800,
            before: "default-reset".into(),
            after: "hard-reset".into(),
        };
        let specs = LEGACY_FIRMWARE_FILES
            .iter()
            .map(|(o, n)| FileSpec {
                offset: parse_offset(&Value::String(o.to_string())).unwrap(),
                name: n.to_string(),
                sha256: None,
            })
            .collect();
        (plan, specs, true)
    };
    let mut images = Vec::new();
    for spec in specs {
        let candidates: Vec<&(usize, String, u64)> = members
            .iter()
            .filter(|m| {
                basename(&m.1) == spec.name
                    && (!legacy || m.1.replace('\\', "/").split('/').any(|p| p == "update"))
            })
            .collect();
        if candidates.is_empty() {
            bail!("Firmware file not found: {}", spec.name);
        }
        if candidates.len() > 1 {
            bail!("Firmware archive contains duplicate files: {}", spec.name);
        }
        let m = candidates[0];
        if m.2 > MAX_FILE {
            bail!("Firmware file is larger than 32 MB: {}", spec.name);
        }
        let mut data = Vec::with_capacity(m.2 as usize);
        zip.by_index(m.0)?
            .take(MAX_FILE + 1)
            .read_to_end(&mut data)?;
        if let Some(expected) = &spec.sha256 {
            let actual = hex::encode(Sha256::digest(&data));
            if !actual.eq_ignore_ascii_case(expected) {
                bail!("Checksum mismatch for {}", spec.name);
            }
        }
        images.push(FirmwareImage {
            offset: spec.offset,
            name: spec.name,
            data,
        });
    }
    Ok((plan, images, legacy))
}

fn agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into()
}

/// Finds the firmware archive URL from the latest GitHub release.
pub fn firmware_archive_url() -> (String, String) {
    let r = (|| -> Result<(String, String)> {
        let mut resp = agent()
            .get(FIRMWARE_RELEASE_API)
            .header("Accept", "application/vnd.github+json")
            .header("User-Agent", USER_AGENT)
            .call()?;
        let release: Value = serde_json::from_str(&resp.body_mut().read_to_string()?)?;
        let tag = release
            .get("tag_name")
            .and_then(Value::as_str)
            .unwrap_or("latest release")
            .to_string();
        if let Some(assets) = release.get("assets").and_then(Value::as_array) {
            for a in assets {
                let name = a
                    .get("name")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_lowercase();
                if name.ends_with(".zip")
                    && name.contains("firmware")
                    && let Some(url) = a.get("browser_download_url").and_then(Value::as_str)
                {
                    return Ok((url.to_string(), tag));
                }
            }
        }
        if let Some(url) = release.get("zipball_url").and_then(Value::as_str) {
            return Ok((url.to_string(), tag));
        }
        bail!("no assets")
    })();
    r.unwrap_or_else(|_| {
        (
            FIRMWARE_FALLBACK_URL.to_string(),
            "beta branch fallback".to_string(),
        )
    })
}

pub fn download(url: &str) -> Result<Vec<u8>> {
    let mut resp = agent().get(url).header("User-Agent", USER_AGENT).call()?;
    if let Some(len) = resp
        .headers()
        .get("Content-Length")
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.parse::<u64>().ok())
        && len > MAX_ARCHIVE
    {
        bail!("Firmware archive is larger than 100 MB");
    }
    let mut data = Vec::new();
    resp.body_mut()
        .as_reader()
        .take(MAX_ARCHIVE + 1)
        .read_to_end(&mut data)?;
    if data.len() as u64 > MAX_ARCHIVE {
        bail!("Firmware archive is larger than 100 MB");
    }
    Ok(data)
}

struct Progress<'a> {
    tx: &'a Sender<String>,
    total: usize,
    last_pct: usize,
}

impl espflash::target::ProgressCallbacks for Progress<'_> {
    fn init(&mut self, addr: u32, total: usize) {
        self.total = total.max(1);
        self.last_pct = 0;
        let _ = self.tx.send(format!("Writing at 0x{addr:08x}... "));
    }
    fn update(&mut self, current: usize) {
        let pct = current * 100 / self.total;
        if pct >= self.last_pct + 10 {
            self.last_pct = pct - pct % 10;
            let _ = self.tx.send(format!("{}% ", self.last_pct));
        }
    }
    fn verifying(&mut self) {
        let _ = self.tx.send("verifying... ".into());
    }
    fn finish(&mut self, skipped: bool) {
        let _ = self.tx.send(if skipped {
            "skipped (already up to date).\n".into()
        } else {
            "done.\n".into()
        });
    }
}

/// Flashes the images using espflash.
pub fn flash(
    port: &str,
    plan: &FlashPlan,
    images: &[FirmwareImage],
    tx: &Sender<String>,
) -> Result<()> {
    use espflash::connection::{Connection, ResetAfterOperation, ResetBeforeOperation};
    use espflash::flasher::Flasher;
    use espflash::image_format::Segment;
    use espflash::target::Chip;

    let before = match plan.before.as_str() {
        "usb-reset" => ResetBeforeOperation::UsbReset,
        "no-reset" => ResetBeforeOperation::NoReset,
        "no-reset-no-sync" => ResetBeforeOperation::NoResetNoSync,
        _ => ResetBeforeOperation::DefaultReset,
    };
    let after = match plan.after.as_str() {
        "no-reset" => ResetAfterOperation::NoReset,
        "no-reset-stub" => ResetAfterOperation::NoResetNoStub,
        "watchdog-reset" => ResetAfterOperation::WatchdogReset,
        _ => ResetAfterOperation::HardReset,
    };
    let info = serialport::available_ports()?
        .into_iter()
        .find(|p| p.port_name == port)
        .ok_or_else(|| anyhow!("Serial port {port} not found"))?;
    let usb = match info.port_type {
        serialport::SerialPortType::UsbPort(u) => u,
        _ => bail!("Firmware update requires a USB serial port ({port} is not USB)"),
    };
    let serial = serialport::new(port, 115_200)
        .flow_control(serialport::FlowControl::None)
        .open_native()
        .with_context(|| format!("Could not open {port}"))?;
    let conn = Connection::new(serial, usb, after, before, 115_200);
    let _ = tx.send(format!("Connecting to {} on {port}...\n", plan.chip));
    let mut flasher = Flasher::connect(conn, true, true, false, Some(Chip::Esp32), Some(plan.baud))
        .map_err(|e| anyhow!("Could not connect to the ESP32 bootloader: {e}"))?;
    let _ = tx.send(format!("Connected (baud {}).\n", plan.baud));
    for img in images {
        let _ = tx.send(format!(
            "  0x{:06x}: {} ({} bytes)\n",
            img.offset,
            img.name,
            img.data.len()
        ));
        if img.offset == 0x1000
            && img.data.len() > 3
            && img.data[0] == 0xE9
            && img.data[2..4] != [0x02, 0x2F]
        {
            let _ = tx.send(
                "Warning: bootloader header is not dio/80m/4MB; flashing it unchanged.\n".into(),
            );
        }
    }
    let segments: Vec<Segment> = images
        .iter()
        .map(|i| Segment::new(i.offset, &i.data))
        .collect();
    let mut progress = Progress {
        tx,
        total: 1,
        last_pct: 0,
    };
    flasher
        .write_bins_to_flash(&segments, &mut progress)
        .map_err(|e| anyhow!("Flashing failed: {e}"))?;
    let _ = tx.send("Resetting the device...\n".into());
    flasher
        .connection()
        .reset_after(true, Chip::Esp32)
        .map_err(|e| anyhow!("Reset failed: {e}"))?;
    Ok(())
}

fn run_firmware_update(shared: &Shared, port: &str, tx: &Sender<String>) -> Result<()> {
    if shared.link.lock().unwrap().is_some() {
        let _ = tx.send("Closing the serial port...\n".into());
        shared.close_link();
        std::thread::sleep(Duration::from_millis(500));
    }
    let (url, release) = firmware_archive_url();
    let _ = tx.send(format!("Downloading firmware ({release})...\n"));
    let archive = download(&url)?;
    let (plan, images, legacy) = prepare_firmware(&archive)?;
    if legacy {
        let _ = tx.send(
            "Warning: this legacy firmware package has no manifest or published checksums.\n"
                .into(),
        );
    }
    let _ = tx.send("Firmware files validated. Flashing...\n".into());
    flash(port, &plan, &images, tx)
}

pub struct UpdaterDialog {
    pub open: bool,
    pub port: Option<String>,
    messages: String,
    rx: Option<Receiver<String>>,
    pub running: bool,
}

impl UpdaterDialog {
    pub fn new(port: Option<String>) -> Self {
        let mut s = Self {
            open: true,
            port,
            messages: String::new(),
            rx: None,
            running: false,
        };
        if s.port.is_none() {
            s.messages
                .push_str("Select a serial port before updating the firmware.\n");
        }
        s
    }

    fn start_firmware(&mut self, ctx: &egui::Context, shared: &Arc<Shared>) {
        self.messages.clear();
        let Some(port) = self.port.clone() else {
            self.messages
                .push_str("Select a serial port before updating the firmware.\n");
            return;
        };
        if self.running {
            return;
        }
        self.running = true;
        let (tx, rx) = unbounded();
        self.rx = Some(rx);
        let sh = shared.clone();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let r = run_firmware_update(&sh, &port, &tx);
            let _ = tx.send(match r {
                Ok(()) => "\nFirmware update completed successfully.\n".into(),
                Err(e) => format!("\nFirmware update failed: {e:#}\n"),
            });
            let _ = tx.send("\u{0}DONE".into());
            ctx.request_repaint();
        });
    }

    fn software_instructions(&mut self) {
        self.messages = format!(
            "Installed version: {}\n\n\
             To update ActVib, close it and download the latest release from the Releases page below, \
             replacing the current executable.\n\n\
             If you built ActVib from source, update the checkout and rebuild:\n\n\
             git pull\ncargo build --release\n\n\
             Releases: {SOFTWARE_RELEASES_URL}\n",
            env!("CARGO_PKG_VERSION")
        );
    }

    pub fn show(&mut self, ctx: &egui::Context, shared: &Arc<Shared>, acquisition_running: bool) {
        if let Some(rx) = &self.rx {
            while let Ok(m) = rx.try_recv() {
                if m == "\u{0}DONE" {
                    self.running = false;
                } else {
                    self.messages.push_str(&m);
                }
            }
            if self.running {
                ctx.request_repaint_after(Duration::from_millis(100));
            }
        }
        let mut open = self.open;
        egui::Window::new("Firmware/Software Updater")
            .open(&mut open)
            .default_size([640.0, 420.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let fw_en = self.port.is_some() && !self.running && !acquisition_running;
                    if ui
                        .add_enabled(fw_en, egui::Button::new("Start Firmware Update..."))
                        .clicked()
                    {
                        self.start_firmware(ctx, shared);
                    }
                    if ui
                        .add_enabled(
                            !self.running,
                            egui::Button::new("Show Software Update Instructions"),
                        )
                        .clicked()
                    {
                        self.software_instructions();
                    }
                });
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        ui.add(
                            egui::TextEdit::multiline(&mut self.messages.as_str())
                                .desired_width(f32::INFINITY)
                                .desired_rows(18)
                                .font(egui::TextStyle::Monospace),
                        );
                    });
            });
        // Never close the window while flashing.
        self.open = open || self.running;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn make_zip(files: &[(&str, &[u8])]) -> Vec<u8> {
        let mut buf = Cursor::new(Vec::new());
        {
            let mut w = zip::ZipWriter::new(&mut buf);
            for (name, data) in files {
                w.start_file(*name, zip::write::SimpleFileOptions::default())
                    .unwrap();
                w.write_all(data).unwrap();
            }
            w.finish().unwrap();
        }
        buf.into_inner()
    }

    fn sha(d: &[u8]) -> String {
        hex::encode(Sha256::digest(d))
    }

    #[test]
    fn manifest_archive_is_validated() {
        let a = b"boot".as_slice();
        let b = b"app".as_slice();
        let manifest = format!(
            r#"{{"schema_version":1,"chip":"esp32","baud":460800,"files":[
                {{"offset":"0x1000","name":"boot.bin","sha256":"{}"}},
                {{"offset":"0x10000","name":"sub/app.bin","sha256":"{}"}}]}}"#,
            sha(a),
            sha(b)
        );
        let z = make_zip(&[
            ("manifest.json", manifest.as_bytes()),
            ("boot.bin", a),
            ("x/app.bin", b),
        ]);
        let (plan, imgs, legacy) = prepare_firmware(&z).unwrap();
        assert!(!legacy);
        assert_eq!(plan.baud, 460_800);
        assert_eq!(imgs.len(), 2);
        assert_eq!(imgs[1].offset, 0x10000);
        assert_eq!(imgs[1].data, b"app");
    }

    #[test]
    fn checksum_mismatch_is_rejected() {
        let manifest = format!(
            r#"{{"schema_version":1,"chip":"esp32","files":[{{"offset":"0x1000","name":"boot.bin","sha256":"{}"}}]}}"#,
            "0".repeat(64)
        );
        let z = make_zip(&[
            ("manifest.json", manifest.as_bytes()),
            ("boot.bin", b"boot"),
        ]);
        let err = prepare_firmware(&z).err().unwrap().to_string();
        assert!(err.contains("Checksum mismatch"), "{err}");
    }

    #[test]
    fn invalid_manifests() {
        let bad = |s: &str| validate_manifest(&serde_json::from_str(s).unwrap()).is_err();
        assert!(bad(r#"{"schema_version":2,"chip":"esp32","files":[]}"#));
        assert!(bad(r#"{"schema_version":1,"chip":"esp32s3","files":[]}"#));
        assert!(bad(r#"{"schema_version":1,"chip":"esp32","files":[]}"#));
        assert!(bad(
            r#"{"schema_version":1,"chip":"esp32","baud":1,"files":[{"offset":"0x0","name":"a","sha256":"x"}]}"#
        ));
        let h = "a".repeat(64);
        assert!(bad(&format!(
            r#"{{"schema_version":1,"chip":"esp32","files":[{{"offset":"0x0","name":"a","sha256":"{h}"}},{{"offset":"0x0","name":"b","sha256":"{h}"}}]}}"#
        )));
        assert!(bad(&format!(
            r#"{{"schema_version":1,"chip":"esp32","before":"bogus","files":[{{"offset":"0x0","name":"a","sha256":"{h}"}}]}}"#
        )));
    }

    #[test]
    fn legacy_archive_requires_update_dir() {
        let files: Vec<(String, &[u8])> = LEGACY_FIRMWARE_FILES
            .iter()
            .map(|(_, n)| (format!("ActVibFirmware-beta/update/{n}"), b"x".as_slice()))
            .collect();
        let refs: Vec<(&str, &[u8])> = files.iter().map(|(n, d)| (n.as_str(), *d)).collect();
        let (_, imgs, legacy) = prepare_firmware(&make_zip(&refs)).unwrap();
        assert!(legacy);
        assert_eq!(imgs.len(), 4);
        assert_eq!(imgs[2].offset, 0xe000);
    }

    #[test]
    fn real_release_archive_if_present() {
        let p = std::path::Path::new(env!("HOME"))
            .join("ActVibFirmware/dist/ActVibFirmware-v0.9.0.zip");
        if let Ok(data) = std::fs::read(p) {
            let (plan, imgs, legacy) = prepare_firmware(&data).unwrap();
            assert!(!legacy);
            assert_eq!(plan.chip, "esp32");
            assert_eq!(imgs.len(), 4);
            assert_eq!(&imgs[0].data[..4], &[0xE9, 0x03, 0x02, 0x2F]);
        }
    }
}
