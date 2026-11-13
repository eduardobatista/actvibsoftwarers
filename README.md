# ActVib (Rust)

## ⬇️ Download — versão 0.1.2

| Sistema | Baixar |
|---|---|
| **Windows 10/11** (64 bits) | **[ActVib-0.1.2-windows-x64.exe](https://github.com/eduardobatista/actvibsoftwarers/releases/latest/download/ActVib-0.1.2-windows-x64.exe)** |
| **Linux** (Ubuntu 22.04+, Debian 12+, Fedora, Arch) | **[ActVib-0.1.2-linux-x86_64](https://github.com/eduardobatista/actvibsoftwarers/releases/latest/download/ActVib-0.1.2-linux-x86_64)** |

Arquivo único, sem instalação. Instruções: [LEIAME.txt](https://github.com/eduardobatista/actvibsoftwarers/releases/latest/download/LEIAME.txt) ·
Checksums: [SHA256SUMS.txt](https://github.com/eduardobatista/actvibsoftwarers/releases/latest/download/SHA256SUMS.txt) ·
[Página da release](https://github.com/eduardobatista/actvibsoftwarers/releases/latest)

- **Windows:** se o SmartScreen avisar, clique em *Mais informações → Executar assim mesmo*.
- **Linux:** `chmod +x ActVib-0.1.2-linux-x86_64 && ./ActVib-0.1.2-linux-x86_64`

---

Rust port of [ActVibSoftware](https://github.com/eduardobatista/ActVibSoftware):
a desktop application for monitoring, data acquisition, path modeling and
active control of laboratory vibration experiments using an ESP32-based
controller running [ActVibFirmware](https://github.com/eduardobatista/ActVibFirmware).

It speaks the same serial protocol as the Python version and reads/writes the
same Feather and CSV files, so recordings and `metacontrol.feather` files are
interchangeable between both versions and with pandas.

## Building

Requires a recent stable Rust toolchain (edition 2024).

```bash
cargo build --release
./target/release/actvib
```

On Linux, building needs the `libudev` development package (`libudev-dev` on
Debian/Ubuntu, `systemd-libs` on Arch). If the serial port is visible but
cannot be opened, add your user to the group that owns the device
(`dialout` on Debian/Ubuntu, `uucp` on Arch) and log in again.

Run the tests with:

```bash
cargo test
```

## Features

Same functionality as the Python application:

- **Measurement mode**: up to three IMUs (MPU6050 / LSM6DS3, I2C or VSPI),
  ADS1115/ADS1015 ADC, four signal generators (noise, harmonic, chirp, step,
  square) with real-time plots.
- **Control mode**: FxNLMS / CVA-FxNLMS active control and path-modeling
  recordings, with algorithm start time, step size and penalization.
- **Saving**: Feather or tab-separated CSV with the experiment log column;
  "Save and Update Metadata in Workdir" plus the Workdir Manager.
- **Path modeling**: NLMS identification of secondary and feedback paths from
  the current recording or a file, FFT view, SVD compression (with singular
  value plots), save/load paths, upload and record in flash, read back paths
  from the device.
- **Data Viewer** for control recordings (open, drag & drop or data in memory).
- **Additional system setup**: output predistortion polynomials and sensor
  fusion weights written to the device.
- **Automator** for scripted experiments (paste a tab-separated command list
  with Ctrl+V or load it from a text file).
- **Firmware updater**: downloads the latest firmware release, validates its
  `manifest.json` and SHA-256 checksums and flashes it with the built-in
  [espflash](https://github.com/esp-rs/espflash) library (no Python/esptool
  needed).

Keyboard shortcuts: `Space` start/stop (or turns the algorithm on during a
control run), `Ctrl+S` save, `Ctrl+Shift+S` save to workdir and reset,
`Alt+P` focus the step-size field.

## Differences from the Python version

- GUI built with [egui](https://github.com/emilk/egui); plots refresh
  continuously instead of once per second (device updates such as generator
  changes and algorithm activation still happen at the same 1 s cadence).
- Settings are stored as JSON in the user configuration directory
  (e.g. `~/.config/actvib/settings.json`) instead of `QSettings`, so settings
  from the Python version are not imported.
- The automator's **Load...** button opens a command-list text file (same
  format as the clipboard paste).
- A few Python bugs were fixed rather than reproduced: "Save and Update
  Metadata in Workdir" now works in control mode (and saves the data file in
  measurement mode), the IMU filter selection is no longer reset on startup,
  notes are not duplicated in the log on repeated saves, the automator's
  `Wait`/`Waiting` commands both work, and `Stop` no longer starts an
  acquisition when nothing is running.
- Developer debug mode (host-simulated cantilever beam) can be enabled with
  `ACTVIB_DEBUG_MODE=1`; it is not reachable from the UI, as in the original.

## Source layout

| Module | Python origin | Purpose |
| --- | --- | --- |
| `driver.rs` | `driverhardware.py` | Device configuration and serial protocol |
| `dataman.rs` | `dataman.py` | Acquisition thread, data buffers, export |
| `panels.rs` | `panels.py` + `.ui` | IMU, generator, control and ADC panels |
| `figures.rs` | `figures.py`, `BaseFigQtGraph.py` | Real-time plots |
| `dialogs.rs` | `dialogs.py` | Path modeling, data viewer, additional setup, workdir manager |
| `automator.rs` | `automator.py` | Experiment automation |
| `updater.rs` | `updater.py` | Firmware download, validation and flashing |
| `dsp.rs` | `DSP.py`, `dialogs.py` | FFT, NLMS, SVD path compression |
| `beam.rs` | `CantileverBeam.py` | Beam simulator for debug mode |
| `table.rs` | pandas | Feather (Arrow IPC) and CSV I/O |
| `app.rs` | `mainwindow.py` | Main window |

## License

MIT — see [LICENSE](LICENSE).
