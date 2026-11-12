//! Hardware driver: device configuration state and the serial protocol spoken
//! with the ActVib ESP32 firmware (port of `driverhardware.py`).

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow, bail};

use crate::beam::CantileverBeam;

/// Default serial timeout, equivalent to pyserial's `timeout=1`.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);
pub const DEFAULT_BAUD: u32 = 500_000;
pub const BAUD_RATES: [u32; 4] = [115_200, 500_000, 921_600, 1_000_000];

/// Minimal byte transport, implemented for real serial ports and for test mocks.
pub trait Transport: Send {
    fn write_all(&mut self, data: &[u8]) -> std::io::Result<()>;
    /// Reads whatever is available (blocking up to an implementation-defined
    /// short timeout). Returns `Ok(0)` when nothing arrived.
    fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
    fn flush(&mut self) -> std::io::Result<()>;
    fn clear_input(&mut self) -> std::io::Result<()>;
    fn clear_output(&mut self) -> std::io::Result<()>;
    fn set_baud_rate(&mut self, baud: u32) -> std::io::Result<()>;
    fn set_dtr(&mut self, level: bool) -> std::io::Result<()>;
    fn set_rts(&mut self, level: bool) -> std::io::Result<()>;
}

impl Transport for Box<dyn serialport::SerialPort> {
    fn write_all(&mut self, data: &[u8]) -> std::io::Result<()> {
        Write::write_all(self, data)
    }
    fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match Read::read(self, buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Write::flush(self)
    }
    fn clear_input(&mut self) -> std::io::Result<()> {
        self.clear(serialport::ClearBuffer::Input)
            .map_err(std::io::Error::other)
    }
    fn clear_output(&mut self) -> std::io::Result<()> {
        self.clear(serialport::ClearBuffer::Output)
            .map_err(std::io::Error::other)
    }
    fn set_baud_rate(&mut self, baud: u32) -> std::io::Result<()> {
        serialport::SerialPort::set_baud_rate(self.as_mut(), baud).map_err(std::io::Error::other)
    }
    fn set_dtr(&mut self, level: bool) -> std::io::Result<()> {
        self.write_data_terminal_ready(level)
            .map_err(std::io::Error::other)
    }
    fn set_rts(&mut self, level: bool) -> std::io::Result<()> {
        self.write_request_to_send(level)
            .map_err(std::io::Error::other)
    }
}

/// A buffered serial link with pyserial-like `read(n)` semantics: block until
/// `n` bytes arrive or the timeout expires, returning what was received.
pub struct Link {
    io: Box<dyn Transport>,
    rx: Vec<u8>,
    rx_pos: usize,
    pub timeout: Duration,
    pub baud: u32,
}

impl Link {
    pub fn new(io: Box<dyn Transport>, baud: u32) -> Self {
        Self {
            io,
            rx: Vec::with_capacity(8192),
            rx_pos: 0,
            timeout: DEFAULT_TIMEOUT,
            baud,
        }
    }

    /// Opens a real serial port (8N1, no flow control).
    pub fn open_serial(port: &str, baud: u32) -> Result<Self> {
        let sp = serialport::new(port, baud)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| anyhow!("Could not open port {port}: {e}"))?;
        Ok(Self::new(Box::new(sp), baud))
    }

    fn buffered(&self) -> usize {
        self.rx.len() - self.rx_pos
    }

    fn fill(&mut self) -> Result<usize> {
        if self.rx_pos > 0 && self.rx_pos == self.rx.len() {
            self.rx.clear();
            self.rx_pos = 0;
        }
        let mut tmp = [0u8; 4096];
        let n = self.io.read_some(&mut tmp)?;
        self.rx.extend_from_slice(&tmp[..n]);
        Ok(n)
    }

    /// pyserial `read(n)`: returns up to `n` bytes (fewer on timeout).
    pub fn read(&mut self, n: usize) -> Result<Vec<u8>> {
        let deadline = Instant::now() + self.timeout;
        while self.buffered() < n {
            if self.fill()? == 0 && Instant::now() >= deadline {
                break;
            }
        }
        let take = n.min(self.buffered());
        let out = self.rx[self.rx_pos..self.rx_pos + take].to_vec();
        self.rx_pos += take;
        Ok(out)
    }

    /// Reads exactly `n` bytes or fails.
    pub fn read_exact(&mut self, n: usize, what: &str) -> Result<Vec<u8>> {
        let v = self.read(n)?;
        if v.len() < n {
            bail!(
                "Timeout reading {what} ({} of {n} bytes received).",
                v.len()
            );
        }
        Ok(v)
    }

    pub fn read_byte(&mut self) -> Result<Option<u8>> {
        Ok(self.read(1)?.first().copied())
    }

    /// pyserial `readline()`: reads up to and including `\n` (or timeout).
    pub fn readline(&mut self) -> Result<String> {
        let deadline = Instant::now() + self.timeout;
        let mut line = Vec::new();
        loop {
            while self.rx_pos < self.rx.len() {
                let b = self.rx[self.rx_pos];
                self.rx_pos += 1;
                line.push(b);
                if b == b'\n' {
                    return Ok(String::from_utf8_lossy(&line).into_owned());
                }
            }
            if self.fill()? == 0 && Instant::now() >= deadline {
                return Ok(String::from_utf8_lossy(&line).into_owned());
            }
        }
    }

    pub fn write(&mut self, data: &[u8]) -> Result<()> {
        self.io.write_all(data)?;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.io.flush()?;
        Ok(())
    }

    pub fn reset_input(&mut self) -> Result<()> {
        self.rx.clear();
        self.rx_pos = 0;
        self.io.clear_input()?;
        Ok(())
    }

    pub fn reset_output(&mut self) -> Result<()> {
        self.io.clear_output()?;
        Ok(())
    }

    pub fn reset_buffers(&mut self) -> Result<()> {
        self.reset_output()?;
        self.reset_input()
    }

    pub fn set_baud(&mut self, baud: u32) -> Result<()> {
        self.io.set_baud_rate(baud)?;
        self.baud = baud;
        Ok(())
    }

    /// Toggles RTS/DTR to force a device reboot, then drains the boot banner.
    pub fn reset_device(&mut self) -> Result<()> {
        self.io.set_rts(false)?;
        self.io.set_dtr(false)?;
        std::thread::sleep(Duration::from_millis(100));
        self.io.set_rts(true)?;
        self.io.set_dtr(true)?;
        std::thread::sleep(Duration::from_millis(200));
        let _ = self.read(2048)?;
        Ok(())
    }
}

/// One decoded acquisition packet.
#[derive(Clone, Debug, Default)]
pub struct Reading {
    pub acc: [[f64; 3]; 3],
    pub gyro: [[f64; 3]; 3],
    pub dacout: [f64; 4],
    pub adcin: [f64; 4],
    pub xref: f64,
    pub xerro: f64,
    pub calctime: [u16; 3],
    pub errorflag: u8,
}

/// Device configuration (everything in `driverhardware` except the port).
#[derive(Clone, Debug)]
pub struct DriverConfig {
    pub accscaler: [f64; 3],
    pub gyroscaler: [f64; 3],
    pub accrangeselection: [u8; 3],
    pub gyrorangeselection: [u8; 3],
    pub gentipo: [u8; 4],
    pub genamp: [f64; 4],
    pub genfreq: [f64; 4],
    pub dclevel: [u16; 4],
    pub iampscaler: [f64; 4],
    pub chirpconf: [[u8; 6]; 4],
    pub pwmduty: [u8; 4],
    pub imuconfigdata: [[u8; 3]; 3],
    pub imu_enable: [bool; 3],
    /// 0 = MPU6050, 1 = LSM6DS3
    pub imu_types: [u8; 3],
    pub gen_config_written: [bool; 4],
    pub adcconfig: [u8; 3],
    pub adcenablemap: [bool; 4],
    pub adcmultiplier: f64,
    pub adcsel: i8,
    pub control_mode: bool,
    pub task_is_control: bool,
    pub debug_mode: bool,
    pub control_channel: u8,
    pub perturb_channel: u8,
    pub ctrlalg: u8,
    pub algon: bool,
    pub ctrlmem: u16,
    pub ctrlmu: f32,
    pub ctrlfi: f32,
    pub refid: u8,
    pub erroid: u8,
    pub ctrltask: u8,
    pub refimuid: u8,
    pub errimuid: u8,
    pub algonchanged: bool,
    pub algontime: f64,
    pub predistenablemap: [bool; 4],
    pub predistcoefs: [Vec<f64>; 4],
    pub fusionweights: [f64; 2],
}

/// Generator parameters as produced by a generator panel.
#[derive(Clone, Debug, PartialEq)]
pub struct GeneratorParams {
    pub tipo: u8,
    pub amp: f64,
    pub freq: f64,
    pub dclevel: u16,
    /// [start time, delta i, end time, delta f, A2]
    pub chirpconf: Option<[f64; 5]>,
    pub pwmduty: f64,
}

/// Control configuration as produced by the control panel.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ControlParams {
    pub alg: u8,
    pub mem: u16,
    pub mu: f32,
    pub fi: f32,
    pub refimuid: u8,
    pub errimuid: u8,
    pub refid: u8,
    pub erroid: u8,
    pub ctrltask: u8,
}

impl Default for DriverConfig {
    fn default() -> Self {
        let dclevel = [1668u16, 1668, 104, 104];
        let mut cfg = Self {
            accscaler: [9.80665 * 2.0 / 32768.0; 3],
            gyroscaler: [250.0 / 32768.0; 3],
            accrangeselection: [0; 3],
            gyrorangeselection: [0; 3],
            gentipo: [0; 4],
            genamp: [0.0; 4],
            genfreq: [0.0; 4],
            dclevel,
            iampscaler: dclevel.map(|d| 1.0 / d as f64),
            chirpconf: [[0; 6]; 4],
            pwmduty: [0; 4],
            imuconfigdata: [[0; 3]; 3],
            imu_enable: [false; 3],
            imu_types: [0; 3],
            gen_config_written: [false; 4],
            adcconfig: [0; 3],
            adcenablemap: [false; 4],
            adcmultiplier: 0.0,
            adcsel: 0,
            control_mode: false,
            task_is_control: true,
            debug_mode: false,
            control_channel: 0,
            perturb_channel: 0,
            ctrlalg: 0,
            algon: false,
            ctrlmem: 100,
            ctrlmu: 0.1,
            ctrlfi: 1e-4,
            refid: 0,
            erroid: 0,
            ctrltask: 0,
            refimuid: 0,
            errimuid: 1,
            algonchanged: false,
            algontime: 0.0,
            predistenablemap: [false; 4],
            predistcoefs: std::array::from_fn(|_| vec![1.0, 0.0]),
            fusionweights: [0.5, 0.5],
        };
        for id in 0..4 {
            cfg.set_generator_config(
                id,
                &GeneratorParams {
                    tipo: 0,
                    amp: 0.0,
                    freq: 10.0,
                    dclevel: 128,
                    chirpconf: None,
                    pwmduty: 0.0,
                },
            );
        }
        cfg
    }
}

impl DriverConfig {
    pub fn set_control_mode(&mut self, mode: bool, task: u8, debugmode: bool) {
        self.control_mode = mode;
        if !mode {
            self.task_is_control = true;
            self.debug_mode = false;
        } else {
            self.debug_mode = debugmode;
            self.task_is_control = task == 0;
        }
    }

    pub fn set_generator_config(&mut self, id: usize, p: &GeneratorParams) {
        if self.gentipo[id] != p.tipo
            || self.genamp[id] != p.amp
            || self.genfreq[id] != p.freq
            || self.dclevel[id] != p.dclevel
        {
            self.gen_config_written[id] = false;
        }
        if p.tipo == 2 {
            let c = p.chirpconf.unwrap_or([0.0; 5]);
            let a2 = c[4].min(1.0);
            let ampaux: i32 = if id <= 2 {
                (a2 * 2047.0) as i32
            } else {
                (a2 * 127.0) as i32
            };
            let new = [
                c[0] as i32 as u8,
                (10.0 * c[1]) as i32 as u8,
                c[2] as i32 as u8,
                (10.0 * c[3]) as i32 as u8,
                ((ampaux >> 8) & 0xFF) as u8,
                (ampaux & 0xFF) as u8,
            ];
            self.chirpconf[id] = new;
            self.gen_config_written[id] = false;
        } else if p.tipo == 4 {
            let dutyaux = (p.pwmduty * 100.0 * 2.0).round() as u8;
            if self.pwmduty[id] != dutyaux {
                self.pwmduty[id] = dutyaux;
                self.gen_config_written[id] = false;
            }
        }
        self.gentipo[id] = p.tipo;
        self.genamp[id] = p.amp;
        self.genfreq[id] = p.freq;
        self.dclevel[id] = p.dclevel;
        self.iampscaler[id] = if p.dclevel == 0 {
            0.0
        } else {
            1.0 / p.dclevel as f64
        };
    }

    pub fn set_adc_config(&mut self, cfg: [u8; 3]) {
        self.adcconfig = cfg;
        const MULT: [f64; 6] = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256];
        self.adcmultiplier = MULT[(cfg[1] as usize).min(5)] / 32768.0;
        if (cfg[0] >> 4) != 0 {
            self.adcmultiplier *= 16.0; // Correction for ADS1015
        }
        self.adcenablemap = std::array::from_fn(|k| (cfg[0] >> k) & 0x01 == 1);
    }

    pub fn adc_active(&self) -> bool {
        (self.adcconfig[0] & 0x0F) > 0
    }

    /// 0 = ±2 g, 1 = ±4 g, 2 = ±8 g, 3 = ±16 g
    pub fn set_acc_range(&mut self, id: usize, nrange: u8) {
        self.accrangeselection[id] = nrange;
        self.accscaler[id] = 2f64.powi(nrange as i32 + 1) * 9.80665 / 32768.0;
    }

    /// 0 = ±125, 1 = ±250, 2 = ±500, 3 = ±1000, 4 = ±2000 °/s
    pub fn set_gyro_range(&mut self, id: usize, nrange: u8) {
        self.gyrorangeselection[id] = nrange;
        self.gyroscaler[id] = 125.0 * 2f64.powi(nrange as i32) / 32768.0;
    }

    pub fn set_imu_config(&mut self, id: usize, data: [u8; 3]) {
        self.imuconfigdata[id] = data;
        self.set_acc_range(id, data[2] & 0x03);
        self.set_gyro_range(id, (data[2] >> 2) & 0x07);
        self.imu_enable[id] = (data[0] & 0x01) == 1;
        self.imu_types[id] = (data[0] >> 1) & 0x01;
    }

    pub fn set_control_config(&mut self, c: &ControlParams) {
        self.ctrlalg = c.alg;
        self.ctrlmem = c.mem;
        self.ctrlmu = c.mu;
        self.ctrlfi = c.fi;
        self.refimuid = c.refimuid;
        self.refid = c.refid;
        self.errimuid = c.errimuid;
        self.erroid = c.erroid;
        self.ctrltask = c.ctrltask;
    }

    pub fn set_alg_on(&mut self, status: bool, algontime: f64, forcewrite: bool) {
        if status != self.algon || forcewrite {
            self.algontime = algontime;
            self.algon = status;
            self.algonchanged = true;
        }
    }

    /// Size in bytes of a measurement packet after the 3-byte header.
    pub fn reading_packet_size(&self) -> usize {
        let mut size = 0;
        for k in 0..3 {
            if self.imu_enable[k] {
                size += if self.imu_types[k] == 0 { 14 } else { 12 };
            }
        }
        let n_adcs = (self.adcconfig[0] & 0x0F).count_ones();
        size += 6 + if n_adcs == 0 { 0 } else { 8 } + 2 + 2 + 2 + 1;
        size
    }

    /// Encodes a generator configuration command (`G`).
    pub fn generator_command(&self, id: usize) -> Vec<u8> {
        let freq = self.genfreq[id];
        let fint = freq.trunc() as i64;
        let ffrac = ((freq - freq.trunc()) * 100.0).round() as i64;
        let ampaux = (self.genamp[id] * 2047.0).round() as i64;
        let dcl = self.dclevel[id];
        let mut cmd = vec![
            b'G',
            id as u8,
            self.gentipo[id],
            ((ampaux >> 8) & 0xFF) as u8,
            (ampaux & 0xFF) as u8,
            fint as u8,
            ffrac as u8,
            (dcl >> 8) as u8,
            (dcl & 0xFF) as u8,
        ];
        if self.gentipo[id] == 2 {
            cmd.extend_from_slice(&self.chirpconf[id]);
        }
        if self.gentipo[id] == 4 {
            cmd.push(self.pwmduty[id]);
        }
        cmd
    }

    /// Encodes the control configuration payload (after the `!` byte).
    pub fn control_payload(&self) -> Vec<u8> {
        let mut buf = vec![
            (self.perturb_channel << 4) + self.control_channel,
            (self.refimuid << 4) + self.refid,
            (self.errimuid << 4) + self.erroid,
            self.ctrlalg,
            (self.ctrlmem >> 8) as u8,
            (self.ctrlmem & 0xFF) as u8,
        ];
        buf.extend_from_slice(&self.ctrlmu.to_le_bytes());
        buf.extend_from_slice(&self.ctrlfi.to_le_bytes());
        buf.push(self.ctrltask + if self.debug_mode { 2 } else { 0 });
        buf
    }

    /// Encodes the predistorter command (`p`).
    pub fn predist_command(&self, id: usize) -> Vec<u8> {
        let order = if self.predistenablemap[id] {
            self.predistcoefs[id].len().saturating_sub(1)
        } else {
            0
        };
        let mut cmd = vec![b'p', id as u8, order as u8];
        if order > 0 {
            for k in 0..=order {
                cmd.extend_from_slice(&(self.predistcoefs[id][k] as f32).to_le_bytes());
            }
        }
        cmd
    }

    /// Decodes a measurement-mode packet (`buf` excludes the header).
    pub fn parse_reading(&self, buf: &[u8], out: &mut Reading) -> Result<()> {
        if buf.len() < self.reading_packet_size() {
            bail!("Incomplete reading packet.");
        }
        let be = |p: usize| i16::from_be_bytes([buf[p], buf[p + 1]]) as f64;
        let le = |p: usize| i16::from_le_bytes([buf[p], buf[p + 1]]) as f64;
        let mut ptr = 0;
        for j in 0..3 {
            if !self.imu_enable[j] {
                continue;
            }
            if self.imu_types[j] == 0 {
                for k in 0..3 {
                    out.acc[j][k] = be(ptr + 2 * k) * self.accscaler[j];
                    out.gyro[j][k] = be(ptr + 2 * k + 8) * self.gyroscaler[j];
                }
                ptr += 14;
            } else {
                for k in 0..3 {
                    out.gyro[j][k] = le(ptr + 2 * k) * self.gyroscaler[j];
                    out.acc[j][k] = le(ptr + 2 * k + 6) * self.accscaler[j];
                }
                ptr += 12;
            }
        }
        out.dacout[0] = be(ptr) * self.iampscaler[0];
        out.dacout[1] = be(ptr + 2) * self.iampscaler[1];
        out.dacout[2] = (buf[ptr + 4] as i8) as f64 * self.iampscaler[2];
        out.dacout[3] = (buf[ptr + 5] as i8) as f64 * self.iampscaler[3];
        ptr += 6;
        if self.adc_active() {
            for k in 0..4 {
                out.adcin[k] = be(ptr) * self.adcmultiplier;
                ptr += 2;
            }
        }
        for k in 0..3 {
            out.calctime[k] = u16::from_be_bytes([buf[ptr + 2 * k], buf[ptr + 2 * k + 1]]);
        }
        out.errorflag = buf[ptr + 6];
        Ok(())
    }

    /// Decodes a control-mode packet (20 bytes after the header).
    pub fn parse_control_reading(&self, buf: &[u8], out: &mut Reading) -> Result<()> {
        if buf.len() < 20 {
            bail!("Incomplete control packet.");
        }
        let be = |p: usize| i16::from_be_bytes([buf[p], buf[p + 1]]) as f64;
        out.dacout[0] = be(0) * self.iampscaler[self.perturb_channel as usize];
        out.dacout[1] = be(2) * self.iampscaler[self.control_channel as usize];
        out.xref = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]) as f64;
        out.xerro = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as f64;
        for k in 0..3 {
            out.calctime[k] = u16::from_be_bytes([buf[13 + 2 * k], buf[14 + 2 * k]]);
        }
        out.errorflag = buf[19];
        Ok(())
    }
}

/// Serial protocol commands. Each takes the link explicitly so the
/// acquisition thread can own the port without holding the config lock.
pub mod proto {
    use super::*;

    pub fn handshake(link: &mut Link) -> Result<()> {
        if link.reset_buffers().is_err() {
            // pyserial would close/reopen here; just retry the clear.
            let _ = link.reset_buffers();
        }
        for _ in 0..2 {
            link.write(b"h")?;
            if link.read_byte()? == Some(b'k') {
                return Ok(());
            }
            link.reset_buffers()?;
            std::thread::sleep(Duration::from_millis(100));
        }
        // Handshake failed: the device may be at a different baud rate.
        let wanted = link.baud;
        for bd in [500_000u32, 921_600, 1_000_000, 115_200] {
            if bd == wanted {
                continue;
            }
            link.set_baud(bd)?;
            std::thread::sleep(Duration::from_millis(50));
            link.write(b"h")?;
            if link.read_byte()? == Some(b'k') {
                log::info(&format!("Found baud rate: {bd}, adjusting to {wanted}."));
                link.reset_buffers()?;
                write_baud_rate(link, wanted)?;
                link.set_baud(wanted)?;
                std::thread::sleep(Duration::from_millis(150));
                std::thread::sleep(Duration::from_millis(100));
                link.reset_buffers()?;
                link.write(b"h")?;
                if link.read_byte()? == Some(b'k') {
                    return Ok(());
                }
                break;
            }
        }
        let _ = link.set_baud(wanted);
        bail!("Handshake with device failed.")
    }

    pub fn baud_code(baud: u32) -> u8 {
        match baud {
            115_200 => 0,
            921_600 => 2,
            1_000_000 => 3,
            _ => 1,
        }
    }

    pub fn write_baud_rate(link: &mut Link, newbaud: u32) -> Result<()> {
        let code = baud_code(newbaud);
        link.write(&[b'b', code])?;
        if link.read(2)? != [b'k', code] {
            bail!("Error setting a new baud rate.");
        }
        Ok(())
    }

    pub fn init_hardware(link: &mut Link, id: u8) -> Result<()> {
        for _ in 0..3 {
            link.write(&[b'i', id])?;
            link.flush()?;
            std::thread::sleep(Duration::from_millis(100));
            if link.read(3)? == b"ok!" {
                return Ok(());
            }
            link.reset_buffers()?;
            std::thread::sleep(Duration::from_millis(100));
        }
        bail!("Fail initializing the IMU with id={id}.")
    }

    pub fn write_imu_config(link: &mut Link, id: u8, data: &[u8; 3]) -> Result<()> {
        link.write(&[b'I', id, data[0], data[1], data[2]])?;
        link.flush()?;
        if link.read(2)? != b"KI" {
            bail!("Error writing IMU{} Config.", id + 1);
        }
        Ok(())
    }

    pub fn write_sampling_micros(link: &mut Link, micros: u32) -> Result<()> {
        if [1000, 2000, 3000, 3003, 4000].contains(&micros) {
            let ms = (micros / 1000) as u8;
            for _ in 0..5 {
                link.write(&[b'f', ms])?;
                link.flush()?;
                if link.read(2)? == [b'f', ms] {
                    return Ok(());
                }
                link.reset_buffers()?;
            }
            bail!("Error writing sampling rate.")
        } else {
            let hi = ((micros >> 8) & 0xFF) as u8;
            let lo = (micros & 0xFF) as u8;
            for _ in 0..5 {
                link.write(&[b'm', hi, lo])?;
                link.flush()?;
                if link.read(3)? == [b'm', hi, lo] {
                    return Ok(());
                }
                link.reset_buffers()?;
            }
            bail!("Error writing sampling rate (micros).")
        }
    }

    pub fn write_predist_config(link: &mut Link, cfg: &DriverConfig, id: usize) -> Result<()> {
        link.reset_buffers()?;
        let cmd = cfg.predist_command(id);
        let order = cmd[2] as usize;
        for _ in 0..5 {
            link.write(&cmd)?;
            link.flush()?;
            if link.read(2)? != b"ok" {
                std::thread::sleep(Duration::from_millis(100));
                link.reset_buffers()?;
                continue;
            }
            if order > 0 {
                let echo = link.read((order + 1) * 4)?;
                if echo != cmd[3..] {
                    bail!("Byte check failed for predistorter {id}. Please try again.");
                }
            }
            return Ok(());
        }
        bail!("Error configuring predistorter {id}.")
    }

    pub fn write_fusion_config(link: &mut Link, weights: [f64; 2]) -> Result<()> {
        link.reset_buffers()?;
        let mut cmd = vec![b'F'];
        for w in weights {
            cmd.extend_from_slice(&(w as f32).to_le_bytes());
        }
        for _ in 0..5 {
            link.write(&cmd)?;
            link.flush()?;
            if link.read(2)? != b"ok" {
                std::thread::sleep(Duration::from_millis(100));
                link.reset_buffers()?;
                continue;
            }
            if link.read(8)? != cmd[1..] {
                bail!("Byte check failed for sensor fusion weights. Please try again.");
            }
            return Ok(());
        }
        bail!("Error configuring fusion coefficients.")
    }

    pub fn record_additional_configs(link: &mut Link) -> Result<()> {
        link.write(b"w")?;
        if link.read(3)? != b"ok!" {
            bail!("Failed record predist data in device memory.");
        }
        Ok(())
    }

    pub fn write_generator_config(
        link: &mut Link,
        cfg: &mut DriverConfig,
        id: usize,
    ) -> Result<()> {
        link.write(&cfg.generator_command(id))?;
        cfg.gen_config_written[id] = true;
        Ok(())
    }

    /// Writes the ADC configuration and returns the selected ADC reported by the device.
    pub fn write_adc_config(link: &mut Link, cfg: &mut DriverConfig) -> Result<()> {
        link.write(&[b'd', cfg.adcconfig[0], cfg.adcconfig[1], cfg.adcconfig[2]])?;
        if link.read(2)? != b"KA" {
            bail!("ADC configuration error.");
        }
        let sel = link.read(4)?;
        cfg.adcsel = sel.first().map(|b| *b as i8).unwrap_or(0);
        if (cfg.adcconfig[0] & 0x0F) == 0 {
            cfg.adcsel = 0;
        }
        Ok(())
    }

    pub fn write_control_config(link: &mut Link, cfg: &DriverConfig) -> Result<()> {
        link.write(b"!")?;
        link.write(&cfg.control_payload())?;
        if link.read(3)? != b"ok!" {
            bail!("Fail recording control configurations.");
        }
        Ok(())
    }

    pub fn write_alg_on(link: &mut Link, cfg: &mut DriverConfig) -> Result<()> {
        if cfg.algon {
            let mut cmd = vec![b'a', 2];
            cmd.extend_from_slice(&cfg.ctrlmu.to_le_bytes());
            cmd.extend_from_slice(&cfg.ctrlfi.to_le_bytes());
            link.write(&cmd)?;
        } else {
            link.write(&[b'a', 0])?;
        }
        cfg.algonchanged = false;
        Ok(())
    }

    pub fn start_readings(link: &mut Link) -> Result<()> {
        link.write(b"s")?;
        if link.read(10)?.len() < 10 {
            bail!("No response when starting readings.");
        }
        Ok(())
    }

    pub fn start_control(link: &mut Link) -> Result<()> {
        link.write(b"S")?;
        if link.read_byte()? != Some(b'K') {
            bail!("No response when starting control.");
        }
        Ok(())
    }

    /// Sends the stop command and drains incoming data until the line is quiet.
    pub fn stop_readings(link: &mut Link) -> Result<()> {
        link.write(b"t")?;
        loop {
            let data = link.read(1000)?;
            if data.is_empty() {
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        Ok(())
    }

    pub fn write_flash(link: &mut Link) -> Result<()> {
        link.write(b"P")?;
        std::thread::sleep(Duration::from_secs(1));
        if link.read(2)? != b"ok" {
            bail!("No response while writing to flash.");
        }
        Ok(())
    }

    /// Uploads path weights (`tipo` = s/f for plain, S/F for SVD-compressed with leading bytes).
    pub fn upload_path(
        link: &mut Link,
        tipo: u8,
        data: &[f64],
        leading: Option<&[u8]>,
        mut progress: impl FnMut(usize, usize),
    ) -> Result<()> {
        const BUFFER_SIZE: usize = 64;
        const FLOATS_PER_PACKET: usize = BUFFER_SIZE / 4;
        if data.is_empty() || data.len() > 3000 {
            bail!("Path data must contain between 1 and 3000 values.");
        }
        let data_size = data.len() * 4;
        let mut done = 0usize;
        progress(0, data_size);
        link.write(&[b'W', tipo])?;
        link.write(&[(data_size >> 8) as u8, (data_size & 0xFF) as u8])?;
        if let Some(lb) = leading {
            link.write(lb)?;
        }
        if link.read_byte()? != Some(b'k') {
            bail!("Device did not acknowledge the path upload.");
        }
        for chunk in data.chunks(FLOATS_PER_PACKET) {
            let bytes: Vec<u8> = chunk
                .iter()
                .flat_map(|v| (*v as f32).to_le_bytes())
                .collect();
            link.write(&bytes)?;
            let resp = link.read(2)?;
            if resp.len() != 2 {
                bail!("Incomplete response during path upload.");
            }
            done += u16::from_be_bytes([resp[0], resp[1]]) as usize;
            progress(done.min(data_size), data_size);
        }
        progress(data_size, data_size);
        Ok(())
    }

    /// Reads the secondary and feedback paths stored in the device.
    pub fn read_paths(link: &mut Link) -> Result<(Vec<f64>, Vec<f64>)> {
        link.write(b"c")?;
        let first = link.readline()?;
        if !first.starts_with("WSec =") {
            bail!("Error reading paths (at the beginning).");
        }
        let mut wsec = Vec::new();
        let mut wfbk = Vec::new();
        let mut line = link.readline()?;
        while !line.is_empty() && !line.starts_with("WFbk =") {
            wsec.push(
                line.trim()
                    .parse::<f64>()
                    .map_err(|e| anyhow!("Invalid path value '{}': {e}", line.trim()))?,
            );
            line = link.readline()?;
        }
        line = link.readline()?;
        while !line.is_empty() && !line.starts_with("End") {
            wfbk.push(
                line.trim()
                    .parse::<f64>()
                    .map_err(|e| anyhow!("Invalid path value '{}': {e}", line.trim()))?,
            );
            line = link.readline()?;
        }
        Ok((wsec, wfbk))
    }

    /// Waits for the 0x0F 0x0F 0x0F header and decodes the next packet.
    pub fn get_reading(link: &mut Link, cfg: &DriverConfig, out: &mut Reading) -> Result<()> {
        let mut h = [0u8; 3];
        let mut ct = 0;
        while !(h == [0xF, 0xF, 0xF]) && ct < 64 {
            h[2] = h[1];
            h[1] = h[0];
            h[0] = link
                .read_byte()?
                .ok_or_else(|| anyhow!("Timeout waiting for data from device."))?;
            ct += 1;
        }
        if ct == 64 {
            bail!("Packet read failure: header not found.");
        }
        if cfg.control_mode {
            let buf = link.read_exact(20, "control packet")?;
            cfg.parse_control_reading(&buf, out)
        } else {
            let buf = link.read_exact(cfg.reading_packet_size(), "reading packet")?;
            cfg.parse_reading(&buf, out)
        }
    }
}

/// Simulated beam used by the (developer-only) debug mode.
pub struct DebugSim {
    pub beam: CantileverBeam,
    pperturb: usize,
    pref: usize,
    pcanc: usize,
    perro: usize,
    pub debug_err: f64,
    pub debug_ref: f64,
    pub debug_perturb: f64,
    pub debug_control: f64,
}

impl DebugSim {
    pub fn new() -> Self {
        let mut beam = CantileverBeam::new(crate::beam::BeamParams {
            tsampling: 4e-3,
            nmodes: 2,
            length: 0.538,
            width: 0.0507,
            thickness: 0.00474 - 0.0001 * 2.0,
            damp: vec![0.010826771653543302, 0.0076347771891554575, 0.0, 0.0, 0.0],
            density: 7850.0,
            elasticmod: 200e9,
            ..Default::default()
        });
        beam.configforcescaler(25.0 / 12.0, 3e-3);
        let np = beam.npoints as f64;
        let len = beam.length;
        let pos = |x: f64| ((x / len) * np).round() as usize;
        beam.noisestd = 0.0045;
        beam.reset();
        Self {
            pperturb: pos(0.34),
            pref: pos(0.29),
            pcanc: pos(0.20),
            perro: pos(0.53),
            beam,
            debug_err: 1.0,
            debug_ref: -1.0,
            debug_perturb: 0.0,
            debug_control: 0.0,
        }
    }

    /// One debug-mode exchange: send simulated sensors, read outputs, step the beam.
    pub fn talk(&mut self, link: &mut Link, cfg: &DriverConfig) -> Result<()> {
        self.debug_err = self.beam.getaccel(self.perro);
        self.debug_ref = self.beam.getaccel(self.pref);
        let mut payload = Vec::with_capacity(8);
        payload.extend_from_slice(&(self.debug_err as f32).to_le_bytes());
        payload.extend_from_slice(&(self.debug_ref as f32).to_le_bytes());
        loop {
            link.write(b"T")?;
            link.write(&payload)?;
            let echo = link.read(8)?;
            if echo == payload {
                break;
            }
            link.reset_buffers()?;
        }
        link.write(b"N")?;
        let buf = link.read(4)?;
        if buf.len() == 4 {
            let pc = cfg.perturb_channel as usize;
            let cc = cfg.control_channel as usize;
            let p = i16::from_be_bytes([buf[0], buf[1]]) as f64;
            let c = i16::from_be_bytes([buf[2], buf[3]]) as f64;
            self.debug_perturb = -(p - cfg.dclevel[pc] as f64) * cfg.iampscaler[pc];
            self.debug_control = -(c - cfg.dclevel[cc] as f64) * cfg.iampscaler[cc];
        } else {
            self.debug_perturb = 0.0;
            self.debug_control = 0.0;
        }
        self.beam.setforce(self.pperturb, self.debug_perturb);
        self.beam.setforce(self.pcanc, self.debug_control);
        self.beam.update();
        Ok(())
    }
}

impl Default for DebugSim {
    fn default() -> Self {
        Self::new()
    }
}

/// Lists serial ports available on the system.
pub fn list_ports() -> Vec<serialport::SerialPortInfo> {
    let mut ports = serialport::available_ports().unwrap_or_default();
    ports.sort_by(|a, b| a.port_name.cmp(&b.port_name));
    ports
}

/// Returns the USB vendor ID of a port, if any.
pub fn port_vid(info: &serialport::SerialPortInfo) -> Option<u16> {
    match &info.port_type {
        serialport::SerialPortType::UsbPort(u) => Some(u.vid),
        _ => None,
    }
}

pub mod log {
    pub fn info(msg: &str) {
        eprintln!("[actvib] {msg}");
    }
}

#[cfg(test)]
pub mod mock {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// In-memory transport: `incoming` is what the "device" sends, `written` records host writes.
    #[derive(Clone, Default)]
    pub struct MockPort {
        pub incoming: Arc<Mutex<VecDeque<u8>>>,
        pub written: Arc<Mutex<Vec<u8>>>,
    }

    impl MockPort {
        pub fn feed(&self, data: &[u8]) {
            self.incoming.lock().unwrap().extend(data.iter().copied());
        }
        pub fn written(&self) -> Vec<u8> {
            self.written.lock().unwrap().clone()
        }
    }

    impl Transport for MockPort {
        fn write_all(&mut self, data: &[u8]) -> std::io::Result<()> {
            self.written.lock().unwrap().extend_from_slice(data);
            Ok(())
        }
        fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut q = self.incoming.lock().unwrap();
            let n = buf.len().min(q.len());
            for b in buf.iter_mut().take(n) {
                *b = q.pop_front().unwrap();
            }
            Ok(n)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
        fn clear_input(&mut self) -> std::io::Result<()> {
            Ok(())
        }
        fn clear_output(&mut self) -> std::io::Result<()> {
            Ok(())
        }
        fn set_baud_rate(&mut self, _baud: u32) -> std::io::Result<()> {
            Ok(())
        }
        fn set_dtr(&mut self, _level: bool) -> std::io::Result<()> {
            Ok(())
        }
        fn set_rts(&mut self, _level: bool) -> std::io::Result<()> {
            Ok(())
        }
    }

    pub fn link(port: &MockPort) -> Link {
        let mut l = Link::new(Box::new(port.clone()), DEFAULT_BAUD);
        l.timeout = Duration::from_millis(20);
        l
    }
}

#[cfg(test)]
mod tests {
    use super::mock::*;
    use super::*;

    #[test]
    fn generator_command_encoding() {
        let mut cfg = DriverConfig::default();
        cfg.set_generator_config(
            1,
            &GeneratorParams {
                tipo: 1,
                amp: 0.5,
                freq: 12.34,
                dclevel: 1668,
                chirpconf: None,
                pwmduty: 0.0,
            },
        );
        let cmd = cfg.generator_command(1);
        // amp = round(0.5*2047) = 1024 -> 0x04 0x00; freq 12 + 34/100; dclevel 1668 = 0x0684
        assert_eq!(cmd, vec![b'G', 1, 1, 0x04, 0x00, 12, 34, 0x06, 0x84]);
        assert!(!cfg.gen_config_written[1]);
    }

    #[test]
    fn chirp_command_has_extra_bytes() {
        let mut cfg = DriverConfig::default();
        cfg.set_generator_config(
            0,
            &GeneratorParams {
                tipo: 2,
                amp: 0.1,
                freq: 10.0,
                dclevel: 1668,
                chirpconf: Some([10.0, 1.5, 100.0, 2.0, 2.0]),
                pwmduty: 0.0,
            },
        );
        let cmd = cfg.generator_command(0);
        assert_eq!(cmd.len(), 9 + 6);
        // A2 clamped to 1.0 -> 2047 = 0x07FF
        assert_eq!(&cmd[9..], &[10, 15, 100, 20, 0x07, 0xFF]);
    }

    #[test]
    fn imu_config_scalers() {
        let mut cfg = DriverConfig::default();
        // enabled LSM, bus 0, addr 0x6A, acc range 1, gyro range 2
        cfg.set_imu_config(0, [0b011, 0x6A, (2 << 2) | 1]);
        assert!(cfg.imu_enable[0]);
        assert_eq!(cfg.imu_types[0], 1);
        assert!((cfg.accscaler[0] - 4.0 * 9.80665 / 32768.0).abs() < 1e-12);
        assert!((cfg.gyroscaler[0] - 500.0 / 32768.0).abs() < 1e-12);
    }

    #[test]
    fn reading_packet_size_and_parse() {
        let mut cfg = DriverConfig::default();
        cfg.set_imu_config(0, [0b001, 0x68, 0]); // MPU
        cfg.set_imu_config(1, [0b011, 0x6A, 0]); // LSM
        cfg.set_adc_config([0x01, 0, 0]);
        assert_eq!(cfg.reading_packet_size(), 14 + 12 + 6 + 8 + 7);
        let mut buf = vec![0u8; cfg.reading_packet_size()];
        buf[0..2].copy_from_slice(&1000i16.to_be_bytes()); // MPU acc x
        buf[8..10].copy_from_slice(&(-200i16).to_be_bytes()); // MPU gyro x
        buf[14..16].copy_from_slice(&300i16.to_le_bytes()); // LSM gyro x
        buf[20..22].copy_from_slice(&(-400i16).to_le_bytes()); // LSM acc x
        buf[26..28].copy_from_slice(&834i16.to_be_bytes()); // dac0
        buf[30] = (-52i8) as u8; // dac2
        buf[32..34].copy_from_slice(&16384i16.to_be_bytes()); // adc0
        buf[40..42].copy_from_slice(&4000u16.to_be_bytes());
        buf[46] = 0x21;
        let mut r = Reading::default();
        cfg.parse_reading(&buf, &mut r).unwrap();
        assert!((r.acc[0][0] - 1000.0 * cfg.accscaler[0]).abs() < 1e-12);
        assert!((r.gyro[0][0] + 200.0 * cfg.gyroscaler[0]).abs() < 1e-12);
        assert!((r.gyro[1][0] - 300.0 * cfg.gyroscaler[1]).abs() < 1e-12);
        assert!((r.acc[1][0] + 400.0 * cfg.accscaler[1]).abs() < 1e-12);
        assert!((r.dacout[0] - 834.0 * cfg.iampscaler[0]).abs() < 1e-12);
        assert!((r.dacout[2] + 52.0 * cfg.iampscaler[2]).abs() < 1e-12);
        assert!((r.adcin[0] - 16384.0 * 6.144 / 32768.0).abs() < 1e-12);
        assert_eq!(r.calctime[0], 4000);
        assert_eq!(r.errorflag, 0x21);
    }

    #[test]
    fn control_payload_layout() {
        let mut cfg = DriverConfig::default();
        cfg.perturb_channel = 1;
        cfg.control_channel = 0;
        cfg.set_control_config(&ControlParams {
            alg: 2,
            mem: 300,
            mu: 0.01,
            fi: 1e-4,
            refimuid: 0,
            errimuid: 1,
            refid: 2,
            erroid: 3,
            ctrltask: 1,
        });
        let p = cfg.control_payload();
        assert_eq!(p.len(), 15);
        assert_eq!(&p[..6], &[0x10, 0x02, 0x13, 2, 1, 44]);
        assert_eq!(&p[6..10], &0.01f32.to_le_bytes());
        assert_eq!(p[14], 1);
    }

    #[test]
    fn handshake_and_get_reading_over_mock() {
        let port = MockPort::default();
        let mut link = link(&port);
        port.feed(b"k");
        proto::handshake(&mut link).unwrap();
        assert_eq!(port.written(), b"h");

        let mut cfg = DriverConfig::default();
        cfg.control_mode = true;
        cfg.perturb_channel = 1;
        cfg.control_channel = 0;
        let mut pkt = vec![0x33, 0x0F, 0x0F, 0x0F];
        pkt.extend_from_slice(&834i16.to_be_bytes());
        pkt.extend_from_slice(&(-834i16).to_be_bytes());
        pkt.extend_from_slice(&1.5f32.to_le_bytes());
        pkt.extend_from_slice(&(-2.5f32).to_le_bytes());
        pkt.push(0);
        pkt.extend_from_slice(&[0x0F, 0xA0, 0x00, 0x10, 0x00, 0x20, 0x00]);
        port.feed(&pkt);
        let mut r = Reading::default();
        proto::get_reading(&mut link, &cfg, &mut r).unwrap();
        assert!((r.dacout[0] - 834.0 * cfg.iampscaler[1]).abs() < 1e-9);
        assert!((r.dacout[1] + 834.0 * cfg.iampscaler[0]).abs() < 1e-9);
        assert_eq!(r.xref, 1.5);
        assert_eq!(r.xerro, -2.5);
        assert_eq!(r.calctime, [4000, 16, 32]);
    }

    #[test]
    fn upload_path_protocol() {
        let port = MockPort::default();
        let mut link = link(&port);
        let data: Vec<f64> = (0..20).map(|v| v as f64).collect();
        port.feed(b"k");
        port.feed(&[0, 64]);
        port.feed(&[0, 16]);
        let mut last = (0, 0);
        proto::upload_path(&mut link, b's', &data, None, |a, b| last = (a, b)).unwrap();
        assert_eq!(last, (80, 80));
        let w = port.written();
        assert_eq!(&w[..4], &[b'W', b's', 0, 80]);
        assert_eq!(w.len(), 4 + 80);
        assert_eq!(&w[4..8], &0f32.to_le_bytes());
    }

    #[test]
    fn read_paths_parses_lines() {
        let port = MockPort::default();
        let mut link = link(&port);
        port.feed(b"WSec = \r\n0.5\r\n-0.25\r\nWFbk = \r\n1.0\r\nEnd!\r\n");
        let (s, f) = proto::read_paths(&mut link).unwrap();
        assert_eq!(s, vec![0.5, -0.25]);
        assert_eq!(f, vec![1.0]);
    }

    #[test]
    fn sampling_command_selection() {
        let port = MockPort::default();
        let mut link = link(&port);
        port.feed(&[b'f', 4]);
        proto::write_sampling_micros(&mut link, 4000).unwrap();
        port.feed(&[b'm', 0x12, 0xC1]);
        proto::write_sampling_micros(&mut link, 4801).unwrap();
        assert_eq!(port.written(), vec![b'f', 4, b'm', 0x12, 0xC1]);
    }
}
