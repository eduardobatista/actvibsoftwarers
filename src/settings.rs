//! Persistent application settings (replaces `QSettings("VibSoftware", "VibView")`).

use std::path::PathBuf;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

use crate::dialogs::PathModelingOptions;
use crate::figures::{CtrlFig, MainFig, OutFig};
use crate::panels::{AdcPanel, ControlPanel, GeneratorPanel, ImuPanel};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Settings {
    pub port: Option<String>,
    pub fsampling: u32,
    pub maxtime: u32,
    pub baud: u32,
    pub workdir: Option<PathBuf>,
    pub last_data_folder: Option<PathBuf>,
    pub last_saved_ext: Option<String>,
    pub predist_enable: [bool; 4],
    pub predist_coefs: [Vec<f64>; 4],
    pub fusion_weights: [f64; 2],
    pub mode: usize,
    pub imu: [ImuPanel; 3],
    pub gens: [GeneratorPanel; 4],
    pub ctrl: ControlPanel,
    pub adc: AdcPanel,
    pub mainfig: MainFig,
    pub ctrlfig: CtrlFig,
    pub outfig: OutFig,
    pub path_modeling: Option<PathModelingOptions>,
    pub sensor_tab: usize,
    pub output_tab: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            port: None,
            fsampling: 250,
            maxtime: 10,
            baud: crate::driver::DEFAULT_BAUD,
            workdir: None,
            last_data_folder: None,
            last_saved_ext: None,
            predist_enable: [false; 4],
            predist_coefs: std::array::from_fn(|_| vec![1.0, 0.0]),
            fusion_weights: [0.5, 0.5],
            mode: 0,
            imu: Default::default(),
            gens: std::array::from_fn(GeneratorPanel::new),
            ctrl: ControlPanel::default(),
            adc: AdcPanel::default(),
            mainfig: MainFig::default(),
            ctrlfig: CtrlFig::default(),
            outfig: OutFig::default(),
            path_modeling: None,
            sensor_tab: 0,
            output_tab: 0,
        }
    }
}

pub fn settings_path() -> Option<PathBuf> {
    directories::ProjectDirs::from("", "ActVib", "ActVib")
        .map(|d| d.config_dir().join("settings.json"))
}

impl Settings {
    pub fn load() -> Settings {
        let Some(p) = settings_path() else {
            return Settings::default();
        };
        let mut s: Settings = std::fs::read_to_string(&p)
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        for (k, g) in s.gens.iter_mut().enumerate() {
            g.id = k;
            g.clamp();
        }
        for i in s.imu.iter_mut() {
            i.normalize();
        }
        s
    }

    pub fn save(&self) -> Result<()> {
        let p = settings_path().context("No configuration directory available")?;
        if let Some(dir) = p.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(&p, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_json() {
        let mut s = Settings::default();
        s.port = Some("/dev/ttyUSB0".into());
        s.gens[2].dclevel = 100;
        s.ctrl.step = "0.002".into();
        let t = serde_json::to_string(&s).unwrap();
        let mut r: Settings = serde_json::from_str(&t).unwrap();
        for (k, g) in r.gens.iter_mut().enumerate() {
            g.id = k;
        }
        assert_eq!(r.port, s.port);
        assert_eq!(r.gens[2].dclevel, 100);
        assert_eq!(r.ctrl.step, "0.002");
        // Unknown/missing fields fall back to defaults.
        let partial: Settings = serde_json::from_str(r#"{"fsampling": 500}"#).unwrap();
        assert_eq!(partial.fsampling, 500);
        assert_eq!(partial.maxtime, 10);
    }
}
