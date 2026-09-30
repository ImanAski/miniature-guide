//! Configuration for the Lithowrite application.
//!
//! Stores all user-configurable parameters including:
//! - Motion controller settings
//! - Laser parameters
//! - Default processing settings
//! - UI preferences

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// Main application configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    pub controller: ControllerConfig,
    pub laser: LaserConfig,
    pub processing: ProcessingConfig,
    pub ui: UiConfig,
    pub file: FileConfig,
}

/// Motion controller configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerConfig {
    pub port: String,
    pub baud_rate: u32,
    pub timeout_secs: f64,
    /// Default velocity in mm/s
    pub default_velocity: f64,
    /// Default acceleration in mm/s²
    pub default_acceleration: f64,
    /// Unit scale (1.0 = mm, 0.001 = um)
    pub unit_scale: f64,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        ControllerConfig {
            port: "COM3".to_string(),
            baud_rate: 115200,
            timeout_secs: 5.0,
            default_velocity: 50.0,
            default_acceleration: 500.0,
            unit_scale: 1.0,
        }
    }
}

/// Laser configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserConfig {
    pub digital_output_pin: u8,
    /// Laser wavelength in nm
    pub wavelength_nm: u32,
    /// Default power (0-100)
    pub default_power: f64,
    /// Frequency in Hz
    pub default_frequency: f64,
    /// Minimum pulse duration in microseconds
    pub min_pulse_us: f64,
    /// Fade-in/fade-out time in ms
    pub fade_time_ms: f64,
    /// Enable laser interlock check
    pub interlock_enabled: bool,
}

impl Default for LaserConfig {
    fn default() -> Self {
        LaserConfig {
            digital_output_pin: 1,
            wavelength_nm: 320, // UV 320nm (close to UV320NM)
            default_power: 50.0,
            default_frequency: 1000.0,
            min_pulse_us: 10.0,
            fade_time_ms: 5.0,
            interlock_enabled: true,
        }
    }
}

/// Processing/Engine configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessingConfig {
    /// Maximum line width before switching to raster mode
    pub raster_threshold: f64,
    /// Hatch spacing in raster mode (mm)
    pub hatch_spacing: f64,
    /// Hatch angle in degrees
    pub hatch_angle: f64,
    /// Enable multi-pass for wide features
    pub enable_multi_pass: bool,
    /// Number of passes for raster
    pub raster_passes: u32,
    /// Overlap percentage between passes
    pub overlap_percent: f64,
    /// Min feature size (mm)
    pub min_feature_size: f64,
    /// Tolerance for path approximation (mm)
    pub approximation_tolerance: f64,
    /// Enable vector-only mode (no raster)
    pub vector_only: bool,
}

impl Default for ProcessingConfig {
    fn default() -> Self {
        ProcessingConfig {
            raster_threshold: 0.1, // 0.1mm threshold
            hatch_spacing: 0.02,
            hatch_angle: 45.0,
            enable_multi_pass: true,
            raster_passes: 2,
            overlap_percent: 10.0,
            min_feature_size: 0.005,
            approximation_tolerance: 0.001,
            vector_only: false,
        }
    }
}

/// UI configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UiConfig {
    pub font_size: f32,
    pub show_grid: bool,
    pub grid_spacing: f64,
    pub show_coordinates: bool,
    pub show_status_bar: bool,
    pub theme: UiTheme,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum UiTheme {
    Dark,
    Light,
}

impl Default for UiConfig {
    fn default() -> Self {
        UiConfig {
            font_size: 14.0,
            show_grid: true,
            grid_spacing: 1.0,
            show_coordinates: true,
            show_status_bar: true,
            theme: UiTheme::Dark,
        }
    }
}

/// File handling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FileConfig {
    /// Default directory for opening files
    pub open_dir: PathBuf,
    /// Default directory for saving files
    pub save_dir: PathBuf,
    /// Last opened file
    pub last_file: Option<PathBuf>,
}

impl Default for FileConfig {
    fn default() -> Self {
        FileConfig {
            open_dir: PathBuf::from("."),
            save_dir: PathBuf::from("."),
            last_file: None,
        }
    }
}

impl Default for AppConfig {
    fn default() -> Self {
        AppConfig {
            controller: ControllerConfig::default(),
            laser: LaserConfig::default(),
            processing: ProcessingConfig::default(),
            ui: UiConfig::default(),
            file: FileConfig::default(),
        }
    }
}

impl AppConfig {
    /// Load configuration from file
    pub fn load(path: &PathBuf) -> Result<Self, String> {
        let content =
            std::fs::read_to_string(path).map_err(|e| format!("Failed to read config: {}", e))?;
        serde_json::from_str(&content).map_err(|e| format!("Failed to parse config: {}", e))
    }

    /// Save configuration to file
    pub fn save(&self, path: &PathBuf) -> Result<(), String> {
        let content = serde_json::to_string_pretty(self)
            .map_err(|e| format!("Failed to serialize config: {}", e))?;
        std::fs::write(path, content).map_err(|e| format!("Failed to write config: {}", e))
    }

    /// Get the default config file path
    pub fn default_path() -> PathBuf {
        let base = directories::BaseDirs::new();
        let config_dir = base
            .map(|d| d.config_dir().to_path_buf())
            .unwrap_or_else(|| PathBuf::from("."));
        let mut path = config_dir;
        path.push("lithowrite");
        std::fs::create_dir_all(&path).ok();
        path.push("config.json");
        path
    }
}
