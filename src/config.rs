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
    /// Hybrid raster/vector write planner (absent from older config files).
    #[serde(default)]
    pub planner: PlannerConfig,
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
    /// Fire the beam automatically while a path-follow run is active.
    #[serde(default = "default_expose_on_follow")]
    pub expose_on_follow: bool,
}

fn default_expose_on_follow() -> bool {
    true
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
            expose_on_follow: default_expose_on_follow(),
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

// ─── Hybrid write planner ────────────────────────────────────────────

/// Parameters of the hybrid raster/vector write planner.
///
/// Unit convention matches the rest of the app: lengths in **mm**, speeds in
/// **mm/s**, accelerations in **mm/s²**, times in **s**. The example values in
/// `TASKS.md` are written in µm — divide them by 1000 (or 1e6 for areas) to
/// compare. Nothing here is hard-coded elsewhere: the planner reads only what
/// is stored in this struct, so changing machine parameters changes decisions.
///
/// Missing sections/keys fall back to [`Default`] (`#[serde(default)]`), so
/// config files written before the planner existed still load.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlannerConfig {
    pub raster: RasterConfig,
    pub vector: VectorConfig,
    pub stage: StageConfig,
    pub beam: BeamConfig,
    /// Spatial tiling grid size (mm). The grid is aligned to multiples of
    /// this value, so tiles are reproducible for a given layout.
    pub tile_size_mm: f64,
    /// Hard cap on tile count: the tile size doubles until the layout fits,
    /// so tiny tile sizes cannot explode into millions of empty tiles.
    pub max_tiles: usize,
    /// Weight of write time in the cost function.
    pub alpha: f64,
    /// Weight of dose (dose uniformity) error.
    pub beta: f64,
    /// Weight of positioning error.
    pub gamma: f64,
    /// Weight of stage/beam jumps (travels between independent strokes).
    pub delta: f64,
    /// Weight of acceleration events (direction changes / line turns).
    pub epsilon: f64,
    /// Maximum tolerated scanline dose ripple (0..1). Raster (and the raster
    /// part of hybrid) is rejected when the modelled ripple exceeds this.
    pub dose_tolerance: f64,
}

/// Raster writer parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RasterConfig {
    /// Distance between scan lines (mm).
    pub pitch_mm: f64,
    /// Exposure scan speed (mm/s), clamped to the stage's max velocity.
    pub speed_mm_s: f64,
    /// Minimum time to return to the start of the next line (s).
    pub flyback_s: f64,
    /// Scan every line in the same direction (false) or both directions (true).
    pub bidirectional: bool,
    /// Travel beyond both span ends so the speed is constant while the beam
    /// is on (mm).
    pub overscan_mm: f64,
    /// One-off overhead each time a raster write starts (s).
    pub setup_s: f64,
}

impl Default for RasterConfig {
    fn default() -> Self {
        RasterConfig {
            pitch_mm: 0.0005, // 0.5 µm
            speed_mm_s: 1.0,  // 1000 µm/s
            flyback_s: 0.001, // 1 ms
            bidirectional: true,
            overscan_mm: 0.002, // 2 µm
            setup_s: 0.05,      // 50 ms
        }
    }
}

/// Vector writer parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VectorConfig {
    /// Contour/fill stroke speed (mm/s), clamped to the stage's max velocity.
    pub write_speed_mm_s: f64,
    /// Beam-off repositioning speed (mm/s), clamped to the stage's max velocity.
    pub jump_speed_mm_s: f64,
    /// Time lost at each corner of a vector path (s).
    pub corner_time_s: f64,
    /// Fixed overhead of each jump between independent strokes (s).
    pub jump_time_s: f64,
}

impl Default for VectorConfig {
    fn default() -> Self {
        VectorConfig {
            write_speed_mm_s: 0.5, // 500 µm/s
            jump_speed_mm_s: 5.0,  // 5000 µm/s
            corner_time_s: 0.0001, // 0.1 ms
            jump_time_s: 0.001,    // 1 ms
        }
    }
}

/// Motion-stage limits used by the travel-time model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StageConfig {
    /// Maximum stage velocity (mm/s).
    pub max_velocity_mm_s: f64,
    /// Maximum stage acceleration (mm/s²) — drives the short-move penalty.
    pub max_acceleration_mm_s2: f64,
    /// Settling time after each direction change (s).
    pub settling_time_s: f64,
}

impl Default for StageConfig {
    fn default() -> Self {
        StageConfig {
            max_velocity_mm_s: 10.0,      // 10 mm/s = 10 000 µm/s
            max_acceleration_mm_s2: 50.0, // 50 mm/s² = 50 000 µm/s²
            settling_time_s: 0.0005,      // 0.5 ms
        }
    }
}

/// Beam parameters: what the optics can actually resolve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BeamConfig {
    /// Beam spot diameter (mm): strokes of about this width expose features.
    pub spot_size_mm: f64,
    /// Smallest feature raster mode may attempt (mm). Regions containing
    /// narrower features are written vectorially instead.
    pub minimum_feature_mm: f64,
}

impl Default for BeamConfig {
    fn default() -> Self {
        BeamConfig {
            spot_size_mm: 0.001,        // 1 µm
            minimum_feature_mm: 0.0008, // 0.8 µm
        }
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        PlannerConfig {
            raster: RasterConfig::default(),
            vector: VectorConfig::default(),
            stage: StageConfig::default(),
            beam: BeamConfig::default(),
            tile_size_mm: 0.1, // 100 µm
            max_tiles: 4096,
            alpha: 1.0,
            beta: 10.0,
            gamma: 10.0,
            delta: 0.5,
            epsilon: 0.5,
            dose_tolerance: 0.1,
        }
    }
}

impl PlannerConfig {
    /// Reject parameter sets the cost model cannot evaluate sanely.
    /// Every message names the offending field so the config file can be fixed.
    pub fn validate(&self) -> Result<(), String> {
        let mut errs: Vec<String> = Vec::new();
        let pos = |name: &str, v: f64, errs: &mut Vec<String>| {
            if !(v.is_finite() && v > 0.0) {
                errs.push(format!("{name} must be > 0 (got {v})"));
            }
        };
        pos("planner.tile_size_mm", self.tile_size_mm, &mut errs);
        pos("planner.raster.pitch_mm", self.raster.pitch_mm, &mut errs);
        pos(
            "planner.raster.speed_mm_s",
            self.raster.speed_mm_s,
            &mut errs,
        );
        pos(
            "planner.raster.overscan_mm",
            self.raster.overscan_mm,
            &mut errs,
        );
        pos("planner.raster.setup_s", self.raster.setup_s, &mut errs);
        pos(
            "planner.vector.write_speed_mm_s",
            self.vector.write_speed_mm_s,
            &mut errs,
        );
        pos(
            "planner.vector.jump_speed_mm_s",
            self.vector.jump_speed_mm_s,
            &mut errs,
        );
        pos(
            "planner.stage.max_velocity_mm_s",
            self.stage.max_velocity_mm_s,
            &mut errs,
        );
        pos(
            "planner.stage.max_acceleration_mm_s2",
            self.stage.max_acceleration_mm_s2,
            &mut errs,
        );
        pos(
            "planner.beam.spot_size_mm",
            self.beam.spot_size_mm,
            &mut errs,
        );
        pos(
            "planner.beam.minimum_feature_mm",
            self.beam.minimum_feature_mm,
            &mut errs,
        );
        if !(self.raster.flyback_s >= 0.0 && self.raster.flyback_s.is_finite()) {
            errs.push(format!(
                "planner.raster.flyback_s must be >= 0 (got {})",
                self.raster.flyback_s
            ));
        }
        if !(self.vector.corner_time_s >= 0.0 && self.vector.corner_time_s.is_finite()) {
            errs.push(format!(
                "planner.vector.corner_time_s must be >= 0 (got {})",
                self.vector.corner_time_s
            ));
        }
        if !(self.vector.jump_time_s >= 0.0 && self.vector.jump_time_s.is_finite()) {
            errs.push(format!(
                "planner.vector.jump_time_s must be >= 0 (got {})",
                self.vector.jump_time_s
            ));
        }
        if !(self.stage.settling_time_s >= 0.0 && self.stage.settling_time_s.is_finite()) {
            errs.push(format!(
                "planner.stage.settling_time_s must be >= 0 (got {})",
                self.stage.settling_time_s
            ));
        }
        if !(self.dose_tolerance.is_finite() && self.dose_tolerance > 0.0) {
            errs.push(format!(
                "planner.dose_tolerance must be > 0 (got {})",
                self.dose_tolerance
            ));
        }
        if self.max_tiles == 0 {
            errs.push("planner.max_tiles must be >= 1".to_string());
        }
        if errs.is_empty() {
            Ok(())
        } else {
            Err(errs.join("; "))
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
            planner: PlannerConfig::default(),
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
