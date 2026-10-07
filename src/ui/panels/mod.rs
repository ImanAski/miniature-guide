//! The application's widgets. See [`crate::ui`] for how to add one.

pub mod config;
pub mod geometry;
pub mod log;
pub mod motion;
pub mod planner;
pub mod status;
pub mod viewport;

pub use config::ConfigPanel;
pub use geometry::GeometryPanel;
pub use log::LogPanel;
pub use motion::MotionPanel;
pub use planner::{PlannerPanel, mode_color};
pub use status::StatusPanel;
pub use viewport::ViewportPanel;
