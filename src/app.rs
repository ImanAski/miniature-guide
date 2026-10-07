use std::collections::HashSet;
use std::path::PathBuf;

use crate::config::AppConfig;
use crate::core::geo::Shape;
use crate::core::motion::MotionHub;
use crate::core::path::{PathEvent, PathRunner};
use crate::logger::Logger;
use crate::ui::buffer::LogBuffer;
use crate::ui::{PanelCtx, PanelRegistry};

pub struct LithoApp {
    logger: Logger,
    panels: PanelRegistry,
    config: AppConfig,
    motion: MotionHub,
    runner: PathRunner,
    log: LogBuffer,
    status: AppStatus,
    shapes: Vec<Shape>,
    /// Indices into `shapes`, shared by viewport and geometry tools.
    selection: Vec<usize>,
    /// Layer names hidden in the viewport.
    hidden_layers: HashSet<String>,
    /// Last computed write plan (planner panel sets, viewport overlays).
    plan: Option<crate::hybrid::Plan>,
}

impl LithoApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let logger = Logger::init(PathBuf::from("./logs"));

        let config = AppConfig::load(&AppConfig::default_path()).unwrap_or_default();
        let motion = MotionHub::new(
            config.controller.default_velocity,
            config.controller.default_acceleration,
        );

        let mut log = LogBuffer::default();
        log.info("app", "ready");

        let app = LithoApp {
            logger,
            panels: PanelRegistry::standard(),
            config,
            motion,
            runner: PathRunner::new(),
            log,
            status: AppStatus::Ready,
            shapes: Vec::new(),
            selection: Vec::new(),
            hidden_layers: HashSet::new(),
            plan: None,
        };

        cc.egui_ctx.set_theme(match app.config.ui.theme {
            crate::config::UiTheme::Dark => egui::Theme::Dark,
            crate::config::UiTheme::Light => egui::Theme::Light,
        });
        app
    }

    pub fn logger(&self) -> &Logger {
        &self.logger
    }

    pub fn registry(&self) -> &PanelRegistry {
        &self.panels
    }

    fn save_config(&mut self) {
        let path = AppConfig::default_path();
        match self.config.save(&path) {
            Ok(()) => self
                .log
                .info("config", format!("saved to {}", path.display())),
            Err(e) => self.log.error("config", e),
        }
    }
}

impl eframe::App for LithoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.motion.tick();
        match self.runner.tick(&mut self.motion) {
            Some(PathEvent::Finished { steps }) => {
                self.log
                    .info("path", format!("path follow finished ({steps} waypoints)"));
                self.status = AppStatus::Done;
            }
            Some(PathEvent::Failed(e)) => {
                self.log.error("path", format!("path follow failed: {e}"));
                self.status = AppStatus::Error(e);
            }
            None => {}
        }
        if self.motion.is_moving() || self.runner.is_running() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        let LithoApp {
            panels,
            config,
            motion,
            runner,
            log,
            status,
            shapes,
            selection,
            hidden_layers,
            plan,
            ..
        } = self;

        panels.render(
            ctx,
            &mut PanelCtx {
                egui: ctx,
                config,
                motion,
                status,
                log,
                shapes,
                runner,
                selection,
                hidden_layers,
                plan,
            },
        );
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        if self.motion.is_linked() {
            self.motion.disconnect();
        }
        self.save_config();
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub enum AppStatus {
    #[default]
    Ready,
    Processing,
    Running,
    Error(String),
    Done,
}

impl std::fmt::Display for AppStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AppStatus::Ready => write!(f, "ready"),
            AppStatus::Processing => write!(f, "processing"),
            AppStatus::Running => write!(f, "running"),
            AppStatus::Error(e) => write!(f, "error: {e}"),
            AppStatus::Done => write!(f, "done"),
        }
    }
}
