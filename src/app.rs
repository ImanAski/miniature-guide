use crate::logger;
use crate::{config::AppConfig, core::geo::Shape};
use std::path::PathBuf;

pub struct LithoApp {
    shapes: Vec<Shape>,
    status: AppStatus,
    config: AppConfig,
    log_messages: Vec<String>,
}

impl LithoApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let log_dir = PathBuf::from("./logs");
        let _logger = logger::Logger::init(log_dir);

        let config_path = AppConfig::default_path();
        let config = AppConfig::load(&config_path).unwrap_or_default();

        LithoApp {
            shapes: Vec::new(),
            status: AppStatus::Ready,
            config: config,
            log_messages: vec!["Application Started".into()],
        }
    }
}

impl eframe::App for LithoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("LithoApp");
            ui.add_space(8.0);
        });
    }
}

pub enum AppStatus {
    Ready,
    Processing,
    Running,
    Error(String),
    Done,
}
