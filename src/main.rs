use egui::ViewportBuilder;
use hybrid_esp::app::LithoApp;

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("Lithowise"),
        ..Default::default()
    };
    eframe::run_native(
        "Lithowise",
        options,
        Box::new(|cc| Ok(Box::new(LithoApp::new(cc)))),
    )
}
