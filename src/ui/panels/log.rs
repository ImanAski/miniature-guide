//! Log widget — the app's in-memory event stream.

use egui::{RichText, ScrollArea, Ui};
use log::Level;

use crate::ui::{Panel, PanelCtx, Slot};

const LEVELS: [Level; 4] = [Level::Error, Level::Warn, Level::Info, Level::Debug];

#[derive(Debug, Default)]
pub struct LogPanel {
    follow: bool,
}

impl Panel for LogPanel {
    fn id(&self) -> &'static str {
        "log"
    }

    fn title(&self) -> &'static str {
        "Log"
    }

    fn slot(&self) -> Slot {
        Slot::Bottom
    }

    fn order(&self) -> i32 {
        10
    }

    fn size(&self) -> f32 {
        180.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("log.level")
                .width(90.0)
                .selected_text(level_label(ctx.log.min_level))
                .show_ui(ui, |ui| {
                    for level in LEVELS {
                        ui.selectable_value(&mut ctx.log.min_level, level, level_label(level));
                    }
                });
            ui.checkbox(&mut self.follow, "follow");
            if ui.button("Clear").clicked() {
                ctx.log.clear();
            }
            if ui.button("Clear trail").clicked() {
                ctx.motion.clear_trail();
            }
            ui.separator();
            if let Some(last) = ctx.log.last() {
                ui.label(RichText::new(format!("{} — {}", last.time, last.message)).small());
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    RichText::new(format!(
                        "{}/{} shown",
                        ctx.log.visible().count(),
                        ctx.log.len()
                    ))
                    .small()
                    .weak(),
                );
            });
        });
        ui.separator();

        ScrollArea::vertical()
            .stick_to_bottom(self.follow)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for e in ctx.log.visible() {
                    let color = level_color(ui, e.level);
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(&e.time).small().weak().monospace());
                        ui.label(
                            RichText::new(format!("{:<5}", level_label(e.level))).color(color),
                        );
                        ui.label(RichText::new(e.module).small().weak());
                        ui.label(RichText::new(&e.message));
                    });
                }
            });
    }
}

fn level_label(level: Level) -> &'static str {
    match level {
        Level::Error => "Error",
        Level::Warn => "Warn",
        Level::Info => "Info",
        Level::Debug => "Debug",
        Level::Trace => "Trace",
    }
}

fn level_color(ui: &Ui, level: Level) -> egui::Color32 {
    match level {
        Level::Error => ui.visuals().error_fg_color,
        Level::Warn => ui.visuals().warn_fg_color,
        _ => ui.visuals().text_color(),
    }
}
