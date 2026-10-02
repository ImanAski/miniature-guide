//! Status strip — app state, drive state and a position readout.

use egui::{RichText, Ui};

use crate::core::geo::fmt_f;
use crate::core::motion::DriveMode;
use crate::ui::{Panel, PanelCtx, Slot};

#[derive(Debug, Default)]
pub struct StatusPanel;

impl Panel for StatusPanel {
    fn id(&self) -> &'static str {
        "status"
    }

    fn title(&self) -> &'static str {
        "Status"
    }

    fn slot(&self) -> Slot {
        Slot::Bottom
    }

    fn order(&self) -> i32 {
        100
    }

    fn size(&self) -> f32 {
        26.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.horizontal(|ui| {
            let status_color = match &*ctx.status {
                crate::app::AppStatus::Error(_) => ui.visuals().error_fg_color,
                crate::app::AppStatus::Running | crate::app::AppStatus::Processing => {
                    ui.visuals().selection.stroke.color
                }
                _ => ui.visuals().text_color(),
            };
            ui.colored_label(status_color, ctx.status.to_string());

            ui.separator();

            let drive = match ctx.motion.mode {
                DriveMode::Simulated => ui.visuals().weak_text_color(),
                DriveMode::Hardware => ui.visuals().selection.stroke.color,
            };
            ui.colored_label(drive, ctx.motion.mode.label());
            ui.label(if ctx.motion.is_moving() {
                "moving"
            } else {
                "idle"
            });
            if !ctx.motion.status.is_home {
                ui.colored_label(ui.visuals().warn_fg_color, "not homed");
            }

            ui.separator();

            let pos = ctx.motion.pos();
            ui.label(
                RichText::new(format!(
                    "X {}  Y {}  Z {}",
                    fmt_f(pos.x),
                    fmt_f(pos.y),
                    fmt_f(pos.z)
                ))
                .monospace(),
            );

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if let Some(err) = ctx.motion.link_error.as_ref() {
                    ui.colored_label(ui.visuals().error_fg_color, err);
                }
                ui.separator();
                ui.label(
                    RichText::new(format!("{} shapes", ctx.shapes.len()))
                        .small()
                        .weak(),
                );
            });
        });
    }
}
