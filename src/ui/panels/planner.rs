//! Planner widget — pre-submit plan report.
//!
//! Computes a [`Plan`] over the visible geometry, shows the per-mode cost
//! comparison and the decision reason for every tile, overlays it in the
//! viewport, and exports the report as text, CSV, or a self-contained SVG.
//! Nothing is written to the machine from here: this is the review step that
//! happens *before* submitting a job.

use egui::{Color32, RichText, ScrollArea, Ui};

use crate::core::geo::Shape;
use crate::hybrid::{WriteMode, diagnostics_csv, diagnostics_report, plan, svg_report};
use crate::ui::{Panel, PanelCtx, Slot};

/// Mode colours shared by the panel, the viewport overlay and the SVG export.
pub fn mode_color(mode: WriteMode) -> Color32 {
    match mode {
        WriteMode::Vector => Color32::from_rgb(86, 156, 214),
        WriteMode::Raster => Color32::from_rgb(206, 145, 120),
        WriteMode::Hybrid => Color32::from_rgb(106, 153, 85),
    }
}

pub struct PlannerPanel {
    /// Per-tile detail rows expanded.
    expanded: bool,
    /// Export format offered by the save button.
    format: ExportFormat,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExportFormat {
    Text,
    Csv,
    Svg,
}

impl ExportFormat {
    fn label(self) -> &'static str {
        match self {
            ExportFormat::Text => "report (.txt)",
            ExportFormat::Csv => "table (.csv)",
            ExportFormat::Svg => "drawing (.svg)",
        }
    }

    fn ext(self) -> &'static str {
        match self {
            ExportFormat::Text => "txt",
            ExportFormat::Csv => "csv",
            ExportFormat::Svg => "svg",
        }
    }
}

impl Default for PlannerPanel {
    fn default() -> Self {
        PlannerPanel {
            expanded: false,
            format: ExportFormat::Text,
        }
    }
}

impl Panel for PlannerPanel {
    fn id(&self) -> &'static str {
        "planner"
    }

    fn title(&self) -> &'static str {
        "Plan"
    }

    fn slot(&self) -> Slot {
        Slot::Right
    }

    fn order(&self) -> i32 {
        10 // after Geometry (0)
    }

    fn size(&self) -> f32 {
        320.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        self.actions(ui, ctx);
        ui.separator();

        if ctx.plan.is_none() {
            ui.label(
                RichText::new("No plan yet.\n\nPress Plan to analyse the visible geometry and preview what each tile will do — raster, vector, or hybrid — before submitting.")
                    .small()
                    .weak(),
            );
            return;
        }
        self.summary(ui, ctx);
        ui.separator();
        self.tile_list(ui, ctx);
    }
}

impl PlannerPanel {
    fn actions(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.horizontal(|ui| {
            if ui.button("Plan").clicked() {
                self.compute(ctx);
            }
            let has_plan = ctx.plan.is_some();
            if ui
                .add_enabled(has_plan, egui::Button::new("Clear"))
                .clicked()
            {
                *ctx.plan = None;
                ctx.note(log::Level::Info, "planner", "plan cleared");
            }
            ui.separator();
            ui.checkbox(&mut self.expanded, "details");
        });

        let has_plan = ctx.plan.is_some();
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("planner.export.format")
                .width(130.0)
                .selected_text(self.format.label())
                .show_ui(ui, |ui| {
                    for f in [ExportFormat::Text, ExportFormat::Csv, ExportFormat::Svg] {
                        ui.selectable_value(&mut self.format, f, f.label());
                    }
                });
            if ui
                .add_enabled(has_plan, egui::Button::new("Export…"))
                .on_disabled_hover_text("create a plan first")
                .clicked()
            {
                self.export(ctx);
            }
        });
    }

    fn compute(&mut self, ctx: &mut PanelCtx<'_>) {
        if let Err(e) = ctx.config.planner.validate() {
            ctx.note(log::Level::Error, "planner", format!("config: {e}"));
            return;
        }
        // The plan describes what will actually be written: hidden layers are
        // excluded, exactly like a follow run would exclude them.
        let visible: Vec<Shape> = ctx
            .shapes
            .iter()
            .filter(|s| {
                !s.layer
                    .as_ref()
                    .is_some_and(|l| ctx.hidden_layers.contains(l))
            })
            .cloned()
            .collect();
        if visible.is_empty() {
            ctx.note(
                log::Level::Warn,
                "planner",
                "nothing to plan — no visible geometry",
            );
            return;
        }
        let p = plan(&visible, &ctx.config.planner);
        ctx.note(log::Level::Info, "planner", format!("plan: {}", p.summary()));
        *ctx.plan = Some(p);
    }

    fn export(&self, ctx: &mut PanelCtx<'_>) {
        let Some(p) = ctx.plan.as_ref() else {
            return;
        };
        let name = format!("write-plan.{}", self.format.ext());
        let Some(path) = rfd::FileDialog::new()
            .set_directory(&ctx.config.file.save_dir)
            .set_file_name(name)
            .save_file()
        else {
            return;
        };
        // SVG embeds the input geometry; text/CSV describe the plan alone.
        let content = match self.format {
            ExportFormat::Text => diagnostics_report(p),
            ExportFormat::Csv => diagnostics_csv(p),
            ExportFormat::Svg => svg_report(p, &ctx.shapes),
        };
        match std::fs::write(&path, content) {
            Ok(()) => ctx.note(
                log::Level::Info,
                "planner",
                format!("exported {}", path.display()),
            ),
            Err(e) => ctx.note(
                log::Level::Error,
                "planner",
                format!("export failed: {e}"),
            ),
        }
    }

    fn summary(&self, ui: &mut Ui, ctx: &PanelCtx<'_>) {
        let Some(p) = ctx.plan.as_ref() else {
            return;
        };
        let [v, r, h] = p.mode_counts();
        egui::Grid::new("planner.summary")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Tiles");
                ui.label(format!("{}", p.tiles.len()));
                ui.end_row();
                ui.label("Mode mix");
                ui.horizontal(|ui| {
                    ui.colored_label(mode_color(WriteMode::Vector), format!("{v} vector"));
                    ui.colored_label(mode_color(WriteMode::Raster), format!("{r} raster"));
                    ui.colored_label(mode_color(WriteMode::Hybrid), format!("{h} hybrid"));
                });
                ui.end_row();
                ui.label("Est. write time");
                ui.label(format!("{:.3} s", p.total_time));
                ui.end_row();
                ui.label("Total cost");
                ui.label(format!("{:.3}", p.total_cost));
                ui.end_row();
                ui.label("Tile size");
                ui.label(format!("{:.3} mm", p.tile_size_mm));
                ui.end_row();
            });
        ui.label(
            RichText::new("overlay on — colours match the viewport")
                .small()
                .weak(),
        );
    }

    fn tile_list(&mut self, ui: &mut Ui, ctx: &PanelCtx<'_>) {
        let Some(p) = ctx.plan.as_ref() else {
            return;
        };
        ScrollArea::vertical()
            .id_salt("planner.tiles")
            .auto_shrink([false, false])
            .max_height(260.0)
            .show(ui, |ui| {
                for (i, tp) in p.tiles.iter().enumerate() {
                    let (tx, ty) = tp.tile.index;
                    let sel = tp.selected();
                    ui.horizontal(|ui| {
                        ui.colored_label(mode_color(tp.mode), tp.mode.label());
                        ui.label(
                            RichText::new(format!("({tx},{ty})")).small().weak(),
                        );
                        ui.label(format!("{:.3} s", sel.write_time));
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                ui.label(
                                    RichText::new(format!("cost {:.2}", sel.cost))
                                        .small()
                                        .weak(),
                                );
                            },
                        );
                    });
                    if self.expanded {
                        egui::CollapsingHeader::new(
                            RichText::new(format!("tile ({tx},{ty}) detail")).small(),
                        )
                        .id_salt(format!("planner.tile.{i}"))
                        .show(ui, |ui| {
                            self.tile_detail(ui, ctx, i);
                        });
                    }
                    ui.separator();
                }
            });
    }

    fn tile_detail(&self, ui: &mut Ui, ctx: &PanelCtx<'_>, index: usize) {
        // Re-borrow through the ctx each call: the plan may be replaced by a
        // Clear between rows.
        let Some(p) = ctx.plan.as_ref() else {
            return;
        };
        let Some(tp) = p.tiles.get(index) else {
            return;
        };
        let a = tp.tile.area() * 1e6;
        let per = tp.tile.perimeter() * 1e3;
        ui.label(
            RichText::new(format!(
                "area {a:.1} um² · perim {per:.1} um · fill {:.3} · {} rings",
                tp.tile.fill_factor(),
                tp.tile.rings.len()
            ))
            .small()
            .weak(),
        );
        for c in &tp.costs {
            let colour = if c.cost.is_finite() {
                mode_color(c.mode)
            } else {
                ui.visuals().error_fg_color
            };
            ui.horizontal(|ui| {
                ui.colored_label(colour, c.mode.label());
                if c.cost.is_finite() {
                    ui.label(
                        RichText::new(format!(
                            "{:.3} s · cost {:.3} · jumps {} · accel {}",
                            c.write_time, c.cost, c.jump_count, c.acceleration_events
                        ))
                        .small(),
                    );
                } else {
                    ui.label(
                        RichText::new(format!("rejected — {}", c.reason))
                            .small()
                            .color(ui.visuals().error_fg_color),
                    );
                }
            });
        }
        ui.label(
            RichText::new(format!("decision: {}", tp.selected().reason))
                .small()
                .italics(),
        );
    }
}
