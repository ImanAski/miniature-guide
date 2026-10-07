//! Geometry toolbox: hatching, boolean operators, shape transforms, layers.
//!
//! Operates on the shared selection (shift/ctrl-click in the viewport to
//! build one). Boolean results replace their operands; hatch output is
//! appended and selected so it can be removed with one Delete.

use egui::{ComboBox, DragValue, RichText, Slider, Ui};

use crate::core::boolops::{BoolOp, boolean_op};
use crate::core::edit::{
    delete_layer, layer_counts, mirror_horizontal, mirror_vertical, rename_layer, rotate,
    selection_center, set_layer, shapes_on_layer, translate,
};
use crate::core::geo::HatchPattern;
use crate::core::hatch::{HatchOptions, hatch};
use crate::ui::{Panel, PanelCtx, Slot};

/// Hatch pattern picker (mirrors [`HatchPattern`] without the embedded
/// spacing/angle fields, which this panel edits separately).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum HatchKind {
    #[default]
    Lines,
    Cross,
    Solid,
}

impl HatchKind {
    fn label(self) -> &'static str {
        match self {
            HatchKind::Lines => "Lines",
            HatchKind::Cross => "Cross hatch",
            HatchKind::Solid => "Solid fill",
        }
    }

    const ALL: [HatchKind; 3] = [HatchKind::Lines, HatchKind::Cross, HatchKind::Solid];
}

#[derive(Default)]
pub struct GeometryPanel {
    pattern: HatchKind,
    spacing: f64,
    angle_deg: f64,
    layer_input: String,
    offset: [f64; 2],
    rotate_deg: f64,
}

impl GeometryPanel {
    pub fn new() -> Self {
        GeometryPanel {
            pattern: HatchKind::Lines,
            spacing: 0.5,
            angle_deg: 0.0,
            layer_input: "hatch".to_string(),
            offset: [1.0, 1.0],
            rotate_deg: 90.0,
        }
    }
}

impl Panel for GeometryPanel {
    fn id(&self) -> &'static str {
        "geometry"
    }

    fn title(&self) -> &'static str {
        "Geometry"
    }

    fn slot(&self) -> Slot {
        Slot::Right
    }

    fn order(&self) -> i32 {
        0
    }

    fn size(&self) -> f32 {
        320.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(
            RichText::new(format!("{} selected", ctx.selection.len()))
                .small()
                .weak(),
        );
        ui.add_space(4.0);

        self.hatch_section(ui, ctx);
        ui.add_space(6.0);
        self.boolean_section(ui, ctx);
        ui.add_space(6.0);
        self.shape_section(ui, ctx);
        ui.add_space(6.0);
        self.layer_section(ui, ctx);
    }
}

impl GeometryPanel {
    // ── Hatch ────────────────────────────────────────────────────────

    fn hatch_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Hatch").strong());
        let solid = self.pattern == HatchKind::Solid;

        egui::Grid::new("geometry.hatch")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Pattern");
                ComboBox::from_id_salt("geometry.hatch.pattern")
                    .selected_text(self.pattern.label())
                    .show_ui(ui, |ui| {
                        for k in HatchKind::ALL {
                            ui.selectable_value(&mut self.pattern, k, k.label());
                        }
                    });
                ui.end_row();

                ui.label("Spacing");
                ui.add_enabled(
                    !solid,
                    Slider::new(&mut self.spacing, 0.01..=20.0)
                        .suffix(" mm")
                        .clamping(egui::SliderClamping::Edits),
                );
                ui.end_row();

                ui.label("Angle");
                ui.add(
                    Slider::new(&mut self.angle_deg, 0.0..=180.0)
                        .suffix("°")
                        .clamping(egui::SliderClamping::Edits),
                );
                ui.end_row();

                ui.label("Layer");
                ui.text_edit_singleline(&mut self.layer_input);
                ui.end_row();
            });

        if solid {
            ui.label(
                RichText::new("solid fill uses the dense hatch spacing from config")
                    .small()
                    .weak(),
            );
        }

        let closed = self.count_selected_closed(ctx);
        let enabled = closed > 0;
        let label = if ctx.selection.is_empty() {
            "Hatch selection".to_string()
        } else {
            format!("Hatch selection ({closed})")
        };
        if ui
            .add_enabled(enabled, egui::Button::new(label))
            .on_disabled_hover_text("select at least one closed shape")
            .clicked()
        {
            self.apply_hatch(ctx);
        }
    }

    fn count_selected_closed(&self, ctx: &PanelCtx<'_>) -> usize {
        ctx.selection
            .iter()
            .filter(|&&i| ctx.shapes.get(i).is_some_and(|s| s.closed))
            .count()
    }

    fn apply_hatch(&mut self, ctx: &mut PanelCtx<'_>) {
        let tolerance = ctx.config.processing.approximation_tolerance;
        let solid_spacing = ctx.config.processing.hatch_spacing;
        let name = self.layer_input.trim().to_string();
        let layer = if name.is_empty() { None } else { Some(name) };
        let pattern = match self.pattern {
            HatchKind::Lines => HatchPattern::Lines {
                spacing: self.spacing,
                angle: self.angle_deg,
            },
            HatchKind::Cross => HatchPattern::CrossHatch {
                spacing: self.spacing,
                angle: self.angle_deg,
            },
            HatchKind::Solid => HatchPattern::Solid,
        };
        let opts = HatchOptions {
            pattern,
            spacing: self.spacing,
            angle_deg: self.angle_deg,
            tolerance,
            solid_spacing,
            layer,
        };

        let mut results = Vec::new();
        let mut failed = 0usize;
        let mut first_err = String::new();
        for &i in &ctx.selection.clone() {
            let Some(shape) = ctx.shapes.get(i) else {
                continue;
            };
            match hatch(shape, &opts) {
                Ok(h) => results.push(h),
                Err(e) => {
                    failed += 1;
                    if first_err.is_empty() {
                        first_err = e;
                    }
                }
            }
        }

        if results.is_empty() {
            let msg = if first_err.is_empty() {
                "nothing to hatch".to_string()
            } else {
                format!("hatch failed: {first_err}")
            };
            ctx.note(log::Level::Error, "hatch", msg);
            return;
        }

        let start = ctx.shapes.len();
        let made = results.len();
        ctx.shapes.extend(results);
        *ctx.selection = (start..start + made).collect();
        let msg = if failed == 0 {
            format!("hatched {made} shapes")
        } else {
            format!("hatched {made} shapes, {failed} skipped ({first_err})")
        };
        ctx.note(log::Level::Info, "hatch", msg);
    }

    // ── Boolean ──────────────────────────────────────────────────────

    fn boolean_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Boolean").strong());
        let enough = ctx.selection.len() >= 2;
        ui.horizontal(|ui| {
            for op in BoolOp::ALL {
                if ui
                    .add_enabled(enough, egui::Button::new(op.label()))
                    .on_disabled_hover_text("select two or more shapes (shift-click in viewport)")
                    .clicked()
                {
                    self.apply_boolean(ctx, op);
                }
            }
        });
    }

    fn apply_boolean(&mut self, ctx: &mut PanelCtx<'_>, op: BoolOp) {
        let tolerance = ctx.config.processing.approximation_tolerance;
        let mut idxs = ctx.selection.clone();
        idxs.sort_unstable();
        idxs.dedup();
        let operands: Vec<crate::core::geo::Shape> = idxs
            .iter()
            .filter_map(|&i| ctx.shapes.get(i).cloned())
            .collect();

        match boolean_op(&operands, op, tolerance) {
            Ok(results) => {
                for &i in idxs.iter().rev() {
                    if i < ctx.shapes.len() {
                        ctx.shapes.remove(i);
                    }
                }
                let start = ctx.shapes.len();
                let n = results.len();
                ctx.shapes.extend(results);
                *ctx.selection = (start..start + n).collect();
                ctx.note(
                    log::Level::Info,
                    "geometry",
                    format!(
                        "{} of {} shapes → {n} contour{}",
                        op.label(),
                        idxs.len(),
                        if n == 1 { "" } else { "s" }
                    ),
                );
            }
            Err(e) => ctx.note(
                log::Level::Error,
                "geometry",
                format!("{} failed: {e}", op.label()),
            ),
        }
    }

    // ── Shape transforms ─────────────────────────────────────────────

    fn shape_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Shape").strong());
        let enabled = !ctx.selection.is_empty();
        let no_selection = "select shapes first (shift-click in viewport)";

        ui.horizontal(|ui| {
            ui.label("offset");
            ui.add(
                DragValue::new(&mut self.offset[0])
                    .speed(0.05)
                    .suffix(" mm"),
            );
            ui.add(
                DragValue::new(&mut self.offset[1])
                    .speed(0.05)
                    .suffix(" mm"),
            );
            if ui
                .add_enabled(enabled, egui::Button::new("Apply"))
                .on_disabled_hover_text(no_selection)
                .clicked()
            {
                self.apply_translate(ctx);
            }
            if ui
                .add_enabled(enabled, egui::Button::new("Duplicate"))
                .on_disabled_hover_text(no_selection)
                .clicked()
            {
                self.apply_duplicate(ctx);
            }
        });

        ui.horizontal(|ui| {
            if ui
                .add_enabled(enabled, egui::Button::new("Mirror X"))
                .on_disabled_hover_text(no_selection)
                .clicked()
            {
                self.apply_mirror(ctx, false);
            }
            if ui
                .add_enabled(enabled, egui::Button::new("Mirror Y"))
                .on_disabled_hover_text(no_selection)
                .clicked()
            {
                self.apply_mirror(ctx, true);
            }
            ui.add(
                DragValue::new(&mut self.rotate_deg)
                    .speed(1.0)
                    .range(0.0..=360.0)
                    .suffix("°"),
            );
            if ui
                .add_enabled(enabled, egui::Button::new("Rotate"))
                .on_disabled_hover_text(no_selection)
                .clicked()
            {
                self.apply_rotate(ctx);
            }
        });
    }

    /// Axis-aligned centre of the current selection.
    fn center(&self, ctx: &PanelCtx<'_>) -> crate::core::geo::Point {
        selection_center(ctx.shapes, ctx.selection)
    }

    fn apply_translate(&mut self, ctx: &mut PanelCtx<'_>) {
        let (dx, dy) = (self.offset[0], self.offset[1]);
        let n = ctx.selection.len();
        for &i in &ctx.selection.clone() {
            if let Some(s) = ctx.shapes.get_mut(i) {
                *s = translate(s, dx, dy);
            }
        }
        ctx.note(
            log::Level::Info,
            "geometry",
            format!("offset {n} shapes by ({dx}, {dy})"),
        );
    }

    fn apply_duplicate(&mut self, ctx: &mut PanelCtx<'_>) {
        let (dx, dy) = (self.offset[0], self.offset[1]);
        let copies: Vec<_> = ctx
            .selection
            .iter()
            .filter_map(|&i| ctx.shapes.get(i))
            .map(|s| translate(s, dx, dy))
            .collect();
        if copies.is_empty() {
            return;
        }
        let start = ctx.shapes.len();
        let n = copies.len();
        ctx.shapes.extend(copies);
        *ctx.selection = (start..start + n).collect();
        ctx.note(
            log::Level::Info,
            "geometry",
            format!("duplicated {n} shapes"),
        );
    }

    fn apply_mirror(&mut self, ctx: &mut PanelCtx<'_>, horizontal: bool) {
        let center = self.center(ctx);
        let axis = if horizontal { center.y } else { center.x };
        let n = ctx.selection.len();
        for &i in &ctx.selection.clone() {
            if let Some(s) = ctx.shapes.get_mut(i) {
                *s = if horizontal {
                    mirror_horizontal(s, axis)
                } else {
                    mirror_vertical(s, axis)
                };
            }
        }
        let axis_name = if horizontal { "Y" } else { "X" };
        ctx.note(
            log::Level::Info,
            "geometry",
            format!("mirrored {n} shapes on {axis_name}"),
        );
    }

    fn apply_rotate(&mut self, ctx: &mut PanelCtx<'_>) {
        let center = self.center(ctx);
        let rad = self.rotate_deg.to_radians();
        let n = ctx.selection.len();
        for &i in &ctx.selection.clone() {
            if let Some(s) = ctx.shapes.get_mut(i) {
                *s = rotate(s, center, rad);
            }
        }
        ctx.note(
            log::Level::Info,
            "geometry",
            format!("rotated {n} shapes by {}°", self.rotate_deg),
        );
    }

    // ── Layers ───────────────────────────────────────────────────────

    fn layer_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Layers").strong());

        ui.horizontal(|ui| {
            ui.text_edit_singleline(&mut self.layer_input);
            let has_selection = !ctx.selection.is_empty();
            if ui
                .add_enabled(has_selection, egui::Button::new("Assign"))
                .on_disabled_hover_text("select shapes first")
                .clicked()
            {
                let name = self.layer_input.trim().to_string();
                let layer = if name.is_empty() {
                    None
                } else {
                    Some(name.clone())
                };
                let n = ctx.selection.len();
                let selection = ctx.selection.clone();
                set_layer(ctx.shapes, &selection, layer.clone());
                match &layer {
                    Some(l) => ctx.note(
                        log::Level::Info,
                        "geometry",
                        format!("assigned {n} shapes to layer '{l}'"),
                    ),
                    None => ctx.note(
                        log::Level::Info,
                        "geometry",
                        format!("removed layer from {n} shapes"),
                    ),
                }
            }
        });

        for (name, count) in layer_counts(ctx.shapes) {
            ui.horizontal(|ui| {
                match &name {
                    Some(layer) => {
                        let mut hidden = ctx.hidden_layers.contains(layer);
                        if ui.checkbox(&mut hidden, "").changed() {
                            if hidden {
                                ctx.hidden_layers.insert(layer.clone());
                            } else {
                                ctx.hidden_layers.remove(layer);
                            }
                        }
                        ui.label(format!("{layer} ({count})"));
                    }
                    None => {
                        ui.add_space(22.0);
                        ui.label(format!("(none) ({count})"));
                    }
                }
                if ui
                    .small_button("Sel")
                    .on_hover_text("select every shape on this layer")
                    .clicked()
                {
                    *ctx.selection = shapes_on_layer(ctx.shapes, &name);
                }
                if let Some(layer) = &name {
                    if ui
                        .small_button("Ren")
                        .on_hover_text("rename to the name typed above")
                        .clicked()
                    {
                        let to = self.layer_input.trim().to_string();
                        if to.is_empty() {
                            ctx.note(log::Level::Warn, "geometry", "type a new layer name first");
                        } else {
                            let n = rename_layer(ctx.shapes, layer, &to);
                            ctx.note(
                                log::Level::Info,
                                "geometry",
                                format!("renamed layer '{layer}' → '{to}' ({n} shapes)"),
                            );
                        }
                    }
                    if ui
                        .small_button("×")
                        .on_hover_text("delete this layer and its shapes")
                        .clicked()
                    {
                        let removed = delete_layer(ctx.shapes, layer);
                        ctx.hidden_layers.remove(layer);
                        ctx.selection.clear();
                        ctx.note(
                            log::Level::Warn,
                            "geometry",
                            format!("deleted layer '{layer}' ({removed} shapes)"),
                        );
                    }
                }
            });
        }
    }
}
