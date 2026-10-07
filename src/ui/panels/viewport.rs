//! Viewport widget — 2D canvas for loaded geometry, with pan/zoom and picking.

use egui::{Color32, RichText, Sense, Stroke, pos2, vec2};

use crate::app::AppStatus;
use crate::core::geo::{PathElement, Point, Rect, Shape, fmt_f};
use crate::ui::{Panel, PanelCtx, Slot, toggle_selection};

const MIN_ZOOM: f64 = 0.01;
const MAX_ZOOM: f64 = 400.0;
/// Click tolerance: how near (px) the pointer must be to a shape's geometry
/// for it to count as a hit.
const HIT_PX: f64 = 6.0;

pub struct ViewportPanel {
    /// World point held at the canvas centre (mm).
    pan: Point,
    /// Screen pixels per millimetre.
    zoom: f64,
    show_grid: bool,
    show_origin: bool,
    fit_pending: bool,
}

impl Default for ViewportPanel {
    fn default() -> Self {
        ViewportPanel {
            pan: Point::ORIGIN,
            zoom: 2.0,
            show_grid: true,
            show_origin: true,
            fit_pending: true,
        }
    }
}

impl ViewportPanel {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Panel for ViewportPanel {
    fn id(&self) -> &'static str {
        "viewport"
    }

    fn title(&self) -> &'static str {
        "Viewport"
    }

    fn slot(&self) -> Slot {
        Slot::Center
    }

    fn size(&self) -> f32 {
        400.0
    }

    fn show(&mut self, ui: &mut egui::Ui, ctx: &mut PanelCtx<'_>) {
        self.toolbar(ui, ctx);
        ui.separator();

        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        self.handle_input(ui, &resp, rect, ctx);

        let accent = ui.visuals().selection.stroke.color;
        let grid_color = ui.visuals().weak_text_color();

        if self.show_grid {
            self.draw_grid(&painter, rect, grid_color, ctx);
        }
        if self.show_origin {
            let o = self.to_screen(Point::ORIGIN, rect);
            painter.line_segment(
                [pos2(rect.left(), o.y), pos2(rect.right(), o.y)],
                Stroke::new(1.0_f32, ui.visuals().warn_fg_color),
            );
            painter.line_segment(
                [pos2(o.x, rect.top()), pos2(o.x, rect.bottom())],
                Stroke::new(1.0_f32, ui.visuals().warn_fg_color),
            );
        }

        let tolerance = ctx.config.processing.approximation_tolerance.max(1e-4);
        for (i, shape) in ctx.shapes.iter().enumerate() {
            if is_hidden(shape, ctx) {
                continue;
            }
            let selected = ctx.selection.contains(&i);
            let color = if selected {
                accent
            } else {
                ui.visuals().text_color()
            };
            for polyline in polylines(shape, tolerance) {
                let pts: Vec<_> = polyline.iter().map(|p| self.to_screen(*p, rect)).collect();
                if selected && shape.closed && pts.len() > 2 {
                    painter.add(egui::Shape::closed_line(pts, Stroke::new(2.0_f32, color)));
                } else {
                    painter.add(egui::Shape::line(pts, Stroke::new(1.0_f32, color)));
                }
            }
            if let Some(b) = shape.bounds() {
                let bb = egui::Rect::from_two_pos(
                    self.to_screen(b.min, rect),
                    self.to_screen(b.max, rect),
                );
                painter.rect_stroke(
                    bb,
                    0.0,
                    Stroke::new(if selected { 1.5_f32 } else { 0.5_f32 }, grid_color),
                    egui::StrokeKind::Middle,
                );
            }
        }

        if self.fit_pending && !ctx.shapes.is_empty() {
            self.fit_to_shapes(ctx, rect.size());
        }

        let info = self.info_line(ctx);
        painter.text(
            rect.left_top() + vec2(8.0, 6.0),
            egui::Align2::LEFT_TOP,
            info,
            egui::TextStyle::Small.resolve(ui.style()),
            grid_color,
        );
    }
}

impl ViewportPanel {
    fn toolbar(&mut self, ui: &mut egui::Ui, ctx: &mut PanelCtx<'_>) {
        ui.horizontal(|ui| {
            if ui.button("Open…").clicked() {
                self.open(ctx);
            }
            if ui.button("Add demo").clicked() {
                ctx.shapes.extend(demo_shapes());
                self.fit_pending = true;
                ctx.note(log::Level::Info, "viewport", "added demo geometry");
            }
            if ui.button("Fit").clicked() {
                self.fit_pending = true;
            }
            ui.separator();
            self.follow_section(ui, ctx);
            ui.separator();
            ui.checkbox(&mut self.show_grid, "grid");
            ui.checkbox(&mut self.show_origin, "origin");
            ui.separator();
            let delete_enabled = !ctx.selection.is_empty();
            if ui
                .add_enabled(delete_enabled, egui::Button::new("Delete"))
                .on_hover_text("remove every selected shape")
                .clicked()
            {
                self.delete_selection(ctx);
            }
            ui.separator();
            ui.label(
                RichText::new(format!("zoom {:.3} px/mm", self.zoom))
                    .small()
                    .weak(),
            );
            ui.label(
                RichText::new(format!(
                    "centre ({}, {})",
                    fmt_f(self.pan.x),
                    fmt_f(self.pan.y)
                ))
                .small()
                .weak(),
            );
        });
    }

    fn follow_section(&mut self, ui: &mut egui::Ui, ctx: &mut PanelCtx<'_>) {
        if ctx.runner.is_running() {
            let (done, total) = ctx.runner.progress();
            ui.label(RichText::new(format!("follow {done}/{total}")).small());
            if ui.button("Stop path").clicked() {
                ctx.runner.stop(ctx.motion);
                *ctx.status = AppStatus::Ready;
                ctx.note(log::Level::Info, "viewport", "path follow stopped");
            }
            return;
        }
        let has_geometry = !ctx.shapes.is_empty();
        if ui
            .add_enabled(has_geometry, egui::Button::new("Follow path"))
            .on_disabled_hover_text("load a DXF or add demo geometry first")
            .clicked()
        {
            self.follow(ctx);
        }
    }

    fn follow(&mut self, ctx: &mut PanelCtx<'_>) {
        let z = ctx.motion.pos().z;
        let tolerance = ctx.config.processing.approximation_tolerance;
        let limits = ctx.motion.sim.limits;
        let fire = ctx.config.laser.expose_on_follow;
        // Hidden layers are not exposed by the machine.
        let visible: Vec<Shape> = ctx
            .shapes
            .iter()
            .filter(|s| !is_hidden(s, ctx))
            .cloned()
            .collect();
        match ctx.runner.start(&visible, z, tolerance, &limits, fire) {
            Ok(steps) => {
                *ctx.status = AppStatus::Running;
                let beam = if fire { ", beam armed" } else { "" };
                ctx.note(
                    log::Level::Info,
                    "viewport",
                    format!("path follow started ({steps} waypoints{beam})"),
                );
            }
            Err(e) => ctx.note(log::Level::Error, "viewport", format!("path follow: {e}")),
        }
    }

    fn open(&mut self, ctx: &mut PanelCtx<'_>) {
        let dialog = rfd::FileDialog::new().set_directory(&ctx.config.file.open_dir);
        let Some(path) = dialog.pick_file() else {
            return;
        };
        match crate::parser::parse_file(&path) {
            Ok(mut shapes) => {
                let n = shapes.len();
                ctx.shapes.append(&mut shapes);
                ctx.config.file.last_file = Some(path);
                self.fit_pending = true;
                ctx.note(log::Level::Info, "viewport", format!("loaded {n} shapes"));
            }
            Err(e) => ctx.note(log::Level::Error, "viewport", format!("{e}")),
        }
    }

    fn handle_input(
        &mut self,
        ui: &egui::Ui,
        resp: &egui::Response,
        rect: egui::Rect,
        ctx: &mut PanelCtx<'_>,
    ) {
        if resp.dragged() {
            let d = resp.drag_delta();
            self.pan.x -= d.x as f64 / self.zoom;
            self.pan.y += d.y as f64 / self.zoom;
            ctx_repaint(ui);
        }
        if resp.hovered() {
            let zoom = ui.input(|i| i.zoom_delta());
            if zoom != 1.0 {
                self.zoom = (self.zoom * zoom as f64).clamp(MIN_ZOOM, MAX_ZOOM);
                ctx_repaint(ui);
            }
        }
        if resp.clicked()
            && let Some(pos) = resp.interact_pointer_pos()
        {
            let world = self.to_world(pos, rect);
            let tolerance = ctx.config.processing.approximation_tolerance.max(1e-4);
            let world_tol = HIT_PX / self.zoom.max(1e-9);
            let hit = pick(ctx.shapes, ctx, world, tolerance, world_tol);
            let additive = ui.input(|i| i.modifiers.shift || i.modifiers.ctrl);
            if additive {
                if let Some(i) = hit {
                    toggle_selection(ctx.selection, i);
                }
            } else {
                ctx.selection.clear();
                if let Some(i) = hit {
                    ctx.selection.push(i);
                }
            }
        }
    }

    /// Remove every selected shape (indices descending so positions stay valid).
    fn delete_selection(&mut self, ctx: &mut PanelCtx<'_>) {
        let mut idxs = ctx.selection.clone();
        idxs.sort_unstable();
        idxs.dedup();
        let n = idxs.len();
        for &i in idxs.iter().rev() {
            if i < ctx.shapes.len() {
                ctx.shapes.remove(i);
            }
        }
        ctx.selection.clear();
        if n > 0 {
            ctx.note(log::Level::Info, "viewport", format!("deleted {n} shapes"));
        }
    }

    fn to_screen(&self, p: Point, rect: egui::Rect) -> egui::Pos2 {
        pos2(
            rect.center().x + ((p.x - self.pan.x) * self.zoom) as f32,
            rect.center().y - ((p.y - self.pan.y) * self.zoom) as f32,
        )
    }

    fn to_world(&self, p: egui::Pos2, rect: egui::Rect) -> Point {
        Point::new(
            self.pan.x + (p.x - rect.center().x) as f64 / self.zoom,
            self.pan.y - (p.y - rect.center().y) as f64 / self.zoom,
        )
    }

    fn draw_grid(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        color: Color32,
        ctx: &PanelCtx<'_>,
    ) {
        let base = ctx.config.ui.grid_spacing.max(0.01);
        let mut step = base;
        while step * self.zoom < 32.0 {
            step *= 10.0;
        }
        while step * self.zoom > 160.0 {
            step /= 10.0;
        }

        let bl = self.to_world(rect.left_bottom(), rect);
        let tr = self.to_world(rect.right_top(), rect);
        let mut x = (bl.x / step).ceil() * step;
        while x <= tr.x {
            let p = self.to_screen(Point::new(x, 0.0), rect);
            let major = (x / step).abs() % 5.0 < 0.5;
            painter.line_segment(
                [pos2(p.x, rect.top()), pos2(p.x, rect.bottom())],
                Stroke::new(if major { 1.0_f32 } else { 0.5_f32 }, color),
            );
            x += step;
        }
        let mut y = (bl.y / step).ceil() * step;
        while y <= tr.y {
            let p = self.to_screen(Point::new(0.0, y), rect);
            let major = (y / step).abs() % 5.0 < 0.5;
            painter.line_segment(
                [pos2(rect.left(), p.y), pos2(rect.right(), p.y)],
                Stroke::new(if major { 1.0_f32 } else { 0.5_f32 }, color),
            );
            y += step;
        }
    }

    fn fit_to_shapes(&mut self, ctx: &PanelCtx<'_>, avail: egui::Vec2) {
        let mut bounds: Option<Rect> = None;
        for s in ctx.shapes.iter() {
            if let Some(b) = s.bounds() {
                bounds = Some(match bounds {
                    Some(cur) => cur.union(&b),
                    None => b,
                });
            }
        }
        let Some(b) = bounds else {
            self.fit_pending = false;
            return;
        };
        self.pan = b.center();
        let span = b.width().max(b.height()).max(1e-3);
        let usable = avail.x.min(avail.y).max(1.0) as f64;
        self.zoom = ((usable * 0.9) / span).clamp(MIN_ZOOM, MAX_ZOOM);
        self.fit_pending = false;
    }

    fn info_line(&self, ctx: &PanelCtx<'_>) -> String {
        let total: f64 = ctx.shapes.iter().map(|s| s.total_length()).sum();
        let selected = if ctx.selection.is_empty() {
            String::new()
        } else {
            format!(" · {} selected", ctx.selection.len())
        };
        format!(
            "{} shapes{} · {:.1} mm path",
            ctx.shapes.len(),
            selected,
            total
        )
    }
}

/// Whether a shape's layer is currently hidden.
fn is_hidden(shape: &Shape, ctx: &PanelCtx<'_>) -> bool {
    shape
        .layer
        .as_ref()
        .is_some_and(|l| ctx.hidden_layers.contains(l))
}

fn ctx_repaint(ui: &egui::Ui) {
    ui.ctx().request_repaint();
}

/// Pick the shape nearest the click: the pointer must lie within `world_tol`
/// of the shape's actual geometry (not just its bounding box), so thin lines
/// and arcs are selectable and overlapping shapes resolve by proximity.
/// Ties go to the topmost (last-drawn) shape.
fn pick(
    shapes: &[Shape],
    ctx: &PanelCtx<'_>,
    world: Point,
    tolerance: f64,
    world_tol: f64,
) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, shape) in shapes.iter().enumerate() {
        if is_hidden(shape, ctx) {
            continue;
        }
        // Cheap reject: only shapes whose (expanded) bounds contain the point
        // can be within tolerance of their geometry.
        if let Some(b) = shape.bounds() {
            let inside = world.x >= b.min.x - world_tol
                && world.x <= b.max.x + world_tol
                && world.y >= b.min.y - world_tol
                && world.y <= b.max.y + world_tol;
            if !inside {
                continue;
            }
        }
        let d = shape_distance(shape, world, tolerance);
        if d <= world_tol && best.is_none_or(|(_, bd)| d <= bd) {
            best = Some((i, d));
        }
    }
    best.map(|(i, _)| i)
}

/// Shortest distance from `p` to the shape's flattened outline (mm).
fn shape_distance(shape: &Shape, p: Point, tolerance: f64) -> f64 {
    let mut best = f64::INFINITY;
    for polyline in polylines(shape, tolerance) {
        for seg in polyline.windows(2) {
            best = best.min(seg_dist(p, seg[0], seg[1]));
        }
    }
    best
}

/// Distance from point `p` to segment `a → b`.
fn seg_dist(p: Point, a: Point, b: Point) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len2 = abx * abx + aby * aby;
    if len2 <= f64::EPSILON {
        return p.distance(a);
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0);
    let proj = Point::new(a.x + t * abx, a.y + t * aby);
    p.distance(proj)
}

fn polylines(shape: &Shape, tolerance: f64) -> Vec<Vec<Point>> {
    shape
        .elements
        .iter()
        .map(|e| match e {
            PathElement::Line(l) => vec![l.start, l.end],
            PathElement::Arc(a) => a.to_segments(arc_segments(a.sweep.abs(), a.radius, tolerance)),
        })
        .collect()
}

fn arc_segments(sweep: f64, radius: f64, tolerance: f64) -> usize {
    let tol = tolerance.abs().max(1e-6);
    let r = radius.abs().max(1e-6);
    let chord_angle = (8.0 * tol / r).sqrt();
    (sweep.abs() / chord_angle).ceil().clamp(2.0, 512.0) as usize
}

fn demo_shapes() -> Vec<Shape> {
    let r = 8.0;
    let circle = Shape::new(
        vec![PathElement::Arc(crate::core::geo::Arc::full_circle(
            Point::new(25.0, 0.0),
            r,
        ))],
        true,
    );
    let mut plate: Vec<PathElement> = Vec::new();
    let corners = [
        Point::new(-20.0, -12.0),
        Point::new(-20.0, 12.0),
        Point::new(0.0, 12.0),
        Point::new(0.0, -12.0),
    ];
    for i in 0..corners.len() {
        plate.push(PathElement::Line(crate::core::geo::Line::new(
            corners[i],
            corners[(i + 1) % corners.len()],
        )));
    }
    let plate = Shape::new(plate, true);
    vec![plate, circle]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::{Arc, Line, PathElement};

    fn hline(y: f64) -> Shape {
        Shape::new(
            vec![PathElement::Line(Line::new(
                Point::new(0.0, y),
                Point::new(10.0, y),
            ))],
            false,
        )
    }

    #[test]
    fn seg_dist_is_zero_on_the_line_and_exact_off_it() {
        let a = Point::new(0.0, 0.0);
        let b = Point::new(10.0, 0.0);
        assert_eq!(seg_dist(Point::new(4.0, 0.0), a, b), 0.0);
        assert_eq!(seg_dist(Point::new(4.0, 3.0), a, b), 3.0);
        // Beyond the ends the distance is to the endpoint.
        assert_eq!(seg_dist(Point::new(13.0, 4.0), a, b), 5.0);
        assert_eq!(seg_dist(Point::new(-3.0, 4.0), a, b), 5.0);
        // Degenerate segment behaves like a point.
        assert_eq!(seg_dist(Point::new(3.0, 4.0), a, a), 5.0);
    }

    #[test]
    fn shape_distance_ignores_bounding_boxes() {
        // An L-shape: its bbox contains (10, 10), but the geometry does not.
        let l = Shape::new(
            vec![
                PathElement::Line(Line::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0))),
                PathElement::Line(Line::new(Point::new(10.0, 0.0), Point::new(10.0, 10.0))),
            ],
            false,
        );
        let inside_bbox = Point::new(2.0, 8.0);
        assert!(l.bounds().unwrap().contains(inside_bbox));
        assert!(
            shape_distance(&l, inside_bbox, 0.01) > 7.0,
            "click far from both legs must be far from the geometry"
        );
        assert_eq!(shape_distance(&l, Point::new(5.0, 0.0), 0.01), 0.0);
    }

    #[test]
    fn arcs_are_measured_on_their_curve() {
        let circle = Shape::new(
            vec![PathElement::Arc(Arc::full_circle(
                Point::new(0.0, 0.0),
                5.0,
            ))],
            true,
        );
        // On the rim: distance zero. At the centre: ~5 mm from the rim
        // (chords of the flattened arc sit up to `tolerance` inside).
        assert!(shape_distance(&circle, Point::new(5.0, 0.0), 0.01) < 1e-6);
        let d = shape_distance(&circle, Point::ORIGIN, 0.01);
        assert!((d - 5.0).abs() < 0.02, "centre distance {d}");
    }

    #[test]
    fn thin_shapes_are_pickable_within_the_click_tolerance() {
        let tolerance = 0.01;
        let world_tol = 0.3; // ~6 px at 20 px/mm
        // A single horizontal line and a click 0.2 mm above it: inside 0.3.
        assert!(shape_distance(&hline(0.0), Point::new(5.0, 0.2), tolerance) <= world_tol);
        // A click 0.5 mm away: outside.
        assert!(shape_distance(&hline(0.0), Point::new(5.0, 0.5), tolerance) > world_tol);
    }
}
