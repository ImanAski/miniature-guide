//! Viewport widget — 2D canvas for loaded geometry, with pan/zoom and picking.

use egui::{Color32, RichText, Sense, Stroke, pos2, vec2};

use crate::core::geo::{PathElement, Point, Rect, Shape, fmt_f};
use crate::ui::{Panel, PanelCtx, Slot};

const MIN_ZOOM: f64 = 0.01;
const MAX_ZOOM: f64 = 400.0;

pub struct ViewportPanel {
    /// World point held at the canvas centre (mm).
    pan: Point,
    /// Screen pixels per millimetre.
    zoom: f64,
    show_grid: bool,
    show_origin: bool,
    selected: Option<usize>,
    fit_pending: bool,
}

impl Default for ViewportPanel {
    fn default() -> Self {
        ViewportPanel {
            pan: Point::ORIGIN,
            zoom: 2.0,
            show_grid: true,
            show_origin: true,
            selected: None,
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
            let selected = self.selected == Some(i);
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
            ui.checkbox(&mut self.show_grid, "grid");
            ui.checkbox(&mut self.show_origin, "origin");
            ui.separator();
            if ui
                .add_enabled(self.selected.is_some(), egui::Button::new("Delete"))
                .clicked()
                && let Some(i) = self.selected.take()
                && i < ctx.shapes.len()
            {
                ctx.shapes.remove(i);
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
        ctx: &PanelCtx<'_>,
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
            self.selected = pick(ctx.shapes, world);
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
        format!("{} shapes · {:.1} mm path", ctx.shapes.len(), total)
    }
}

fn ctx_repaint(ui: &egui::Ui) {
    ui.ctx().request_repaint();
}

fn pick(shapes: &[Shape], world: Point) -> Option<usize> {
    shapes
        .iter()
        .enumerate()
        .filter(|(_, s)| s.bounds().is_some_and(|b| b.contains(world)))
        .map(|(i, _)| i)
        .next_back()
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
