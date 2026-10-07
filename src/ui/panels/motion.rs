//! Motor tracking / simulation widget: drive mode, live XYZ readout, jog,
//! and a top-down plot of where the stage has been.

use std::time::Duration;

use egui::{
    ComboBox, DragValue, Pos2, Rect as EguiRect, RichText, Sense, Slider, Stroke, Ui, Vec2, pos2,
    vec2,
};

use crate::app::AppStatus;
use crate::core::geo::fmt_f;
use crate::core::motion::{AXES, Axes, DriveMode};
use crate::esp301::LinkConfig;
use crate::ui::{LASER_COLOR, Panel, PanelCtx, Slot};

const PLOT_HEIGHT: f32 = 190.0;

#[derive(Debug, Clone, Copy)]
struct PlotView {
    /// World-space centre of the plot, mm (X/Y only).
    center: (f64, f64),
    px_per_mm: f32,
}

pub struct MotionPanel {
    step_mm: f64,
    feed_mm_s: f64,
    target_mm: [f64; 3],
    show_plot: bool,
    port: String,
    ports: Vec<String>,
    view: Option<PlotView>,
}

impl Default for MotionPanel {
    fn default() -> Self {
        MotionPanel {
            step_mm: 1.0,
            feed_mm_s: 50.0,
            target_mm: [0.0; 3],
            show_plot: true,
            port: String::new(),
            ports: Vec::new(),
            view: None,
        }
    }
}

impl MotionPanel {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Panel for MotionPanel {
    fn id(&self) -> &'static str {
        "motion"
    }

    fn title(&self) -> &'static str {
        "Motion"
    }

    fn slot(&self) -> Slot {
        Slot::Left
    }

    fn order(&self) -> i32 {
        0
    }

    fn size(&self) -> f32 {
        340.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        // Keep the beam pin in sync with the config (it can change any frame).
        ctx.motion.laser_pin = ctx.config.laser.digital_output_pin;
        self.drive_section(ui, ctx);
        ui.add_space(6.0);
        self.readout_section(ui, ctx);
        ui.add_space(6.0);
        self.target_section(ui, ctx);
        ui.add_space(6.0);
        self.jog_section(ui, ctx);
        ui.add_space(6.0);
        self.machine_section(ui, ctx);
        if self.show_plot {
            ui.add_space(6.0);
            self.plot_section(ui, ctx);
        }
    }
}

impl MotionPanel {
    fn drive_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        egui::Grid::new("motion.drive")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Drive");
                ComboBox::from_id_salt("motion.mode")
                    .selected_text(ctx.motion.mode.label())
                    .show_ui(ui, |ui| {
                        for mode in DriveMode::ALL {
                            ui.selectable_value(&mut ctx.motion.mode, mode, mode.label());
                        }
                    });
                ui.end_row();

                ui.label("Feedrate");
                ui.add(
                    Slider::new(&mut self.feed_mm_s, 0.0..=500.0)
                        .suffix(" mm/s")
                        .clamping(egui::SliderClamping::Edits),
                )
                .on_hover_text("Also written to the controller on every jog");
                ui.end_row();
            });

        if ctx.motion.mode == DriveMode::Hardware {
            ui.horizontal(|ui| {
                if ui.button("Refresh").clicked() {
                    self.ports = list_ports();
                    if self.port.is_empty() {
                        self.port = self.ports.first().cloned().unwrap_or_default();
                    }
                }
                ComboBox::from_id_salt("motion.port")
                    .width(120.0)
                    .selected_text(if self.port.is_empty() {
                        "<no ports>".into()
                    } else {
                        self.port.clone()
                    })
                    .show_ui(ui, |ui| {
                        if self.ports.is_empty() {
                            ui.label("Refresh to scan");
                        }
                        for p in &self.ports.clone() {
                            ui.selectable_value(&mut self.port, p.clone(), p);
                        }
                    });

                if ctx.motion.is_linked() {
                    if ui.button("Disconnect").clicked() {
                        ctx.motion.disconnect();
                        ctx.note(log::Level::Warn, "motion", "link closed");
                    }
                } else {
                    let enabled = !self.port.is_empty();
                    if ui
                        .add_enabled(enabled, egui::Button::new("Connect"))
                        .clicked()
                    {
                        self.connect(ctx);
                    }
                }
            });
        }

        match &ctx.motion.link_error {
            Some(err) => {
                ui.colored_label(ui.visuals().error_fg_color, format!("link error: {err}"));
            }
            None => {
                let (text, color) = match ctx.motion.mode {
                    DriveMode::Simulated => ("simulation", ui.visuals().weak_text_color()),
                    DriveMode::Hardware if ctx.motion.is_linked() => {
                        ("connected", ui.visuals().selection.stroke.color)
                    }
                    DriveMode::Hardware => ("not connected", ui.visuals().warn_fg_color),
                };
                ui.colored_label(color, text);
            }
        }
    }

    fn connect(&mut self, ctx: &mut PanelCtx<'_>) {
        let cfg = LinkConfig {
            port: self.port.clone(),
            baud: ctx.config.controller.baud_rate,
            timeout: Duration::from_secs_f64(ctx.config.controller.timeout_secs.max(0.1)),
        };
        match ctx.motion.connect(&cfg) {
            Ok(()) => {
                let msg = format!("connected to {}", cfg.port);
                ctx.note(log::Level::Info, "motion", msg);
            }
            Err(e) => {
                let msg = format!("connect failed: {e}");
                ctx.note(log::Level::Error, "motion", msg);
            }
        }
    }

    fn readout_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        let pos = ctx.motion.pos();
        let vel = ctx.motion.vel();
        let pos = pos.to_array();
        let vel = vel.to_array();

        let moving = ctx.motion.is_moving();
        let state = if moving {
            ui.visuals().selection.stroke.color
        } else {
            ui.visuals().weak_text_color()
        };

        egui::Grid::new("motion.readout")
            .num_columns(5)
            .spacing([12.0, 2.0])
            .show(ui, |ui| {
                ui.label(RichText::new("axis").weak());
                ui.label(RichText::new("position").weak());
                ui.label(RichText::new("velocity").weak());
                ui.label(RichText::new("target").weak());
                ui.label(RichText::new("error").weak());
                ui.end_row();

                let target = ctx.motion.target().to_array();
                for (i, name) in AXES.iter().enumerate() {
                    ui.label(*name);
                    ui.label(mono(fmt_f(pos[i])));
                    ui.label(mono(fmt_f(vel[i])));
                    ui.label(mono(fmt_f(target[i])));
                    let err = pos[i] - target[i];
                    ui.colored_label(
                        if err.abs() > 0.001 {
                            ui.visuals().warn_fg_color
                        } else {
                            ui.visuals().weak_text_color()
                        },
                        mono(fmt_f(err)),
                    );
                    ui.end_row();
                }
            });

        ui.horizontal(|ui| {
            ui.label(mono(format!("{} mm to go", fmt_f(ctx.motion.remaining()))));
            ui.colored_label(state, if moving { "moving" } else { "idle" });
            let eta = ctx.motion.eta();
            if moving {
                ui.label(if eta.is_finite() {
                    mono(format!("eta {eta:.2}s"))
                } else {
                    mono("eta --".to_string())
                });
            }
        });
    }

    fn target_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Move to").strong());
        ui.horizontal(|ui| {
            for (i, name) in AXES.iter().enumerate() {
                ui.label(*name);
                ui.add(
                    DragValue::new(&mut self.target_mm[i])
                        .speed(0.05)
                        .max_decimals(3),
                );
            }
            if ui.button("Go").clicked() {
                self.cancel_follow(ctx);
                let target = Axes::from_array(self.target_mm);
                let msg = format!("move to {}", target);
                match ctx.motion.move_to(target) {
                    Ok(()) => ctx.note(log::Level::Info, "motion", msg),
                    Err(e) => ctx.note(log::Level::Error, "motion", e),
                }
            }
            if ui.button("Read").clicked() {
                self.target_mm = ctx.motion.pos().to_array();
            }
        });
    }

    fn jog_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Jog").strong());
            ui.checkbox(&mut self.show_plot, "plot");
            ui.label("step");
            ui.add(
                DragValue::new(&mut self.step_mm)
                    .speed(0.01)
                    .range(0.001..=100.0)
                    .suffix(" mm"),
            );
        });

        egui::Grid::new("motion.jog").num_columns(3).show(ui, |ui| {
            for (i, name) in AXES.iter().enumerate() {
                ui.label(*name);
                if ui.button("−").clicked() {
                    self.jog(ctx, i, -1.0);
                }
                if ui.button("+").clicked() {
                    self.jog(ctx, i, 1.0);
                }
                ui.end_row();
            }
        });
    }

    fn jog(&mut self, ctx: &mut PanelCtx<'_>, axis: usize, direction: f64) {
        self.cancel_follow(ctx);
        let step = self.step_mm;
        let feed = self.feed_mm_s;
        if let Err(e) = ctx.motion.jog(axis, direction, step, feed) {
            ctx.note(log::Level::Error, "motion", e);
        }
    }

    fn machine_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        ui.label(RichText::new("Machine").strong());
        ui.horizontal(|ui| {
            if ui.button("Home").clicked() {
                let axes = [0usize, 1, 2];
                match ctx.motion.home(&axes) {
                    Ok(()) => ctx.note(log::Level::Info, "motion", "homing"),
                    Err(e) => ctx.note(log::Level::Error, "motion", e),
                }
            }
            if ui.button("Stop").clicked() {
                self.cancel_follow(ctx);
                let _ = ctx.motion.stop(false);
            }
            if ui.button("Ramp stop").clicked() {
                self.cancel_follow(ctx);
                let _ = ctx.motion.stop(true);
            }
            if ui.button("Reset sim").clicked() {
                self.cancel_follow(ctx);
                ctx.motion.sim.reset();
                self.target_mm = [0.0; 3];
                ctx.note(log::Level::Info, "motion", "simulator reset");
            }
        });
        ui.horizontal(|ui| {
            self.laser_button(ui, ctx);
            ui.checkbox(&mut ctx.config.laser.expose_on_follow, "auto on follow")
                .on_hover_text("fire the beam automatically while following a path");
        });
    }

    /// Manual beam toggle: flips the digital output the config points at.
    fn laser_button(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        let on = ctx.motion.laser_on();
        let can_fire = ctx.motion.mode == DriveMode::Simulated || ctx.motion.is_linked();
        let text = if on {
            RichText::new("● Laser ON").color(LASER_COLOR)
        } else {
            RichText::new("○ Laser OFF").weak()
        };
        let resp = ui
            .add_enabled(can_fire, egui::Button::new(text))
            .on_hover_text(format!(
                "manual beam toggle — digital output {} from config",
                ctx.config.laser.digital_output_pin
            ))
            .on_disabled_hover_text("connect the controller first");
        if resp.clicked() {
            match ctx.motion.set_laser(!on) {
                Ok(()) => ctx.note(
                    log::Level::Info,
                    "motion",
                    if !on { "laser ON" } else { "laser OFF" },
                ),
                Err(e) => ctx.note(log::Level::Error, "motion", format!("laser: {e}")),
            }
        }
    }

    /// Abort an active path-follow run, e.g. when the operator takes over.
    fn cancel_follow(&mut self, ctx: &mut PanelCtx<'_>) {
        if ctx.runner.is_running() {
            ctx.runner.stop(ctx.motion);
            *ctx.status = AppStatus::Ready;
            ctx.note(log::Level::Info, "motion", "path follow stopped");
        }
    }

    fn plot_section(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        let size = Vec2::new(ui.available_width(), PLOT_HEIGHT);
        let (resp, painter) = ui.allocate_painter(size, Sense::click());
        let rect = resp.rect;
        let view = self.fit_view(rect, ctx);
        self.view = Some(view);

        painter.rect_stroke(
            rect,
            0.0,
            Stroke::new(1.0_f32, ui.visuals().weak_text_color()),
            egui::StrokeKind::Middle,
        );
        draw_grid(&painter, rect, view, ui);

        let accent = ui.visuals().selection.stroke.color;
        let trail: Vec<Pos2> = ctx
            .motion
            .trail
            .iter()
            .map(|p| project(view, rect, p.x, p.y))
            .collect();
        if trail.len() >= 2 {
            painter.add(egui::Shape::line(trail, Stroke::new(1.5_f32, accent)));
        }

        let pos = ctx.motion.pos();
        let laser_on = ctx.motion.laser_on();
        let cur = project(view, rect, pos.x, pos.y);
        let marker = if laser_on { LASER_COLOR } else { accent };
        painter.circle_filled(cur, 4.0, marker);
        painter.circle_stroke(cur, 8.0, Stroke::new(1.0_f32, marker));

        let target = ctx.motion.target();
        let tp = project(view, rect, target.x, target.y);
        let mark = Stroke::new(1.0_f32, ui.visuals().warn_fg_color);
        painter.line_segment([tp + vec2(-5.0, 0.0), tp + vec2(5.0, 0.0)], mark);
        painter.line_segment([tp + vec2(0.0, -5.0), tp + vec2(0.0, 5.0)], mark);

        painter.text(
            rect.left_top() + vec2(6.0, 4.0),
            egui::Align2::LEFT_TOP,
            format!(
                "x {}  y {}  z {}   ({} samples)",
                fmt_f(pos.x),
                fmt_f(pos.y),
                fmt_f(pos.z),
                ctx.motion.trail.len()
            ),
            egui::TextStyle::Small.resolve(ui.style()),
            ui.visuals().text_color(),
        );

        // Beam state, top-right corner of the plot.
        painter.text(
            rect.right_top() + vec2(-6.0, 4.0),
            egui::Align2::RIGHT_TOP,
            if laser_on {
                "● LASER"
            } else {
                "○ laser off"
            },
            egui::TextStyle::Small.resolve(ui.style()),
            if laser_on {
                LASER_COLOR
            } else {
                ui.visuals().weak_text_color()
            },
        );

        if resp.clicked()
            && let Some(pointer) = resp.interact_pointer_pos()
        {
            let wx = view.center.0 + (pointer.x - rect.center().x) as f64 / view.px_per_mm as f64;
            let wy = view.center.1 - (pointer.y - rect.center().y) as f64 / view.px_per_mm as f64;
            self.cancel_follow(ctx);
            let target = Axes::new(wx, wy, pos.z);
            self.target_mm = target.to_array();
            if let Err(e) = ctx.motion.move_to(target) {
                ctx.note(log::Level::Error, "motion", e);
            }
        }
    }

    fn fit_view(&self, rect: EguiRect, ctx: &PanelCtx<'_>) -> PlotView {
        let pos = ctx.motion.pos();
        let mut min = (pos.x, pos.y);
        let mut max = (pos.x, pos.y);
        for p in ctx.motion.trail.iter() {
            min = (min.0.min(p.x), min.1.min(p.y));
            max = (max.0.max(p.x), max.1.max(p.y));
        }
        let span_x = (max.0 - min.0).max(2.0);
        let span_y = (max.1 - min.1).max(2.0);
        let usable = rect.width().min(rect.height()) * 0.86;
        PlotView {
            center: ((min.0 + max.0) * 0.5, (min.1 + max.1) * 0.5),
            px_per_mm: (usable as f64 / span_x.max(span_y)).max(0.01) as f32,
        }
    }
}

fn project(view: PlotView, rect: EguiRect, x: f64, y: f64) -> Pos2 {
    pos2(
        rect.center().x + ((x - view.center.0) * view.px_per_mm as f64) as f32,
        rect.center().y - ((y - view.center.1) * view.px_per_mm as f64) as f32,
    )
}

fn draw_grid(painter: &egui::Painter, rect: EguiRect, view: PlotView, ui: &Ui) {
    let color = ui.visuals().weak_text_color();
    let (min_x, min_y, max_x, max_y) = (
        view.center.0 - rect.width() as f64 / view.px_per_mm as f64 / 2.0,
        view.center.1 - rect.height() as f64 / view.px_per_mm as f64 / 2.0,
        view.center.0 + rect.width() as f64 / view.px_per_mm as f64 / 2.0,
        view.center.1 + rect.height() as f64 / view.px_per_mm as f64 / 2.0,
    );
    let mut step = 1.0f64;
    while step * (view.px_per_mm as f64) < 40.0 {
        step *= 2.0;
    }
    while step * (view.px_per_mm as f64) > 120.0 {
        step /= 2.0;
    }

    let mut x = (min_x / step).ceil() * step;
    while x <= max_x {
        let p = project(view, rect, x, 0.0);
        let major = (x / step).abs() % 5.0 < 0.5;
        painter.line_segment(
            [pos2(p.x, rect.top()), pos2(p.x, rect.bottom())],
            Stroke::new(if major { 1.0_f32 } else { 0.5_f32 }, color),
        );
        x += step;
    }
    let mut y = (min_y / step).ceil() * step;
    while y <= max_y {
        let p = project(view, rect, 0.0, y);
        let major = (y / step).abs() % 5.0 < 0.5;
        painter.line_segment(
            [pos2(rect.left(), p.y), pos2(rect.right(), p.y)],
            Stroke::new(if major { 1.0_f32 } else { 0.5_f32 }, color),
        );
        y += step;
    }
    painter.text(
        rect.right_bottom() + vec2(-6.0, -14.0),
        egui::Align2::RIGHT_BOTTOM,
        format!("grid {step} mm"),
        egui::TextStyle::Small.resolve(ui.style()),
        color,
    );
}

fn mono(s: impl ToString) -> RichText {
    RichText::new(s.to_string()).monospace()
}

fn list_ports() -> Vec<String> {
    match serialport::available_ports() {
        Ok(ports) => ports.into_iter().map(|p| p.port_name).collect(),
        Err(e) => {
            log::warn!("motion: port scan failed: {e}");
            Vec::new()
        }
    }
}
