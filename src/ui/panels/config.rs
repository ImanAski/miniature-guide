//! Configuration widget — one section per `AppConfig` sub-struct.

use egui::{DragValue, RichText, Ui};

use crate::config::{AppConfig, UiTheme};
use crate::ui::{Panel, PanelCtx, Slot};

#[derive(Debug, Default)]
pub struct ConfigPanel {
    dirty: bool,
    applied_theme: Option<UiTheme>,
}

impl Panel for ConfigPanel {
    fn id(&self) -> &'static str {
        "config"
    }

    fn title(&self) -> &'static str {
        "Configuration"
    }

    fn slot(&self) -> Slot {
        Slot::Left
    }

    fn order(&self) -> i32 {
        10
    }

    fn size(&self) -> f32 {
        340.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>) {
        self.apply_theme(ctx);
        ui.label(
            RichText::new(AppConfig::default_path().display().to_string())
                .small()
                .weak(),
        );
        ui.horizontal(|ui| {
            if ui.button("Save").clicked() {
                self.save(ctx);
            }
            if ui.button("Revert to defaults").clicked() {
                *ctx.config = AppConfig::default();
                self.dirty = false;
                self.apply_theme(ctx);
                ctx.note(log::Level::Warn, "config", "reverted to defaults (unsaved)");
            }
            if self.dirty {
                ui.label(RichText::new("modified").small().weak());
            }
        });

        let mut changed = false;

        section("Controller", true, ui, |ui| {
            changed |= controller(ui, ctx);
        });
        section("Laser", true, ui, |ui| {
            changed |= laser(ui, ctx);
        });
        section("Processing", false, ui, |ui| {
            changed |= processing(ui, ctx);
        });
        section("Interface", false, ui, |ui| {
            changed |= interface(ui, ctx);
        });
        section("Files", false, ui, |ui| {
            changed |= files(ui, ctx);
        });

        if changed {
            self.dirty = true;
        }
    }
}

impl ConfigPanel {
    fn save(&mut self, ctx: &mut PanelCtx<'_>) {
        let path = AppConfig::default_path();
        match ctx.config.save(&path) {
            Ok(()) => {
                self.dirty = false;
                ctx.note(
                    log::Level::Info,
                    "config",
                    format!("saved to {}", path.display()),
                );
            }
            Err(e) => ctx.note(log::Level::Error, "config", e),
        }
    }

    fn apply_theme(&mut self, ctx: &PanelCtx<'_>) {
        let theme = ctx.config.ui.theme;
        if self.applied_theme != Some(theme) {
            ctx.egui.set_theme(match theme {
                UiTheme::Dark => egui::Theme::Dark,
                UiTheme::Light => egui::Theme::Light,
            });
            self.applied_theme = Some(theme);
        }
    }
}

fn section(title: &str, default_open: bool, ui: &mut Ui, body: impl FnOnce(&mut Ui)) -> bool {
    egui::CollapsingHeader::new(RichText::new(title).strong())
        .id_salt(title)
        .default_open(default_open)
        .show(ui, body)
        .fully_open()
}

fn controller(ui: &mut Ui, ctx: &mut PanelCtx<'_>) -> bool {
    let c = &mut ctx.config.controller;
    let mut changed = false;
    egui::Grid::new("cfg.controller")
        .num_columns(2)
        .show(ui, |ui| {
            changed |= field(ui, "Port", &mut c.port);
            changed |= drag(ui, "Baud rate", &mut c.baud_rate, 0u32, 400_000);
            changed |= drag(ui, "Timeout (s)", &mut c.timeout_secs, 0.05f64, 60.0);
            changed |= drag(ui, "Velocity (mm/s)", &mut c.default_velocity, 0.0, 1000.0);
            changed |= drag(
                ui,
                "Acceleration (mm/s²)",
                &mut c.default_acceleration,
                0.0,
                20_000.0,
            );
            changed |= drag(ui, "Unit scale", &mut c.unit_scale, 0.0001, 1.0);
        });
    changed
}

fn laser(ui: &mut Ui, ctx: &mut PanelCtx<'_>) -> bool {
    let c = &mut ctx.config.laser;
    let mut changed = false;
    egui::Grid::new("cfg.laser").num_columns(2).show(ui, |ui| {
        changed |= slider(ui, "Default power", &mut c.default_power, 0.0, 100.0);
        changed |= drag(
            ui,
            "Frequency (Hz)",
            &mut c.default_frequency,
            0.0,
            100_000.0,
        );
        changed |= ui
            .add(DragValue::new(&mut c.wavelength_nm).speed(1.0))
            .changed();
        changed |= drag(ui, "Min pulse (µs)", &mut c.min_pulse_us, 0.0, 10_000.0);
        changed |= drag(ui, "Fade (ms)", &mut c.fade_time_ms, 0.0, 1000.0);
        changed |= ui
            .add(DragValue::new(&mut c.digital_output_pin).speed(1.0))
            .changed();
        changed |= ui
            .checkbox(&mut c.interlock_enabled, "Interlock check")
            .changed();
    });
    changed
}

fn processing(ui: &mut Ui, ctx: &mut PanelCtx<'_>) -> bool {
    let c = &mut ctx.config.processing;
    let mut changed = false;
    egui::Grid::new("cfg.processing")
        .num_columns(2)
        .show(ui, |ui| {
            changed |= drag(
                ui,
                "Raster threshold (mm)",
                &mut c.raster_threshold,
                0.0,
                10.0,
            );
            changed |= drag(ui, "Hatch spacing (mm)", &mut c.hatch_spacing, 0.0001, 1.0);
            changed |= drag(ui, "Hatch angle (°)", &mut c.hatch_angle, 0.0, 180.0);
            changed |= drag(
                ui,
                "Min feature (mm)",
                &mut c.min_feature_size,
                0.0001,
                10.0,
            );
            changed |= drag(
                ui,
                "Approx. tolerance (mm)",
                &mut c.approximation_tolerance,
                0.0001,
                1.0,
            );
            changed |= ui
                .add(DragValue::new(&mut c.raster_passes).speed(0.1))
                .changed();
            changed |= slider(ui, "Overlap (%)", &mut c.overlap_percent, 0.0, 90.0);
            changed |= ui
                .checkbox(&mut c.enable_multi_pass, "Multi-pass")
                .changed();
            changed |= ui.checkbox(&mut c.vector_only, "Vector only").changed();
        });
    changed
}

fn interface(ui: &mut Ui, ctx: &mut PanelCtx<'_>) -> bool {
    let c = &mut ctx.config.ui;
    let mut changed = false;
    egui::Grid::new("cfg.ui").num_columns(2).show(ui, |ui| {
        changed |= drag(ui, "Font size", &mut c.font_size, 8.0f32, 32.0f32);
        changed |= drag(ui, "Grid spacing (mm)", &mut c.grid_spacing, 0.01, 100.0);
        changed |= ui.checkbox(&mut c.show_grid, "Show grid").changed();
        changed |= ui
            .checkbox(&mut c.show_coordinates, "Show coordinates")
            .changed();
        changed |= ui
            .checkbox(&mut c.show_status_bar, "Show status bar")
            .changed();

        ui.label("Theme");
        let mut theme = c.theme;
        egui::ComboBox::from_id_salt("cfg.theme")
            .selected_text(match theme {
                UiTheme::Dark => "Dark",
                UiTheme::Light => "Light",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut theme, UiTheme::Dark, "Dark");
                ui.selectable_value(&mut theme, UiTheme::Light, "Light");
            });
        if theme != c.theme {
            c.theme = theme;
            changed = true;
        }
        ui.end_row();
    });
    changed
}

fn files(ui: &mut Ui, ctx: &mut PanelCtx<'_>) -> bool {
    let f = &mut ctx.config.file;
    let mut changed = false;
    egui::Grid::new("cfg.files").num_columns(2).show(ui, |ui| {
        ui.label("Open dir");
        ui.horizontal(|ui| {
            ui.label(RichText::new(f.open_dir.display().to_string()).small());
            if ui.small_button("…").clicked()
                && let Some(dir) = rfd::FileDialog::new().pick_folder()
            {
                f.open_dir = dir;
                changed = true;
            }
        });
        ui.end_row();

        ui.label("Save dir");
        ui.horizontal(|ui| {
            ui.label(RichText::new(f.save_dir.display().to_string()).small());
            if ui.small_button("…").clicked()
                && let Some(dir) = rfd::FileDialog::new().pick_folder()
            {
                f.save_dir = dir;
                changed = true;
            }
        });
        ui.end_row();

        if let Some(p) = f.last_file.clone() {
            ui.label("Last file");
            ui.label(RichText::new(p.display().to_string()).small());
            ui.end_row();
        }
    });
    changed
}

fn field(ui: &mut Ui, label: &str, value: &mut String) -> bool {
    ui.label(label);
    ui.text_edit_singleline(value).changed()
}

fn drag<T: egui::emath::Numeric>(ui: &mut Ui, label: &str, value: &mut T, min: T, max: T) -> bool {
    ui.label(label);
    ui.add(
        DragValue::new(value)
            .speed(0.05)
            .range(min..=max)
            .max_decimals(4),
    )
    .changed()
}

fn slider(ui: &mut Ui, label: &str, value: &mut f64, min: f64, max: f64) -> bool {
    ui.label(label);
    ui.add(egui::Slider::new(value, min..=max).clamping(egui::SliderClamping::Edits))
        .changed()
}
