//! Modular UI.
//!
//! Every widget is a `Panel` implementation in [`panels`]. A panel owns its own
//! UI state, declares which [`Slot`] it lives in, and receives shared state
//! through [`PanelCtx`]. Adding a widget is three steps:
//!
//! 1. Implement `Panel` in a new file under `src/ui/panels/`, keeping widget
//!    state in the struct and reading/writing shared state via `ctx`.
//! 2. Export it from `src/ui/panels/mod.rs`.
//! 3. Add it to [`PanelRegistry::standard`].
//!
//! Nothing else has to change: the registry sorts panels by slot and order,
//! frames them, and hands the context over.

pub mod buffer;
pub mod panels;

use egui::{CollapsingHeader, Context, RichText, Ui};

use crate::app::AppStatus;
use crate::config::AppConfig;
use crate::core::motion::MotionHub;
use crate::core::path::PathRunner;

use buffer::LogBuffer;

/// Colour used for the beam indicator anywhere in the UI.
pub const LASER_COLOR: egui::Color32 = egui::Color32::from_rgb(235, 82, 82);

/// Add `i` to the selection, or remove it when already present.
pub fn toggle_selection(selection: &mut Vec<usize>, i: usize) {
    match selection.iter().position(|&x| x == i) {
        Some(pos) => {
            selection.remove(pos);
        }
        None => selection.push(i),
    }
}

/// Where a panel is placed. Ordering is derived, so a panel can move slots
/// without touching the app.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Slot {
    Top,
    Left,
    Right,
    Bottom,
    Center,
}

impl Slot {
    /// Side/top panels are framed as collapsible sections, the rest draw their
    /// own header (a status strip has no business collapsing).
    pub fn is_collapsible(self) -> bool {
        matches!(self, Slot::Left | Slot::Right | Slot::Top)
    }

    /// Render `body` into the egui container for this slot.
    pub fn show(
        self,
        ctx: &Context,
        id: &'static str,
        _title: &str,
        size: f32,
        body: impl FnOnce(&mut Ui),
    ) {
        let _collapsible = self.is_collapsible();
        let wrap = move |ui: &mut Ui| {
            body(ui);
            // if collapsible {
            //     CollapsingHeader::new(RichText::new(title).strong())
            //         .id_salt(id)
            //         .show(ui, body);
            // } else {
            //     body(ui);
            // }
        };
        match self {
            Slot::Top => {
                egui::TopBottomPanel::top(id)
                    .resizable(true)
                    .default_height(size)
                    .show(ctx, wrap);
            }
            Slot::Bottom => {
                egui::TopBottomPanel::bottom(id)
                    .resizable(true)
                    .default_height(size)
                    .show(ctx, wrap);
            }
            Slot::Left => {
                egui::SidePanel::left(id)
                    .resizable(true)
                    .default_width(size)
                    .show(ctx, wrap);
            }
            Slot::Right => {
                egui::SidePanel::right(id)
                    .resizable(true)
                    .default_width(size)
                    .show(ctx, wrap);
            }
            Slot::Center => {
                egui::CentralPanel::default().show(ctx, wrap);
            }
        }
    }
}

/// Shared state handed to every panel for one frame.
///
/// Each field is a disjoint borrow of a field on [`crate::app::LithoApp`], so a
/// panel can mutate shared state without any interior mutability.
pub struct PanelCtx<'a> {
    pub egui: &'a Context,
    pub config: &'a mut AppConfig,
    pub motion: &'a mut MotionHub,
    pub status: &'a mut AppStatus,
    pub log: &'a mut LogBuffer,
    pub shapes: &'a mut Vec<crate::core::geo::Shape>,
    pub runner: &'a mut PathRunner,
    /// Selected shape indices (shared between viewport and geometry tools).
    pub selection: &'a mut Vec<usize>,
    /// Layer names hidden in the viewport.
    pub hidden_layers: &'a mut std::collections::HashSet<String>,
}

impl PanelCtx<'_> {
    pub fn note(&mut self, level: log::Level, module: &'static str, message: impl Into<String>) {
        self.log.push(level, module, message);
    }
}

/// A modular widget.
pub trait Panel {
    /// Stable id: used for egui state persistence and container ids.
    fn id(&self) -> &'static str;

    /// Header text.
    fn title(&self) -> &'static str;

    fn slot(&self) -> Slot {
        Slot::Left
    }

    /// Sort key inside the slot; lower comes first.
    fn order(&self) -> i32 {
        0
    }

    fn default_open(&self) -> bool {
        true
    }

    /// Default width (side panels) or height (top/bottom panels) in points.
    fn size(&self) -> f32 {
        300.0
    }

    fn show(&mut self, ui: &mut Ui, ctx: &mut PanelCtx<'_>);
}

/// Ordered collection of widgets, rendered once per frame.
#[derive(Default)]
pub struct PanelRegistry {
    panels: Vec<Box<dyn Panel>>,
}

impl PanelRegistry {
    pub fn new() -> Self {
        PanelRegistry::default()
    }

    /// The application layout: motion + config on the left, viewport in the
    /// middle, log and status at the bottom.
    pub fn standard() -> Self {
        use panels::*;
        PanelRegistry::new()
            .with(MotionPanel::new())
            // .with(ConfigPanel::default())
            .with(ViewportPanel::default())
            .with(GeometryPanel::new())
            .with(LogPanel::default())
            .with(StatusPanel)
    }

    pub fn with(mut self, panel: impl Panel + 'static) -> Self {
        self.add(panel);
        self
    }

    pub fn add(&mut self, panel: impl Panel + 'static) -> &mut Self {
        self.panels.push(Box::new(panel));
        self
    }

    pub fn len(&self) -> usize {
        self.panels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.panels.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &dyn Panel> {
        self.panels.iter().map(|p| p.as_ref())
    }

    pub fn get_mut(&mut self, id: &str) -> Option<&mut (dyn Panel + '_)> {
        let panel = self
            .panels
            .iter_mut()
            .find(|p| p.id() == id)
            .map(|p| p.as_mut())?;
        Some(panel)
    }

    pub fn render(&mut self, ctx: &Context, pctx: &mut PanelCtx<'_>) {
        let mut order: Vec<usize> = (0..self.panels.len()).collect();
        order.sort_by_key(|&i| (self.panels[i].slot(), self.panels[i].order()));

        for i in order {
            let (slot, id, title, size) = {
                let p = self.panels[i].as_ref();
                (p.slot(), p.id(), p.title(), p.size())
            };
            let panel = self.panels[i].as_mut();
            slot.show(ctx, id, title, size, |ui| panel.show(ui, pctx));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::motion::Axes;

    #[test]
    fn standard_registry_has_unique_ids() {
        let mut reg = PanelRegistry::standard();
        assert_eq!(reg.len(), 5);
        let mut ids: Vec<&str> = reg.iter().map(|p| p.id()).collect();
        ids.sort_unstable();
        let unique = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), unique, "panel ids must be unique");
        assert!(reg.iter().any(|p| p.slot() == Slot::Center));
        assert!(reg.get_mut("motion").is_some());
        assert!(reg.get_mut("nope").is_none());
    }

    #[test]
    fn toggle_selection_adds_then_removes() {
        let mut sel = Vec::new();
        toggle_selection(&mut sel, 3);
        toggle_selection(&mut sel, 1);
        toggle_selection(&mut sel, 3);
        assert_eq!(sel, vec![1], "second toggle of 3 removes it");
    }

    #[test]
    fn panels_render_without_panicking() {
        let ctx = Context::default();
        for _ in 0..3 {
            let _ = ctx.run(egui::RawInput::default(), |ctx| {
                let mut config = AppConfig::default();
                let mut motion = MotionHub::new(50.0, 500.0);
                motion.move_to(Axes::new(10.0, 5.0, 1.0)).unwrap();
                motion.tick();
                let mut log = LogBuffer::default();
                log.info("test", "frame");
                let mut status = AppStatus::Ready;
                let mut shapes = Vec::new();
                let mut runner = PathRunner::new();
                let mut selection = Vec::new();
                let mut hidden_layers = std::collections::HashSet::new();

                let mut registry = PanelRegistry::standard();
                let mut pctx = PanelCtx {
                    egui: ctx,
                    config: &mut config,
                    motion: &mut motion,
                    status: &mut status,
                    log: &mut log,
                    shapes: &mut shapes,
                    runner: &mut runner,
                    selection: &mut selection,
                    hidden_layers: &mut hidden_layers,
                };
                registry.render(ctx, &mut pctx);
            });
        }
    }
}
