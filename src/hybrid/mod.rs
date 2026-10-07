//! Hybrid raster/vector write planner.
//!
//! Geometry is partitioned into rectangular tiles aligned to the configured
//! grid; every tile gets a [`WriteMode`] by minimising the estimated cost of
//! the three physically valid strategies (`Vector`, `Raster`, `Hybrid`) subject
//! to hard constraints. The tile size never explodes — it doubles on the rare
//! case the grid produces an enormous count — and costs are deterministic for
//! a given layout and machine configuration.
//!
//! This is the "simplest correct version" asked for: an analytical cost model
//! with clear hooks for a more advanced trajectory/TSP optimizer later. The
//! geometry analysis here is independent of the GDS parser so it can be
//! unit-tested on synthetic shapes. It operates on `mm` throughout.

use crate::core::geo::{Point, Rect};

pub mod cost;
pub mod plan;
pub mod raster;
pub mod segment;
pub mod vector;

pub use plan::{Plan, TilePlan, diagnostics_csv, diagnostics_report, plan, svg_report};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WriteMode {
    Vector,
    Raster,
    Hybrid,
}

impl WriteMode {
    pub fn label(self) -> &'static str {
        match self {
            WriteMode::Vector => "VECTOR",
            WriteMode::Raster => "RASTER",
            WriteMode::Hybrid => "HYBRID",
        }
    }

    /// Index into the [`crate::hybrid::cost::evaluate`] cost triple.
    /// Declaration order (Vector, Raster, Hybrid) is the canonical mode order:
    /// it breaks cost ties and orders report columns.
    pub fn index(self) -> usize {
        self as usize
    }

    /// Whether the mode traces contours/fill strokes with the beam.
    pub fn uses_vector(self) -> bool {
        matches!(self, WriteMode::Vector | WriteMode::Hybrid)
    }

    /// Whether the mode scan-sweeps the interior with the beam.
    pub fn uses_raster(self) -> bool {
        matches!(self, WriteMode::Raster | WriteMode::Hybrid)
    }
}

/// Cost breakdown for a candidate mode.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeCost {
    pub mode: WriteMode,
    pub write_time: f64,
    pub dose_error: f64,
    pub positioning_error: f64,
    pub jump_count: usize,
    pub acceleration_events: usize,
    pub cost: f64,
    /// Human-readable reason for the decision/penalty.
    pub reason: String,
}

/// Spatial tile containing geometry rings (exterior + interior) for analysis.
#[derive(Debug, Clone, PartialEq)]
pub struct Tile {
    pub index: (usize, usize),
    pub bounds: Rect,
    pub rings: Vec<Vec<Point>>,
}

impl Tile {
    pub fn area(&self) -> f64 {
        self.rings
            .iter()
            .map(|r: &Vec<Point>| ring_area(r.as_slice()))
            .sum()
    }

    pub fn perimeter(&self) -> f64 {
        self.rings
            .iter()
            .map(|r: &Vec<Point>| ring_perimeter(r.as_slice()))
            .sum()
    }

    pub fn fill_factor(&self) -> f64 {
        let w = self.bounds.width();
        let h = self.bounds.height();
        if w <= 0.0 || h <= 0.0 {
            0.0
        } else {
            let a = self.area().abs();
            (a / (w * h)).clamp(0.0, 1.0)
        }
    }
}

/// Clip rings to a rectangular tile (Sutherland–Hodgman-style polygon clip).
/// `ring` is a closed, counterclockwise-positive outline; holes may be
/// clockwise — the even-odd crossing in the rasterizer still holds as long as
/// the ring is a sequence of boundary points. Clipping by the axis-aligned tile
/// returns the polygonal intersection (possibly empty) by edge-wise clipping.
fn clip_ring_to_rect(ring: &[Point], bounds: Rect) -> Vec<Point> {
    let mut subject: Vec<Point> = ring.to_vec();
    // Sutherland–Hodgman against each edge: left, right, bottom, top.
    for edge in 0..4 {
        if subject.is_empty() {
            break;
        }
        let mut output: Vec<Point> = Vec::with_capacity(subject.len());
        for i in 0..subject.len() {
            let p = subject[i];
            let q = subject[(i + 1) % subject.len()];
            let p_in = point_inside_edge(p, edge, bounds);
            let q_in = point_inside_edge(q, edge, bounds);
            if q_in {
                if !p_in {
                    let intersect = intersect_edge(p, q, edge, bounds);
                    output.push(intersect);
                }
                output.push(q);
            } else if p_in {
                let intersect = intersect_edge(p, q, edge, bounds);
                output.push(intersect);
            }
        }
        subject = output;
    }
    // Remove near-duplicate consecutive points.
    let mut out: Vec<Point> = Vec::with_capacity(subject.len());
    for p in subject {
        if out.last().is_none_or(|l| l.distance(p) > 1e-9) {
            out.push(p);
        }
    }
    if out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    out
}

fn point_inside_edge(p: Point, edge: usize, bounds: Rect) -> bool {
    match edge {
        0 => p.x >= bounds.min.x - 1e-12,
        1 => p.x <= bounds.max.x + 1e-12,
        2 => p.y >= bounds.min.y - 1e-12,
        3 => p.y <= bounds.max.y + 1e-12,
        _ => true,
    }
}

fn intersect_edge(p: Point, q: Point, edge: usize, bounds: Rect) -> Point {
    let t = match edge {
        0 => (bounds.min.x - p.x) / ((q.x - p.x).max(1e-12)),
        1 => (bounds.max.x - p.x) / ((q.x - p.x).max(1e-12)),
        2 => (bounds.min.y - p.y) / ((q.y - p.y).max(1e-12)),
        3 => (bounds.max.y - p.y) / ((q.y - p.y).max(1e-12)),
        _ => 0.0,
    };
    let t = t.clamp(0.0, 1.0);
    Point::new(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y))
}

fn ring_area(ring: &[Point]) -> f64 {
    let mut a = 0.0;
    for i in 0..ring.len() {
        let p = ring[i];
        let q = ring[(i + 1) % ring.len()];
        a += p.x * q.y - q.x * p.y;
    }
    a.abs() / 2.0
}

fn ring_perimeter(ring: &[Point]) -> f64 {
    let mut l = 0.0;
    for i in 0..ring.len() {
        let p = ring[i];
        let q = ring[(i + 1) % ring.len()];
        l += p.distance(q);
    }
    l
}
