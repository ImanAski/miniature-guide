//! High-level planner API: partition, estimate, select mode, export reports.
//!
//! [`plan`] is the pre-submit checkpoint: it partitions the visible geometry,
//! evaluates every physically valid mode per tile, generates the actual
//! toolpaths the selected mode would run, and packages everything into a
//! [`Plan`] the UI can overlay, summarise, and export as a human-readable
//! report (text, CSV, or a self-contained SVG drawing).

use crate::config::PlannerConfig;
use crate::core::geo::{Point, Rect, Shape};
use crate::hybrid::cost::{evaluate, select_from};
use crate::hybrid::raster::{RasterLine, rasterize};
use crate::hybrid::segment::partition_into_tiles;
use crate::hybrid::vector::{
    Stroke, StrokeKind, order_strokes, serpentine_fill, strokes_to_shapes,
};
use crate::hybrid::{ModeCost, Tile, WriteMode};

/// Per-tile decision: every candidate cost plus the toolpaths the selected
/// mode will actually run (vector strokes and/or raster scan lines).
#[derive(Debug, Clone, PartialEq)]
pub struct TilePlan {
    pub tile: Tile,
    /// Costs in mode order `[Vector, Raster, Hybrid]`.
    pub costs: [ModeCost; 3],
    pub mode: WriteMode,
    /// Contour/fill strokes — populated when `mode.uses_vector()`.
    pub strokes: Vec<Stroke>,
    /// Scan lines — populated when `mode.uses_raster()`.
    pub raster_lines: Vec<RasterLine>,
}

impl TilePlan {
    /// The winning cost (alias for `costs[mode.index()]`).
    pub fn selected(&self) -> &ModeCost {
        &self.costs[self.mode.index()]
    }
}

/// A complete pre-submit plan: what will be written, how, and how long it
/// should take. Deterministic for a given geometry + configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub tiles: Vec<TilePlan>,
    /// Union of all tile bounds (the planned write area).
    pub bounds: Rect,
    pub tile_size_mm: f64,
    /// Sum of selected-mode write times (s).
    pub total_time: f64,
    /// Sum of selected-mode costs.
    pub total_cost: f64,
}

impl Plan {
    /// Number of tiles per mode, in `[Vector, Raster, Hybrid]` order.
    pub fn mode_counts(&self) -> [usize; 3] {
        let mut counts = [0usize; 3];
        for t in &self.tiles {
            counts[t.mode.index()] += 1;
        }
        counts
    }

    /// One-line summary for the status UI.
    pub fn summary(&self) -> String {
        let [v, r, h] = self.mode_counts();
        format!(
            "{} tiles · {v} vector · {r} raster · {h} hybrid · {:.3} s · cost {:.3}",
            self.tiles.len(),
            self.total_time,
            self.total_cost
        )
    }

    /// Convert the plan into executable shapes (tile row-major, vector
    /// strokes before raster spans within each tile). This is what a follow
    /// run of *the plan* would trace, as opposed to the raw input geometry.
    pub fn exposure_shapes(&self) -> Vec<Shape> {
        let mut out = Vec::new();
        for tp in &self.tiles {
            if tp.mode.uses_vector() {
                out.extend(strokes_to_shapes(&tp.strokes, None));
            }
            if tp.mode.uses_raster() {
                for line in &tp.raster_lines {
                    for &(x0, x1) in &line.spans {
                        out.push(Shape::new(
                            vec![crate::core::geo::PathElement::Line(
                                crate::core::geo::Line::new(
                                    Point::new(x0, line.y),
                                    Point::new(x1, line.y),
                                ),
                            )],
                            false,
                        ));
                    }
                }
            }
        }
        out
    }
}

/// Build the plan for `shapes` using `cfg`.
///
/// Pipeline: partition into tiles → evaluate all three modes per tile →
/// pick the cheapest feasible → generate that mode's toolpaths. Empty tiles
/// are dropped so the plan only describes real work.
pub fn plan(shapes: &[Shape], cfg: &PlannerConfig) -> Plan {
    let tiles = partition_into_tiles(shapes, cfg.tile_size_mm, cfg.max_tiles);
    let mut tile_plans: Vec<TilePlan> = Vec::with_capacity(tiles.len());
    let mut total_time = 0.0;
    let mut total_cost = 0.0;
    let mut bounds: Option<Rect> = None;

    for t in tiles {
        if t.rings.is_empty() {
            continue; // no geometry in this tile: no work
        }
        let costs = evaluate(&t, cfg);
        let selected = select_from(costs.clone());
        let mode = selected.mode;
        if selected.cost.is_finite() {
            total_time += selected.write_time;
            total_cost += selected.cost;
        }
        let (strokes, raster_lines) = toolpaths(&t, mode, cfg);
        bounds = Some(match bounds {
            Some(b) => b.union(&t.bounds),
            None => t.bounds,
        });
        tile_plans.push(TilePlan {
            tile: t,
            costs,
            mode,
            strokes,
            raster_lines,
        });
    }

    Plan {
        tiles: tile_plans,
        bounds: bounds.unwrap_or_else(|| Rect::from_min_max(0.0, 0.0, 1.0, 1.0)),
        tile_size_mm: cfg.tile_size_mm,
        total_time,
        total_cost,
    }
}

/// Generate the toolpaths `mode` will run for this tile.
///
/// Vector mode traces contours plus an interior serpentine fill (a filled
/// region must expose its whole area); raster mode sweeps scan lines; hybrid
/// does both — contours first, then the sweep.
fn toolpaths(tile: &Tile, mode: WriteMode, cfg: &PlannerConfig) -> (Vec<Stroke>, Vec<RasterLine>) {
    use crate::hybrid::segment::split_bulk_narrow;

    let pitch = cfg.raster.pitch_mm.max(1e-9);
    let mut strokes = Vec::new();
    let mut raster_lines = Vec::new();

    match mode {
        WriteMode::Vector => {
            // Contours for every ring plus an interior serpentine fill: a
            // filled region written vectorially must expose its whole area.
            for ring in &tile.rings {
                if let Some(s) = contour(ring) {
                    strokes.push(s);
                }
            }
            let fill_pitch = pitch.min(cfg.beam.spot_size_mm).max(1e-9);
            if let Some(fill) = serpentine_fill(&tile.rings, tile.bounds, fill_pitch) {
                strokes.push(fill);
            }
            strokes = order_strokes(strokes);
        }
        WriteMode::Raster => {
            raster_lines = rasterize(&tile.rings, tile.bounds, pitch, cfg.raster.bidirectional);
        }
        WriteMode::Hybrid => {
            // Same split the cost model charged for: sweep the bulk, trace
            // the narrow. The preview must match what the estimate assumed.
            let threshold = cfg.beam.minimum_feature_mm.max(pitch);
            let (bulk, narrow) = split_bulk_narrow(&tile.rings, &tile.bounds, threshold);
            for ring in &narrow {
                if let Some(s) = contour(ring) {
                    strokes.push(s);
                }
            }
            strokes = order_strokes(strokes);
            raster_lines = rasterize(&bulk, tile.bounds, pitch, cfg.raster.bidirectional);
        }
    }
    (strokes, raster_lines)
}

/// Closed contour stroke for a ring, or `None` when it is degenerate.
fn contour(ring: &[Point]) -> Option<Stroke> {
    if ring.len() < 3 {
        return None;
    }
    let mut pts = ring.to_vec();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    (pts.len() >= 3).then(|| Stroke::new(pts, true, StrokeKind::Contour))
}

// ── Report exports ────────────────────────────────────────────────────────

/// Human-readable diagnostics report: per-tile geometry, the full cost
/// comparison, the decision and its reason. Areas in µm², perimeters in µm.
pub fn diagnostics_report(plan: &Plan) -> String {
    let mut out = String::new();
    let [v, r, h] = plan.mode_counts();
    out.push_str("Hybrid Write Planner — Pre-Submit Report\n");
    out.push_str("========================================\n\n");
    out.push_str(&format!("Tiles:          {}\n", plan.tiles.len()));
    out.push_str(&format!("Tile size:      {:.3} mm\n", plan.tile_size_mm));
    out.push_str(&format!(
        "Mode mix:       {v} vector / {r} raster / {h} hybrid\n"
    ));
    out.push_str(&format!("Estimated time: {:.3} s\n", plan.total_time));
    out.push_str(&format!("Total cost:     {:.3}\n\n", plan.total_cost));

    for tp in &plan.tiles {
        let (i, j) = tp.tile.index;
        out.push_str(&format!("Tile ({i},{j})  [{}]\n", tp.mode.label()));
        let a = tp.tile.area() * 1e6; // mm² → µm²
        let p = tp.tile.perimeter() * 1e3; // mm → µm
        out.push_str(&format!(
            "  geometry: area {:.1} um2 · perimeter {:.1} um · fill {:.3} · rings {}\n",
            a,
            p,
            tp.tile.fill_factor(),
            tp.tile.rings.len()
        ));
        for c in &tp.costs {
            if c.cost.is_finite() {
                out.push_str(&format!(
                    "  {:<6}   time {:9.3} s · cost {:9.3} · jumps {} · accel {}\n",
                    c.mode.label(),
                    c.write_time,
                    c.cost,
                    c.jump_count,
                    c.acceleration_events
                ));
            } else {
                out.push_str(&format!(
                    "  {:<6}   REJECTED — {}\n",
                    c.mode.label(),
                    c.reason
                ));
            }
        }
        out.push_str(&format!(
            "  decision: {} — {}\n\n",
            tp.mode.label(),
            tp.selected().reason
        ));
    }
    out
}

/// Compact CSV: one row per tile with all three times/costs and the decision.
pub fn diagnostics_csv(plan: &Plan) -> String {
    let mut out = String::new();
    out.push_str(
        "tx,ty,area_um2,perimeter_um,fill_factor,mode,t_vector_s,t_raster_s,t_hybrid_s,\
         cost_vector,cost_raster,cost_hybrid,selected_cost,jumps,accel,reason\n",
    );
    for tp in &plan.tiles {
        let reason = tp.costs[tp.mode.index()].reason.replace(',', ";");
        let fmt = |c: &ModeCost| {
            if c.cost.is_finite() {
                format!("{:.6}", c.write_time)
            } else {
                "inf".to_string()
            }
        };
        let fmtc = |c: &ModeCost| {
            if c.cost.is_finite() {
                format!("{:.6}", c.cost)
            } else {
                "inf".to_string()
            }
        };
        out.push_str(&format!(
            "{},{},{:.3},{:.3},{:.4},{},{},{},{},{},{},{},{:.6},{},{},\"{}\"\n",
            tp.tile.index.0,
            tp.tile.index.1,
            tp.tile.area() * 1e6,
            tp.tile.perimeter() * 1e3,
            tp.tile.fill_factor(),
            tp.mode.label(),
            fmt(&tp.costs[WriteMode::Vector.index()]),
            fmt(&tp.costs[WriteMode::Raster.index()]),
            fmt(&tp.costs[WriteMode::Hybrid.index()]),
            fmtc(&tp.costs[WriteMode::Vector.index()]),
            fmtc(&tp.costs[WriteMode::Raster.index()]),
            fmtc(&tp.costs[WriteMode::Hybrid.index()]),
            tp.selected().cost,
            tp.selected().jump_count,
            tp.selected().acceleration_events,
            reason
        ));
    }
    out
}

/// Self-contained SVG drawing of the plan: input geometry faint, tiles tinted
/// by mode, generated toolpaths drawn on top, summary in the corner. Opens in
/// any browser — this is the visual "what will happen" report.
pub fn svg_report(plan: &Plan, shapes: &[Shape]) -> String {
    const MODE_HEX: [&str; 3] = ["#569cd6", "#ce9178", "#6a9955"]; // vector, raster, hybrid
    let b = plan.bounds;
    let pad = b.width().max(b.height()) * 0.05 + plan.tile_size_mm;
    let (x0, y0) = (b.min.x - pad, b.min.y - pad);
    let (x1, y1) = (b.max.x + pad, b.max.y + pad);
    let (w, h) = (x1 - x0, y1 - y0);

    // World (y-up) → SVG (y-down).
    let sx = |x: f64| (x - x0) * 1000.0;
    let sy = |y: f64| (y1 - y) * 1000.0;
    let stroke_w = (plan.tile_size_mm * 0.01).max(0.02);

    let mut out = String::new();
    out.push_str(&format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {vw} {vh}" font-family="monospace" font-size="16">
<rect width="{vw}" height="{vh}" fill="#1e1e1e"/>
"##,
        vw = (w * 1000.0) as u32,
        vh = (h * 1000.0) as u32,
    ));

    // Input geometry, faint.
    let tol = 0.01_f64;
    for s in shapes {
        for poly in flatten(s, tol) {
            if poly.len() < 2 {
                continue;
            }
            let pts: Vec<String> = poly
                .iter()
                .map(|p| format!("{:.2},{:.2}", sx(p.x), sy(p.y)))
                .collect();
            out.push_str(&format!(
                "<polyline points=\"{}\" fill=\"none\" stroke=\"#555\" stroke-width=\"{:.2}\"/>\n",
                pts.join(" "),
                stroke_w * 1000.0 * 0.5
            ));
        }
    }

    // Tiles: tinted rects + centre label.
    for tp in &plan.tiles {
        let r = tp.tile.bounds;
        let hex = MODE_HEX[tp.mode.index()];
        out.push_str(&format!(
            "<rect x=\"{:.2}\" y=\"{:.2}\" width=\"{:.2}\" height=\"{:.2}\" fill=\"{}\" fill-opacity=\"0.10\" stroke=\"{}\" stroke-width=\"{:.2}\"/>\n",
            sx(r.min.x),
            sy(r.max.y),
            r.width() * 1000.0,
            r.height() * 1000.0,
            hex,
            hex,
            stroke_w * 500.0
        ));
        let c = r.center();
        out.push_str(&format!(
            "<text x=\"{:.2}\" y=\"{:.2}\" fill=\"{}\" text-anchor=\"middle\" font-size=\"{}\">{}</text>\n",
            sx(c.x),
            sy(c.y) + 5.0,
            hex,
            (r.height() * 1000.0 * 0.18).clamp(8.0, 40.0),
            tp.mode.label()
        ));
    }

    // Toolpaths on top.
    for tp in &plan.tiles {
        let hex = MODE_HEX[tp.mode.index()];
        for st in &tp.strokes {
            if st.points.len() < 2 {
                continue;
            }
            let pts: Vec<String> = st
                .points
                .iter()
                .map(|p| format!("{:.2},{:.2}", sx(p.x), sy(p.y)))
                .collect();
            let close = if st.closed { " Z" } else { "" };
            out.push_str(&format!(
                "<polyline points=\"{}{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"{:.3}\" opacity=\"0.9\"/>\n",
                pts.join(" "),
                close,
                hex,
                stroke_w * 1000.0 * 0.6
            ));
        }
        for line in &tp.raster_lines {
            for &(a, bx) in &line.spans {
                out.push_str(&format!(
                    "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"{}\" stroke-width=\"{:.3}\" opacity=\"0.8\"/>\n",
                    sx(a),
                    sy(line.y),
                    sx(bx),
                    sy(line.y),
                    hex,
                    stroke_w * 1000.0 * 0.5
                ));
            }
        }
    }

    // Legend / summary, top-left inside the pad.
    let [v, r, h] = plan.mode_counts();
    out.push_str(&format!(
        "<text x=\"{:.2}\" y=\"{:.2}\" fill=\"#ddd\" font-size=\"22\">Pre-submit plan — {} tiles</text>\n",
        sx(plan.bounds.min.x),
        sy(plan.bounds.max.y) + 26.0,
        plan.tiles.len()
    ));
    out.push_str(&format!(
        "<text x=\"{:.2}\" y=\"{:.2}\" fill=\"#999\">{} vector · {} raster · {} hybrid · est. {:.3} s · cost {:.3}</text>\n",
        sx(plan.bounds.min.x),
        sy(plan.bounds.max.y) + 48.0,
        v,
        r,
        h,
        plan.total_time,
        plan.total_cost
    ));
    out.push_str("</svg>\n");
    out
}

/// Flatten a shape into polylines (lines and arc approximations).
fn flatten(shape: &Shape, tolerance: f64) -> Vec<Vec<Point>> {
    shape
        .elements
        .iter()
        .map(|e| match e {
            crate::core::geo::PathElement::Line(l) => vec![l.start, l.end],
            crate::core::geo::PathElement::Arc(a) => {
                a.to_segments(arc_segments(a.sweep.abs(), a.radius, tolerance))
            }
        })
        .collect()
}

fn arc_segments(sweep: f64, radius: f64, tolerance: f64) -> usize {
    let tol = tolerance.abs().max(1e-6);
    let r = radius.abs().max(1e-6);
    let chord_angle = (8.0 * tol / r).sqrt();
    (sweep.abs() / chord_angle).ceil().clamp(2.0, 512.0) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::{Line, PathElement};

    fn rect_shape(x0: f64, y0: f64, x1: f64, y1: f64) -> Shape {
        Shape::new(
            vec![
                PathElement::Line(Line::new(Point::new(x0, y0), Point::new(x1, y0))),
                PathElement::Line(Line::new(Point::new(x1, y0), Point::new(x1, y1))),
                PathElement::Line(Line::new(Point::new(x1, y1), Point::new(x0, y1))),
                PathElement::Line(Line::new(Point::new(x0, y1), Point::new(x0, y0))),
            ],
            true,
        )
    }

    fn open_line() -> Shape {
        Shape::new(
            vec![PathElement::Line(Line::new(
                Point::new(0.0, 0.0),
                Point::new(2.0, 0.0),
            ))],
            false,
        )
    }

    #[test]
    fn plan_is_deterministic() {
        let shapes = vec![
            rect_shape(0.0, 0.0, 1.0, 1.0),
            rect_shape(3.0, 0.0, 4.0, 1.0),
        ];
        let cfg = PlannerConfig::default();
        let a = plan(&shapes, &cfg);
        let b = plan(&shapes, &cfg);
        assert_eq!(a, b, "same input + config must give identical plans");
    }

    #[test]
    fn plan_drops_empty_tiles_and_keeps_geometry_tiles() {
        let shapes = vec![rect_shape(0.0, 0.0, 0.5, 0.5)];
        let cfg = PlannerConfig::default();
        let p = plan(&shapes, &cfg);
        assert!(!p.tiles.is_empty());
        assert!(
            p.tiles.iter().all(|t| !t.tile.rings.is_empty()),
            "empty tiles must be dropped"
        );
        assert!(p.total_time.is_finite() && p.total_time > 0.0);
        assert!(p.bounds.width() >= 0.5 - 1e-9);
    }

    #[test]
    fn selected_mode_matches_minimum_cost() {
        let shapes = vec![rect_shape(0.0, 0.0, 1.0, 1.0)];
        let cfg = PlannerConfig::default();
        let p = plan(&shapes, &cfg);
        for tp in &p.tiles {
            let winner = tp.costs[tp.mode.index()].cost;
            for c in &tp.costs {
                assert!(
                    winner <= c.cost + 1e-9,
                    "selected cost {} must not exceed {}",
                    winner,
                    c.cost
                );
            }
        }
    }

    #[test]
    fn vector_mode_generates_strokes_raster_generates_lines() {
        // Two features: a 1 mm square (bulk → swept) and a 0.5 µm strip
        // (narrow → traced). Hybrid only does "both" when both exist; a
        // bulk-only tile is raster work and gets no strokes.
        let shapes = vec![
            rect_shape(0.0, 0.0, 1.0, 1.0),
            rect_shape(0.0, 2.0, 2.0, 2.0005),
        ];
        let mut cfg = PlannerConfig::default();
        cfg.tile_size_mm = 4.0; // one tile covers everything

        // Force each mode by hand via toolpaths().
        let tiles = partition_into_tiles(&shapes, cfg.tile_size_mm, cfg.max_tiles);
        let t = tiles.iter().find(|t| !t.rings.is_empty()).unwrap();

        let (strokes, lines) = toolpaths(t, WriteMode::Vector, &cfg);
        assert!(!strokes.is_empty(), "vector mode must produce strokes");
        assert!(lines.is_empty(), "pure vector mode has no raster lines");

        let (strokes, lines) = toolpaths(t, WriteMode::Raster, &cfg);
        assert!(strokes.is_empty(), "pure raster mode has no strokes");
        assert!(!lines.is_empty(), "raster mode must produce scan lines");

        let (strokes, lines) = toolpaths(t, WriteMode::Hybrid, &cfg);
        assert!(!strokes.is_empty(), "hybrid traces the narrow strip");
        assert!(!lines.is_empty(), "hybrid sweeps the bulk square");
    }

    #[test]
    fn raster_never_selected_for_thin_geometry() {
        // A 0.5 µm wide strip: below the beam's minimum raster feature.
        let thin = rect_shape(0.0, 0.0, 2.0, 0.0005);
        let cfg = PlannerConfig::default();
        let p = plan(&[thin], &cfg);
        assert!(!p.tiles.is_empty());
        for tp in &p.tiles {
            assert_eq!(
                tp.mode,
                WriteMode::Vector,
                "sub-min-feature geometry must be written vectorially"
            );
        }
    }

    #[test]
    fn reports_are_well_formed() {
        let shapes = vec![rect_shape(0.0, 0.0, 1.0, 1.0)];
        let cfg = PlannerConfig::default();
        let p = plan(&shapes, &cfg);

        let report = diagnostics_report(&p);
        assert!(report.contains("Pre-Submit Report"));
        assert!(report.contains("decision:"));
        assert!(report.contains("Tile ("));

        let csv = diagnostics_csv(&p);
        let mut lines = csv.lines();
        let header = lines.next().unwrap();
        assert!(header.starts_with("tx,ty,"));
        assert!(header.contains("t_vector_s"));
        assert!(lines.all(|l| l.split(',').count() == header.split(',').count()));

        let svg = svg_report(&p, &shapes);
        assert!(svg.starts_with("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("VECTOR") || svg.contains("RASTER") || svg.contains("HYBRID"));
    }

    #[test]
    fn exposure_shapes_come_out_of_every_mode() {
        let shapes = vec![rect_shape(0.0, 0.0, 1.0, 1.0)];
        let cfg = PlannerConfig::default();
        let p = plan(&shapes, &cfg);
        let ex = p.exposure_shapes();
        assert!(!ex.is_empty(), "a plan for real geometry exposes something");
        assert!(ex.iter().all(|s| !s.elements.is_empty()));
    }

    #[test]
    fn open_shapes_do_not_break_the_plan() {
        // Parser front ends emit open paths (GDS PATH); planner must cope.
        let cfg = PlannerConfig::default();
        let p = plan(&[open_line(), rect_shape(5.0, 5.0, 6.0, 6.0)], &cfg);
        assert!(!p.tiles.is_empty());
        assert!(p.total_time.is_finite());
    }
}
