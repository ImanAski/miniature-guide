//! Mode selection and cost estimation.
//!
//! The decision is analytic, not heuristic: for each tile we estimate
//! `T_vector`, `T_raster`, `T_hybrid` (in seconds) and their costs
//! `C = alpha*T_write + beta*dose + gamma*pos + delta*jumps + epsilon*accel`
//! using the configured machine parameters. Hard constraints (minimum raster
//! feature, dose tolerance) rule a mode out before comparing costs.

use crate::config::PlannerConfig;
use crate::hybrid::raster::rasterize;
use crate::hybrid::vector::{Stroke as VStroke, StrokeKind, order_strokes, serpentine_fill, stats};
use crate::hybrid::{ModeCost, Tile, WriteMode};

fn clamp_speed(v: f64, maxv: f64) -> f64 {
    if !v.is_finite() || v <= 0.0 {
        maxv.max(1e-3)
    } else {
        v.min(maxv.max(1e-3))
    }
}

/// Vector fill pitch (mm): the tighter of the scan pitch and the beam spot —
/// vector mode must expose the area just as densely as raster does.
fn fill_pitch(cfg: &PlannerConfig) -> f64 {
    cfg.raster
        .pitch_mm
        .min(cfg.beam.spot_size_mm)
        .max(1e-9)
}

/// Trapezoid (constant accel + cruise + decel) move time for distance d.
/// When d is small, the move never reaches max velocity.
fn move_time(d: f64, v_max: f64, a: f64) -> f64 {
    if d <= 0.0 {
        return 0.0;
    }
    let a = a.abs().max(1e-3);
    let v_max = v_max.abs().max(1e-3);
    let dist_max = (v_max * v_max) / a; // distance to reach v_max
    if d >= 2.0 * dist_max {
        // accelerate + cruise + decel
        let t_accel = v_max / a;
        let d_cruise = d - 2.0 * dist_max;
        t_accel + t_accel + d_cruise / v_max
    } else {
        // triangular
        (2.0 * d / a).sqrt()
    }
}

pub fn cost_vector(tile: &Tile, cfg: &PlannerConfig) -> ModeCost {
    let mut reason: Vec<String> = Vec::new();
    let v_write = clamp_speed(cfg.vector.write_speed_mm_s, cfg.stage.max_velocity_mm_s);
    let v_jump = clamp_speed(cfg.vector.jump_speed_mm_s, cfg.stage.max_velocity_mm_s);
    let mut strokes: Vec<VStroke> = Vec::new();
    for ring in &tile.rings {
        if ring.len() < 3 {
            continue;
        }
        let mut pts = ring.clone();
        if pts.len() > 1 && *pts.first().unwrap() == *pts.last().unwrap() {
            pts.pop();
        }
        // contour stroke
        strokes.push(VStroke::new(pts.clone(), true, StrokeKind::Contour));
    }
    // Interior fill: a filled region written vectorially still has to expose
    // every point of its area, so charge the same physical coverage raster
    // gives (fill pitch = min(scan pitch, spot)), at vector speed.
    if let Some(fill) = serpentine_fill(&tile.rings, tile.bounds, fill_pitch(cfg)) {
        strokes.push(fill);
    }
    if strokes.is_empty() {
        return ModeCost {
            mode: WriteMode::Vector,
            write_time: 0.0,
            dose_error: 0.0,
            positioning_error: 0.1,
            jump_count: 0,
            acceleration_events: 0,
            cost: cfg.gamma * 0.1,
            reason: "no contours to trace".to_string(),
        };
    }
    let ordered = order_strokes(strokes);
    let s = stats(&ordered);
    let a = cfg.stage.max_acceleration_mm_s2;
    // Travel times, trapezoid model — no constant-velocity assumption.
    let t_write = move_time(s.write_distance, v_write, a)
        + cfg.stage.settling_time_s * (s.corner_count as f64);
    let t_jump = (s.jump_count as f64) * cfg.vector.jump_time_s
        + move_time(s.jump_distance, v_jump, a)
        + cfg.stage.settling_time_s * (s.jump_count as f64);
    let write_time = t_write + t_jump + cfg.vector.corner_time_s * (s.corner_count as f64);
    let accel_events = s.corner_count;
    let dose_error = 0.05;
    let pos_err = 0.02;
    let cost = cfg.alpha * write_time
        + cfg.beta * dose_error
        + cfg.gamma * pos_err
        + cfg.delta * (s.jump_count as f64)
        + cfg.epsilon * (accel_events as f64);
    reason.push(format!(
        "contours={}, jumps={}, write={:.3}mm",
        ordered.len(),
        s.jump_count,
        s.write_distance
    ));
    ModeCost {
        mode: WriteMode::Vector,
        write_time,
        dose_error,
        positioning_error: pos_err,
        jump_count: s.jump_count,
        acceleration_events: accel_events,
        cost,
        reason: reason.join("; "),
    }
}

pub fn cost_raster(tile: &Tile, cfg: &PlannerConfig) -> ModeCost {
    let mut reason: Vec<String> = Vec::new();
    let v = clamp_speed(cfg.raster.speed_mm_s, cfg.stage.max_velocity_mm_s);
    let pitch = cfg.raster.pitch_mm.max(1e-9);
    if tile.rings.is_empty() {
        return ModeCost {
            mode: WriteMode::Raster,
            write_time: 0.0,
            dose_error: 0.0,
            positioning_error: 0.05,
            jump_count: 0,
            acceleration_events: 0,
            cost: cfg.gamma * 0.05,
            reason: "no geometry — empty tile".to_string(),
        };
    }
    // min feature check
    let minf = tile
        .rings
        .iter()
        .map(|r| crate::hybrid::segment::min_feature_width_from_rings(&[r.clone()], &tile.bounds))
        .fold(f64::INFINITY, f64::min);
    if minf.is_finite() && minf < cfg.beam.minimum_feature_mm - 1e-9 {
        reason.push(format!(
            "min feature {:.3}um < {:.3}um -> reject",
            minf * 1000.0,
            cfg.beam.minimum_feature_mm * 1000.0
        ));
        return ModeCost {
            mode: WriteMode::Raster,
            write_time: f64::INFINITY,
            dose_error: f64::INFINITY,
            positioning_error: f64::INFINITY,
            jump_count: 0,
            acceleration_events: 0,
            cost: f64::INFINITY,
            reason: reason.join("; "),
        };
    }
    let lines = rasterize(&tile.rings, tile.bounds, pitch, cfg.raster.bidirectional);
    if lines.is_empty() {
        return ModeCost {
            mode: WriteMode::Raster,
            write_time: cfg.raster.setup_s,
            dose_error: 0.01,
            positioning_error: 0.01,
            jump_count: 0,
            acceleration_events: 0,
            cost: cfg.alpha * cfg.raster.setup_s,
            reason: "no exposed rows — trivial".to_string(),
        };
    }
    let mut t = cfg.raster.setup_s;
    let a = cfg.stage.max_acceleration_mm_s2;
    let mut accel = 0usize;
    let overscan = cfg.raster.overscan_mm;
    for l in &lines {
        let (x0, x1) = match l.spans.first() {
            Some(s) => (s.0, s.1),
            None => (l.start_x(), l.end_x()),
        };
        let span = (x1 - x0).abs();
        let exp = (span + 2.0 * overscan).max(1e-9);
        // Each span starts and stops at the ends (flyback/settling between
        // rows), so charge a full trapezoid rather than cruise speed.
        t += move_time(exp, v, a);
        accel += 1;
    }
    // Flyback: unidirectional mode re-travels the row width every line,
    // bidirectional mode only on alternate pairs.
    let jumps = if cfg.raster.bidirectional {
        lines.len().saturating_sub(1) / 2
    } else {
        lines.len().saturating_sub(1)
    };
    let flyback_d = (tile.bounds.max.x - tile.bounds.min.x).abs();
    t += (jumps as f64) * cfg.raster.flyback_s + move_time(flyback_d, v, a) * (jumps as f64);
    let dose_error = dose_ripple(cfg);
    let pos_err = 0.03;
    if dose_error > cfg.dose_tolerance {
        return ModeCost {
            mode: WriteMode::Raster,
            write_time: t,
            dose_error,
            positioning_error: pos_err,
            jump_count: jumps,
            acceleration_events: accel,
            cost: f64::INFINITY,
            reason: format!(
                "dose error {:.3} > tol {:.3}",
                dose_error, cfg.dose_tolerance
            ),
        };
    }
    let cost = cfg.alpha * t
        + cfg.beta * dose_error
        + cfg.gamma * pos_err
        + cfg.delta * (jumps as f64)
        + cfg.epsilon * (accel as f64);
    ModeCost {
        mode: WriteMode::Raster,
        write_time: t,
        dose_error,
        positioning_error: pos_err,
        jump_count: jumps,
        acceleration_events: accel,
        cost,
        reason: format!(
            "{} rows, pitch {:.3}um, overscan {:.3}um",
            lines.len(),
            pitch * 1000.0,
            overscan * 1000.0
        ),
    }
}

/// Hybrid: raster-sweep the bulk regions, vector-trace the narrow ones.
///
/// Rejected outright when there is no bulk to sweep (then it would just be
/// vector work with a misleading label) or when the bulk sweep would break
/// the dose tolerance. Rasterising a narrow feature is exactly what the
/// min-feature constraint exists to prevent, so the split happens *before*
/// any raster time is charged.
pub fn cost_hybrid(tile: &Tile, cfg: &PlannerConfig) -> ModeCost {
    let threshold = cfg.beam.minimum_feature_mm.max(cfg.raster.pitch_mm);
    let (bulk, narrow) =
        crate::hybrid::segment::split_bulk_narrow(&tile.rings, &tile.bounds, threshold);

    if bulk.is_empty() {
        return ModeCost {
            mode: WriteMode::Hybrid,
            write_time: f64::INFINITY,
            dose_error: f64::INFINITY,
            positioning_error: f64::INFINITY,
            jump_count: 0,
            acceleration_events: 0,
            cost: f64::INFINITY,
            reason: format!(
                "no bulk regions (all {} rings narrower than {:.3}um)",
                tile.rings.len(),
                threshold * 1000.0
            ),
        };
    }

    let v_write = clamp_speed(cfg.vector.write_speed_mm_s, cfg.stage.max_velocity_mm_s);
    // Raster part: sweep the bulk rings only.
    let pitch = cfg.raster.pitch_mm.max(1e-9);
    let lines = rasterize(&bulk, tile.bounds, pitch, cfg.raster.bidirectional);
    let mut t_r = cfg.raster.setup_s;
    let overscan = cfg.raster.overscan_mm;
    let v = clamp_speed(cfg.raster.speed_mm_s, cfg.stage.max_velocity_mm_s);
    let a = cfg.stage.max_acceleration_mm_s2;
    for l in &lines {
        for (x0, x1) in &l.spans {
            let exp = ((x1 - x0).abs() + 2.0 * overscan).max(1e-9);
            t_r += move_time(exp, v, a);
        }
    }
    let flybacks = if cfg.raster.bidirectional {
        lines.len().saturating_sub(1) / 2
    } else {
        lines.len().saturating_sub(1)
    };
    t_r += (flybacks as f64) * cfg.raster.flyback_s;

    // Dose check applies to the bulk sweep (the narrow part is vector work).
    let dose_error = if lines.is_empty() {
        0.02
    } else {
        dose_ripple(cfg)
    };
    if dose_error > cfg.dose_tolerance {
        return ModeCost {
            mode: WriteMode::Hybrid,
            write_time: t_r,
            dose_error,
            positioning_error: 0.03,
            jump_count: flybacks,
            acceleration_events: lines.len(),
            cost: f64::INFINITY,
            reason: format!(
                "bulk dose ripple {:.3} > tol {:.3}",
                dose_error, cfg.dose_tolerance
            ),
        };
    }

    // Vector part: contour the narrow rings (their width is below the spot
    // pitch, so the trace itself covers the feature — no fill needed).
    let mut strokes: Vec<VStroke> = Vec::new();
    for ring in &narrow {
        if ring.len() < 3 {
            continue;
        }
        let mut pts = ring.clone();
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        strokes.push(VStroke::new(pts, true, StrokeKind::Contour));
    }
    let ordered = order_strokes(strokes);
    let s = stats(&ordered);
    let t_v = move_time(s.write_distance, v_write, a)
        + move_time(s.jump_distance, v_write, a)
        + (s.corner_count as f64) * cfg.vector.corner_time_s
        + (s.jump_count as f64) * cfg.vector.jump_time_s
        + cfg.stage.settling_time_s * (s.jump_count as f64);

    let write_time = t_r + t_v;
    let pos_err = 0.02;
    let cost = cfg.alpha * write_time
        + cfg.beta * dose_error
        + cfg.gamma * pos_err
        + cfg.delta * ((flybacks + s.jump_count) as f64)
        + cfg.epsilon * ((lines.len() + s.corner_count) as f64);
    let reason = format!(
        "bulk: {} rows · narrow: {} contours ({} rings raster / {} vector)",
        lines.len(),
        ordered.len(),
        bulk.len(),
        narrow.len()
    );
    ModeCost {
        mode: WriteMode::Hybrid,
        write_time,
        dose_error,
        positioning_error: pos_err,
        jump_count: flybacks + s.jump_count,
        acceleration_events: lines.len() + s.corner_count,
        cost,
        reason,
    }
}

/// Scanline dose ripple model (0..1): the modulation left between passes at
/// `pitch` given a Gaussian-ish spot of `spot_size`. Deterministic, monotone
/// in pitch/spot — used by both raster and hybrid feasibility checks.
pub fn dose_ripple(cfg: &PlannerConfig) -> f64 {
    let pitch = cfg.raster.pitch_mm.max(1e-12);
    let spot = cfg.beam.spot_size_mm.max(1e-12);
    let x = std::f64::consts::PI * spot / (2.0 * pitch);
    (-x * x).exp().clamp(0.0, 1.0)
}

/// Cost every physically valid mode for this tile: `(vector, raster, hybrid)`.
/// The report uses this to show the comparison; [`select_mode`] picks the best.
pub fn evaluate(tile: &Tile, cfg: &PlannerConfig) -> [ModeCost; 3] {
    [
        cost_vector(tile, cfg),
        cost_raster(tile, cfg),
        cost_hybrid(tile, cfg),
    ]
}

/// Pick the cheapest finite mode from an [`evaluate`]d cost triple.
/// Ties (within 1e-12) favour the earlier mode: Vector, then Raster, then
/// Hybrid. When every mode is infeasible fall back to Vector so the plan is
/// still executable geometry-wise; the reason strings explain the rejection.
pub fn select_from(costs: [ModeCost; 3]) -> ModeCost {
    let mut best = 0usize;
    for i in 1..costs.len() {
        if costs[i].cost < costs[best].cost - 1e-12 {
            best = i;
        }
    }
    if costs[best].cost.is_finite() {
        costs[best].clone()
    } else {
        costs[0].clone()
    }
}

pub fn select_mode(tile: &Tile, cfg: &PlannerConfig) -> ModeCost {
    select_from(evaluate(tile, cfg))
}
