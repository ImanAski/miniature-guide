//! Scanline raster generation.
//!
//! The rasterizer walks horizontal rows at the configured beam pitch and
//! reports, per row, the *exposed* spans inside the geometry (even-odd
//! crossing against every ring, so holes fall out for free). Rows that cross
//! nothing are dropped entirely — an empty bounding box costs nothing, which
//! is what keeps sparse layouts from paying for their own emptiness.
//!
//! Dose comes from the caller's config, not from constants here: pitch,
//! overscan, scan direction and speeds all live in
//! [`crate::config::RasterConfig`]. The *timing* of the lines produced here is
//! modelled in [`crate::hybrid::cost`], next to the other mode costs, so the
//! geometry and the cost model stay independently testable.

use crate::core::geo::Rect;

/// One non-empty scan line: the spans the beam exposes at height `y`.
#[derive(Debug, Clone, PartialEq)]
pub struct RasterLine {
    /// Row height (mm).
    pub y: f64,
    /// Exposed intervals `(x0, x1)` with `x0 < x1`, sorted, non-overlapping.
    pub spans: Vec<(f64, f64)>,
    /// Scan direction on this row (`true` = left→right). Always `true` when
    /// unidirectional scanning is configured.
    pub forward: bool,
}

impl RasterLine {
    /// First exposed x on this row.
    pub fn start_x(&self) -> f64 {
        self.spans.first().map_or(0.0, |s| s.0)
    }

    /// Last exposed x on this row.
    pub fn end_x(&self) -> f64 {
        self.spans.last().map_or(0.0, |s| s.1)
    }

    /// Total exposed length on this row (mm).
    pub fn exposed_length(&self) -> f64 {
        self.spans.iter().map(|(a, b)| b - a).sum()
    }
}

/// Row heights covered by `bbox` on the pitch grid: `(k + 0.5) * pitch`.
///
/// The half-pitch offset keeps rows off integer multiples of the pitch, which
/// is where polygon vertices most often sit; a row exactly through a vertex
/// would otherwise depend on rounding. Returns an empty iterator when the box
/// is thinner than one pitch — callers rely on the min-feature constraint to
/// have rejected raster in that case already.
pub fn row_positions(bbox: Rect, pitch: f64) -> Vec<f64> {
    if pitch <= 0.0 || bbox.is_empty() {
        return Vec::new();
    }
    let k_min = ((bbox.min.y / pitch) - 0.5).ceil();
    let k_max = ((bbox.max.y / pitch) - 0.5).floor();
    if !k_min.is_finite() || !k_max.is_finite() || k_max < k_min {
        return Vec::new();
    }
    // Defensive cap: a sane layout never approaches this, and it turns a
    // pathological pitch into a deterministic (if pessimistic) plan.
    const MAX_ROWS: f64 = 2_000_000.0;
    let k_max = k_max.min(k_min + MAX_ROWS);
    let mut out = Vec::with_capacity((k_max - k_min + 1.0) as usize);
    let mut k = k_min;
    while k <= k_max {
        out.push((k + 0.5) * pitch);
        k += 1.0;
    }
    out
}

/// Sorted x coordinates where the rings cross height `y` (even-odd rule).
///
/// The half-open comparison `(p.y > y) != (q.y > y)` counts a vertex that sits
/// exactly on the row once instead of twice, so crossings stay paired.
fn crossings(rings: &[Vec<crate::core::geo::Point>], y: f64) -> Vec<f64> {
    let mut xs: Vec<f64> = Vec::new();
    for ring in rings {
        if ring.len() < 3 {
            continue;
        }
        for i in 0..ring.len() {
            let p = ring[i];
            let q = ring[(i + 1) % ring.len()];
            if (p.y > y) == (q.y > y) {
                continue;
            }
            let t = (y - p.y) / (q.y - p.y);
            xs.push(p.x + t * (q.x - p.x));
        }
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xs
}

/// Exposed spans of `rings` at height `y`: sorted, merged, degenerate-free.
pub fn spans_at(rings: &[Vec<crate::core::geo::Point>], y: f64) -> Vec<(f64, f64)> {
    let xs = crossings(rings, y);
    let mut spans: Vec<(f64, f64)> = xs
        .chunks_exact(2)
        .filter(|c| c[1] - c[0] > 1e-12)
        .map(|c| (c[0], c[1]))
        .collect();
    // An odd crossing count means a vertex grazed the row; the trailing
    // unpaired crossing contributes no area, so dropping it is safe.
    // Merge spans that touch (shared boundary between abutting regions).
    let mut merged: Vec<(f64, f64)> = Vec::with_capacity(spans.len());
    for s in spans.drain(..) {
        match merged.last_mut() {
            Some(last) if s.0 <= last.1 + 1e-9 => last.1 = last.1.max(s.1),
            _ => merged.push(s),
        }
    }
    merged
}

/// Rasterize `rings` over `bbox` at `pitch`.
///
/// `bidirectional` alternates the scan direction row by row; unidirectional
/// mode always scans left→right (the caller's cost model then charges the
/// flyback). Rows without geometry are omitted.
pub fn rasterize(
    rings: &[Vec<crate::core::geo::Point>],
    bbox: Rect,
    pitch: f64,
    bidirectional: bool,
) -> Vec<RasterLine> {
    let mut out = Vec::new();
    for (i, y) in row_positions(bbox, pitch).into_iter().enumerate() {
        let spans = spans_at(rings, y);
        if spans.is_empty() {
            continue;
        }
        out.push(RasterLine {
            y,
            spans,
            forward: if bidirectional { i % 2 == 1 } else { true },
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::Point;

    fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Point> {
        vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ]
    }

    #[test]
    fn spans_split_at_a_hole() {
        let outer = square(0.0, 0.0, 10.0, 10.0);
        let hole = square(4.0, 4.0, 6.0, 6.0); // reversed orientation irrelevant: even-odd
        let spans = spans_at(&[outer, hole], 5.0);
        assert_eq!(spans.len(), 2, "hole splits the row: {spans:?}");
        assert!((spans[0].0 - 0.0).abs() < 1e-9 && (spans[0].1 - 4.0).abs() < 1e-9);
        assert!((spans[1].0 - 6.0).abs() < 1e-9 && (spans[1].1 - 10.0).abs() < 1e-9);
    }

    #[test]
    fn vertex_on_the_row_is_counted_once() {
        // Diamond: vertices at (5,0),(10,5),(5,10),(0,5). A row through y=5
        // passes exactly through two vertices; the result must stay paired.
        let diamond = vec![
            Point::new(5.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(5.0, 10.0),
            Point::new(0.0, 5.0),
        ];
        let spans = spans_at(&[diamond], 5.0);
        assert_eq!(spans.len(), 1, "{spans:?}");
        assert!((spans[0].0 - 0.0).abs() < 1e-9 && (spans[0].1 - 10.0).abs() < 1e-9);
    }

    #[test]
    fn empty_rows_are_skipped() {
        let rings = vec![square(0.0, 0.0, 1.0, 1.0)];
        let lines = rasterize(&rings, Rect::from_min_max(0.0, 0.0, 5.0, 5.0), 0.5, true);
        assert!(!lines.is_empty());
        assert!(
            lines.iter().all(|l| l.y >= 0.0 && l.y <= 1.0),
            "rows outside the geometry must be dropped: {lines:?}"
        );
    }

    #[test]
    fn direction_alternates_only_when_bidirectional() {
        let rings = vec![square(0.0, 0.0, 1.0, 1.0)];
        let bi = rasterize(&rings, Rect::from_min_max(0.0, 0.0, 1.0, 1.0), 0.25, true);
        let uni = rasterize(&rings, Rect::from_min_max(0.0, 0.0, 1.0, 1.0), 0.25, false);
        assert!(bi.iter().any(|l| !l.forward), "bidirectional must reverse");
        assert!(uni.iter().all(|l| l.forward), "unidirectional must not");
        assert_eq!(bi.len(), uni.len(), "direction must not change the rows");
    }

    #[test]
    fn row_positions_are_on_the_half_pitch_grid() {
        let rows = row_positions(Rect::from_min_max(0.0, 0.0, 1.0, 1.0), 0.5);
        assert_eq!(rows, vec![0.25, 0.75], "half-pitch offset, {rows:?}");
    }

    #[test]
    fn thin_box_yields_no_rows() {
        assert!(row_positions(Rect::from_min_max(0.0, 0.0, 1.0, 0.1), 0.5).is_empty());
    }
}
