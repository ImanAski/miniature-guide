//! Vector path generation: contour/fill strokes, vertex reduction, ordering.
//!
//! Vector writing decomposes a region into *strokes* — polylines the beam
//! traces with exposure on ([`StrokeKind::Contour`], [`StrokeKind::Path`]) or
//! interior fill sweeps ([`StrokeKind::Fill`]). Fills are boustrophedon
//! (serpentine) sweeps over rows at the beam-limited pitch: a filled square is
//! not four edges, it has to be exposed everywhere, so vector mode carries its
//! own fill strokes. That keeps vector and raster costs comparable — both
//! expose the same physical area, differing in speeds, overheads and contour
//! work, which is exactly what lets machine parameters flip a decision.
//!
//! Ordering is nearest-neighbour (seeded deterministically, ties broken by
//! input index) and swappable: [`order_strokes`] is a plain function over a
//! list, so a better TSP-style optimizer can replace it without touching the
//! cost model.

use crate::core::geo::{Line, PathElement, Point, Rect, Shape};

use super::raster::{row_positions, spans_at};

/// What a stroke exposes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrokeKind {
    /// A region outline (closed) or an open path from the layout.
    Contour,
    /// Interior sweep filling a region.
    Fill,
}

/// One polyline traced with a fixed exposure state.
#[derive(Debug, Clone, PartialEq)]
pub struct Stroke {
    /// Vertices; for closed strokes the closing vertex is implicit.
    pub points: Vec<Point>,
    pub closed: bool,
    pub kind: StrokeKind,
}

impl Stroke {
    pub fn new(points: Vec<Point>, closed: bool, kind: StrokeKind) -> Self {
        Stroke {
            points,
            closed,
            kind,
        }
    }

    /// Beam-on path length (mm); closed strokes include the closing segment.
    pub fn length(&self) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }
        let mut len = 0.0;
        for w in self.points.windows(2) {
            len += w[0].distance(w[1]);
        }
        if self.closed {
            let last = *self.points.last().expect("checked");
            len += last.distance(self.points[0]);
        }
        len
    }

    /// Where the stage enters this stroke.
    pub fn entry(&self) -> Point {
        self.points.first().copied().unwrap_or(Point::ORIGIN)
    }

    /// Where the stage leaves (closed strokes end where they start).
    pub fn exit(&self) -> Point {
        match (self.closed, self.points.last()) {
            (true, Some(_)) => self.entry(),
            (_, Some(&p)) => p,
            _ => Point::ORIGIN,
        }
    }

    /// Rotate a closed stroke so it starts at the vertex nearest `p` — the
    /// stage can enter a ring anywhere, so use that freedom to cut travel.
    pub fn rotate_to_nearest(&mut self, p: Point) {
        if !self.closed || self.points.len() < 2 {
            return;
        }
        let (mut best, mut best_d) = (0usize, f64::INFINITY);
        for (i, v) in self.points.iter().enumerate() {
            let d = p.distance(*v);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        self.points.rotate_left(best);
    }

    /// Reverse an open stroke (direction is free for open paths).
    pub fn reversed(mut self) -> Self {
        if !self.closed {
            self.points.reverse();
        }
        self
    }

    /// Number of direction changes: every vertex of a reduced closed ring,
    /// the interior vertices of an open one. Endpoints are arrivals/departures,
    /// not corners.
    pub fn corner_count(&self) -> usize {
        if self.closed {
            self.points.len()
        } else {
            self.points.len().saturating_sub(2)
        }
    }
}

/// Remove vertices that lie (within `eps`) on the straight line between their
/// neighbours — the path they describe is unchanged to within `eps`, so the
/// exposure is too. Reversals (t outside the segment) are never removed.
pub fn drop_collinear(points: &[Point], closed: bool, eps: f64) -> Vec<Point> {
    // De-duplicate consecutive points first; near-identical vertices would
    // otherwise make the line test degenerate.
    let mut pts: Vec<Point> = Vec::with_capacity(points.len());
    for &p in points {
        if pts.last().is_none_or(|l| l.distance(p) > 1e-12) {
            pts.push(p);
        }
    }
    if closed && pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    let n = pts.len();
    let min_len = if closed { 3 } else { 2 };
    if n <= min_len {
        return pts;
    }

    let collinear = |a: Point, b: Point, c: Point| -> bool {
        let ac = c - a;
        let len = ac.length();
        if len <= 1e-12 {
            return false;
        }
        let ab = b - a;
        let t = (ab.x * ac.x + ab.y * ac.y) / (len * len);
        if t <= 0.0 || t >= 1.0 {
            return false; // reversal or extrapolation: keep the vertex
        }
        let proj = a + ac * t;
        b.distance(proj) <= eps
    };

    if closed {
        let mut keep = vec![true; n];
        for i in 0..n {
            let a = pts[(i + n - 1) % n];
            let c = pts[(i + 1) % n];
            if collinear(a, pts[i], c) {
                keep[i] = false;
            }
        }
        let kept: Vec<Point> = pts
            .iter()
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(_, p)| *p)
            .collect();
        if kept.len() >= 3 {
            kept
        } else {
            pts // degenerate ring: reduction must not erase it
        }
    } else {
        let mut keep = vec![true; n];
        for i in 1..n - 1 {
            let a = pts[i - 1];
            let b = pts[i];
            let c = pts[i + 1];
            if collinear(a, b, c) {
                keep[i] = false;
            }
        }
        pts.iter()
            .enumerate()
            .filter(|(_i, _p)| keep[*_i])
            .map(|(_i, p)| *p)
            .collect()
    }
}

/// Interior sweep of `rings`: one serpentine stroke over every row at
/// `pitch` that exposes something, alternating direction row by row.
///
/// Returns `None` when no row lands inside the geometry (the caller then has
/// no fill work — a case the min-feature constraint has already vetted).
pub fn serpentine_fill(rings: &[Vec<Point>], bbox: Rect, pitch: f64) -> Option<Stroke> {
    let mut points: Vec<Point> = Vec::new();
    let mut forward = true;
    for y in row_positions(bbox, pitch) {
        let spans = spans_at(rings, y);
        if spans.is_empty() {
            continue;
        }
        let (x0, x1) = (
            spans.first().expect("non-empty").0,
            spans.last().expect("non-empty").1,
        );
        if forward {
            points.push(Point::new(x0, y));
            points.push(Point::new(x1, y));
        } else {
            points.push(Point::new(x1, y));
            points.push(Point::new(x0, y));
        }
        forward = !forward;
    }
    let points = drop_collinear(&points, false, 0.0);
    (points.len() >= 2).then(|| Stroke::new(points, false, StrokeKind::Fill))
}

/// Nearest-neighbour ordering of independent strokes.
///
/// Seed is the first stroke (input order is deterministic); ties go to the
/// lower input index, open strokes may be reversed, closed strokes rotate to
/// the vertex nearest the current position. Deterministic for a given input.
pub fn order_strokes(strokes: Vec<Stroke>) -> Vec<Stroke> {
    if strokes.len() <= 1 {
        return strokes;
    }
    let mut remaining = strokes;
    let mut ordered = Vec::with_capacity(remaining.len());
    ordered.push(remaining.remove(0));
    let mut pos = ordered[0].exit();

    while !remaining.is_empty() {
        let (mut best_i, mut best_d, mut best_rev) = (0usize, f64::INFINITY, false);
        for (i, s) in remaining.iter().enumerate() {
            let (d, rev) = if s.closed {
                let nearest = s
                    .points
                    .iter()
                    .map(|v| pos.distance(*v))
                    .fold(f64::INFINITY, f64::min);
                (nearest, false)
            } else {
                let forward = pos.distance(s.entry());
                let backward = pos.distance(s.exit());
                if backward < forward {
                    (backward, true)
                } else {
                    (forward, false)
                }
            };
            if d < best_d - 1e-12 {
                best_d = d;
                best_i = i;
                best_rev = rev;
            }
        }
        let mut next = remaining.remove(best_i);
        if next.closed {
            next.rotate_to_nearest(pos);
        } else if best_rev {
            next = next.reversed();
        }
        pos = next.exit();
        ordered.push(next);
    }
    ordered
}

/// Travel statistics of an ordered stroke list.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VectorStats {
    /// Total beam-on path length (mm).
    pub write_distance: f64,
    /// Travels between independent strokes (`strokes - 1`).
    pub jump_count: usize,
    /// Total beam-off travel distance (mm).
    pub jump_distance: f64,
    /// Direction changes across all strokes.
    pub corner_count: usize,
}

pub fn stats(ordered: &[Stroke]) -> VectorStats {
    let mut out = VectorStats {
        corner_count: ordered.iter().map(Stroke::corner_count).sum(),
        ..Default::default()
    };
    for s in ordered {
        out.write_distance += s.length();
    }
    for w in ordered.windows(2) {
        out.jump_count += 1;
        out.jump_distance += w[0].exit().distance(w[1].entry());
    }
    out
}

/// Convert ordered strokes back into renderable/executable shapes.
pub fn strokes_to_shapes(ordered: &[Stroke], layer: Option<String>) -> Vec<Shape> {
    ordered
        .iter()
        .map(|s| {
            let elements: Vec<PathElement> = match (s.closed, s.points.len()) {
                (true, n) if n >= 3 => (0..n)
                    .map(|i| PathElement::Line(Line::new(s.points[i], s.points[(i + 1) % n])))
                    .collect(),
                (_, n) if n >= 2 => s
                    .points
                    .windows(2)
                    .map(|w| PathElement::Line(Line::new(w[0], w[1])))
                    .collect(),
                _ => Vec::new(),
            };
            let mut shape = Shape::new(elements, s.closed);
            shape.layer = layer.clone();
            shape
        })
        .filter(|s: &Shape| !s.elements.is_empty())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pts(ps: &[(f64, f64)]) -> Vec<Point> {
        ps.iter().map(|&(x, y)| Point::new(x, y)).collect()
    }

    #[test]
    fn collinear_vertex_is_removed_but_corners_survive() {
        let straight = pts(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]);
        let reduced = drop_collinear(&straight, false, 1e-6);
        assert_eq!(reduced.len(), 2, "{reduced:?}");

        let corner = pts(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        let kept = drop_collinear(&corner, false, 1e-6);
        assert_eq!(kept.len(), 3, "a real corner must survive: {kept:?}");
    }

    #[test]
    fn collinear_reduction_respects_the_tolerance() {
        // 1 µm deviation from the straight line.
        let nearly = pts(&[(0.0, 0.0), (1.0, 0.001), (2.0, 0.0)]);
        assert_eq!(
            drop_collinear(&nearly, false, 5e-4).len(),
            3,
            "kept at 0.5 µm eps"
        );
        assert_eq!(
            drop_collinear(&nearly, false, 2e-3).len(),
            2,
            "removed at 2 µm eps"
        );
    }

    #[test]
    fn closed_reduction_never_erases_the_ring() {
        let line = pts(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]);
        let reduced = drop_collinear(&line, true, 1e-9);
        assert!(reduced.len() >= 3, "degenerate ring stays: {reduced:?}");
    }

    #[test]
    fn reversal_vertices_are_preserved() {
        // A hairpin: the middle vertex is a reversal, never collinear-in-segment.
        let hairpin = pts(&[(0.0, 0.0), (10.0, 0.0), (0.0, 1e-6)]);
        assert_eq!(drop_collinear(&hairpin, false, 1e-3).len(), 3);
    }

    #[test]
    fn serpentine_alternates_and_skips_empty_rows() {
        let ring = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ];
        let bbox = Rect::from_min_max(0.0, 0.0, 1.0, 1.0);
        let fill = serpentine_fill(&[ring], bbox, 0.5).expect("fill exists");
        assert!(!fill.closed);
        assert_eq!(fill.kind, StrokeKind::Fill);
        // Row 1 (y=0.25) sweeps 0→1, row 2 (y=0.75) sweeps 1→0.
        let rows: Vec<(f64, f64, f64)> = fill
            .points
            .chunks(2)
            .filter(|c| c.len() == 2)
            .map(|c| (c[0].y, c[0].x, c[1].x))
            .collect();
        assert_eq!(rows.len(), 2, "{rows:?}");
        assert_eq!(rows[0], (0.25, 0.0, 1.0), "first row sweeps left→right");
        assert_eq!(rows[1], (0.75, 1.0, 0.0), "second row sweeps right→left");
    }

    #[test]
    fn ordering_ties_go_to_input_index_and_cut_travel() {
        let a = Stroke::new(pts(&[(0.0, 0.0), (1.0, 0.0)]), false, StrokeKind::Contour);
        let b = Stroke::new(pts(&[(5.0, 0.0), (6.0, 0.0)]), false, StrokeKind::Contour);
        let c = Stroke::new(pts(&[(2.0, 0.0), (3.0, 0.0)]), false, StrokeKind::Contour);
        let ordered = order_strokes(vec![a, b, c]);
        let starts: Vec<Point> = ordered.iter().map(Stroke::entry).collect();
        assert_eq!(
            starts,
            pts(&[(0.0, 0.0), (2.0, 0.0), (5.0, 0.0)]),
            "nearest neighbour order"
        );
        let s = stats(&ordered);
        assert_eq!(s.jump_count, 2);
        assert!((s.jump_distance - 3.0).abs() < 1e-9, "{}", s.jump_distance);
    }

    #[test]
    fn ordering_may_reverse_open_strokes() {
        let a = Stroke::new(pts(&[(0.0, 0.0), (1.0, 0.0)]), false, StrokeKind::Contour);
        // Enters at (10,0): reversed entry at (9,0) is nearer than (10,0)… no:
        // from (1,0) the forward entry (10,0) is nearer than (9,0)? 9 < 10 yes.
        let b = Stroke::new(pts(&[(10.0, 0.0), (9.0, 0.0)]), false, StrokeKind::Contour);
        let ordered = order_strokes(vec![a, b]);
        assert_eq!(
            ordered[1].entry(),
            Point::new(9.0, 0.0),
            "reversed to enter near"
        );
    }

    #[test]
    fn closed_strokes_rotate_to_the_nearest_vertex() {
        let ring = Stroke::new(
            pts(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]),
            true,
            StrokeKind::Contour,
        );
        let from = Stroke::new(pts(&[(9.5, 9.5), (9.6, 9.5)]), false, StrokeKind::Contour);
        let ordered = order_strokes(vec![from, ring]);
        assert_eq!(
            ordered[1].entry(),
            Point::new(10.0, 10.0),
            "ring starts at the nearest corner"
        );
    }

    #[test]
    fn fill_length_covers_the_exposed_area() {
        // 1 × 1 mm square, 0.5 mm pitch: two sweeps of ~1 mm + connectors.
        let ring = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ];
        let bbox = Rect::from_min_max(0.0, 0.0, 1.0, 1.0);
        let fill = serpentine_fill(&[ring], bbox, 0.5).unwrap();
        let len = fill.length();
        assert!(
            (2.0..3.0).contains(&len),
            "≈2 sweeps + 1 connector, got {len}"
        );
        // Serpentine beats a naive there-and-back: naive would be ~4.
        assert!(len < 3.5, "serpentine must not waste travel: {len}");
    }
}
