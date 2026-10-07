//! Scanline hatching for closed shapes.
//!
//! The shape is flattened to a polygon, rotated so hatch lines are horizontal,
//! intersected with evenly spaced scanlines (even-odd rule, so concave outlines
//! work), then rotated back. Cross-hatching simply runs a second pass at +90°.

use crate::core::geo::{HatchPattern, Line, PathElement, Point, Shape};

/// Minimum spacing accepted from the UI (mm).
const MIN_SPACING: f64 = 1e-4;
/// Segments shorter than this are dropped (mm).
const MIN_SEGMENT: f64 = 1e-6;

/// Hatching parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct HatchOptions {
    pub pattern: HatchPattern,
    /// Distance between scanlines (mm). Ignored for [`HatchPattern::Solid`].
    pub spacing: f64,
    /// Hatch angle in degrees.
    pub angle_deg: f64,
    /// Arc flattening tolerance (mm).
    pub tolerance: f64,
    /// Spacing used by [`HatchPattern::Solid`] (mm).
    pub solid_spacing: f64,
    /// Layer assigned to the generated shape.
    pub layer: Option<String>,
}

impl Default for HatchOptions {
    fn default() -> Self {
        HatchOptions {
            pattern: HatchPattern::Lines {
                spacing: 0.5,
                angle: 0.0,
            },
            spacing: 0.5,
            angle_deg: 0.0,
            tolerance: 0.01,
            solid_spacing: 0.02,
            layer: Some("hatch".to_string()),
        }
    }
}

/// Generate hatch line elements covering `shape`.
///
/// The shape must be closed with a non-degenerate outline; the result is a
/// single (unclosed) [`Shape`] holding every hatch segment, on `opts.layer`.
pub fn hatch(shape: &Shape, opts: &HatchOptions) -> Result<Shape, String> {
    if !shape.closed {
        return Err("shape is not closed".to_string());
    }
    let poly = flatten(shape, opts.tolerance);
    if poly.len() < 3 {
        return Err("shape has no usable outline".to_string());
    }

    let spacing = match opts.pattern {
        HatchPattern::Solid => opts.solid_spacing,
        HatchPattern::Lines { .. } | HatchPattern::CrossHatch { .. } => opts.spacing,
    }
    .max(MIN_SPACING);

    let angles = match opts.pattern {
        HatchPattern::Lines { .. } | HatchPattern::Solid => vec![opts.angle_deg],
        HatchPattern::CrossHatch { .. } => vec![opts.angle_deg, opts.angle_deg + 90.0],
    };

    let mut elements = Vec::new();
    for angle_deg in angles {
        elements.extend(scanlines(&poly, spacing, angle_deg.to_radians()));
    }
    if elements.is_empty() {
        return Err("no hatch lines generated (spacing larger than shape?)".to_string());
    }

    Ok(Shape {
        elements,
        closed: false,
        layer: opts.layer.clone(),
    })
}

/// Flatten a shape into a closed point ring (first point not repeated at the
/// end), dropping zero-length steps.
fn flatten(shape: &Shape, tolerance: f64) -> Vec<Point> {
    shape.ring(tolerance)
}

/// Intersect `poly` with horizontal scanlines spaced `spacing` apart at
/// `angle`, returning the segments rotated back to the original frame.
fn scanlines(poly: &[Point], spacing: f64, angle: f64) -> Vec<PathElement> {
    let mut min = poly[0];
    let mut max = poly[0];
    for p in poly {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
    }
    let center = (min + max) * 0.5;
    if (max.x - min.x) < MIN_SEGMENT || (max.y - min.y) < MIN_SEGMENT {
        return Vec::new();
    }

    // Rotate the outline so the hatch direction becomes horizontal.
    let rot: Vec<Point> = poly
        .iter()
        .map(|p| p.rotate_around(center, -angle))
        .collect();

    let mut rmin = rot[0];
    let mut rmax = rot[0];
    for p in &rot {
        rmin.y = rmin.y.min(p.y);
        rmax.y = rmax.y.max(p.y);
        rmin.x = rmin.x.min(p.x);
        rmax.x = rmax.x.max(p.x);
    }

    let mut out: Vec<PathElement> = Vec::new();
    // Scanlines strictly inside the outline: nudging the start by an epsilon
    // keeps rows exactly on the bbox edges (where the even-odd rule is
    // asymmetric) out of the result on both sides.
    let eps = spacing * 1e-9;
    let mut y = ((rmin.y + eps) / spacing).ceil() * spacing;
    let mut row = 0usize;
    while y < rmax.y - eps {
        let mut xs = crossings(&rot, y);
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // Even-odd rule: pair up crossings into covered spans.
        for pair in xs.chunks_exact(2) {
            let (x0, x1) = (pair[0], pair[1]);
            if x1 - x0 <= MIN_SEGMENT {
                continue;
            }
            // Boustrophedon: alternate rows run opposite ways so travel
            // between rows stays short.
            let (a, b) = if row.is_multiple_of(2) {
                (x0, x1)
            } else {
                (x1, x0)
            };
            let start = Point::new(a, y).rotate_around(center, angle);
            let end = Point::new(b, y).rotate_around(center, angle);
            out.push(PathElement::Line(Line::new(start, end)));
        }
        y += spacing;
        row += 1;
    }
    out
}

/// x-coordinates where the horizontal line `y` crosses the polygon edges.
///
/// Uses the strict half-plane rule (`p.y > y`) so a vertex shared by two edges
/// is counted exactly once.
fn crossings(poly: &[Point], y: f64) -> Vec<f64> {
    let mut xs = Vec::new();
    let n = poly.len();
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        if (p1.y > y) == (p2.y > y) {
            continue;
        }
        let t = (y - p1.y) / (p2.y - p1.y);
        xs.push(p1.x + t * (p2.x - p1.x));
    }
    xs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::{Arc, PathElement};

    /// Axis-aligned square centred on the origin.
    fn square(size: f64) -> Shape {
        let h = size / 2.0;
        let c = [
            Point::new(-h, -h),
            Point::new(h, -h),
            Point::new(h, h),
            Point::new(-h, h),
        ];
        let elements = (0..4)
            .map(|i| PathElement::Line(Line::new(c[i], c[(i + 1) % c.len()])))
            .collect();
        Shape::new(elements, true)
    }

    fn circle(center: Point, radius: f64) -> Shape {
        Shape::new(
            vec![PathElement::Arc(Arc::full_circle(center, radius))],
            true,
        )
    }

    fn lines(s: &Shape) -> Vec<&Line> {
        s.elements
            .iter()
            .filter_map(|e| match e {
                PathElement::Line(l) => Some(l),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn horizontal_lines_cover_a_square() {
        let opts = HatchOptions {
            pattern: HatchPattern::Lines {
                spacing: 1.0,
                angle: 0.0,
            },
            spacing: 1.0,
            ..Default::default()
        };
        let h = hatch(&square(10.0), &opts).unwrap();
        // Rows strictly inside y ∈ (-5, 5): -4..4 → 9 rows of 10 mm each.
        assert_eq!(h.elements.len(), 9);
        let total: f64 = lines(&h).iter().map(|l| l.length()).sum();
        assert!((total - 90.0).abs() < 1e-6, "total {total}");
        for l in lines(&h) {
            assert!((l.start.y - l.end.y).abs() < 1e-9, "rows must be flat");
            assert!(l.start.x.abs() <= 5.0 + 1e-9 && l.end.x.abs() <= 5.0 + 1e-9);
        }
    }

    #[test]
    fn vertical_hatch_at_90_degrees() {
        let opts = HatchOptions {
            pattern: HatchPattern::Lines {
                spacing: 1.0,
                angle: 90.0,
            },
            spacing: 1.0,
            angle_deg: 90.0,
            ..Default::default()
        };
        let h = hatch(&square(10.0), &opts).unwrap();
        assert_eq!(h.elements.len(), 9);
        for l in lines(&h) {
            assert!(
                (l.start.x - l.end.x).abs() < 1e-9,
                "columns must be vertical"
            );
        }
    }

    #[test]
    fn cross_hatch_runs_two_passes() {
        let opts = HatchOptions {
            pattern: HatchPattern::CrossHatch {
                spacing: 2.0,
                angle: 0.0,
            },
            spacing: 2.0,
            ..Default::default()
        };
        let h = hatch(&square(10.0), &opts).unwrap();
        // spacing 2 → rows at -4..4 per pass (±5 boundary also hit: -4,-2,0,2,4 →
        // ceil(-5/2)*2 = -4 … 4 → 5 rows per pass, 10 total).
        assert_eq!(h.elements.len(), 10);
    }

    #[test]
    fn solid_uses_the_dense_spacing() {
        let opts = HatchOptions {
            pattern: HatchPattern::Solid,
            solid_spacing: 2.5,
            spacing: 999.0, // must be ignored
            ..Default::default()
        };
        let h = hatch(&square(10.0), &opts).unwrap();
        // Rows strictly inside y ∈ (-5, 5) at 2.5 mm: -2.5, 0, 2.5 → 3.
        assert_eq!(h.elements.len(), 3);
    }

    #[test]
    fn circle_hatch_stays_inside_the_radius() {
        let opts = HatchOptions {
            pattern: HatchPattern::Lines {
                spacing: 1.0,
                angle: 0.0,
            },
            spacing: 1.0,
            tolerance: 0.001,
            ..Default::default()
        };
        let h = hatch(&circle(Point::ORIGIN, 5.0), &opts).unwrap();
        assert!(!h.elements.is_empty());
        for l in lines(&h) {
            for p in [l.start, l.end] {
                assert!(p.length() <= 5.0 + 0.01, "hatch point {} outside radius", p);
            }
        }
    }

    #[test]
    fn open_shapes_are_rejected() {
        let open = Shape::new(
            vec![PathElement::Line(Line::new(
                Point::ORIGIN,
                Point::new(5.0, 0.0),
            ))],
            false,
        );
        let err = hatch(&open, &HatchOptions::default()).unwrap_err();
        assert!(err.contains("not closed"), "{err}");
    }

    #[test]
    fn layer_is_carried_over() {
        let opts = HatchOptions {
            layer: Some("hatch-top".to_string()),
            ..Default::default()
        };
        let h = hatch(&square(4.0), &opts).unwrap();
        assert_eq!(h.layer.as_deref(), Some("hatch-top"));
    }
}
