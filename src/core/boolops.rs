//! Boolean set operations on closed shapes: union, intersection, difference.
//!
//! Shapes are flattened to polygons and handed to the `geo` crate's overlay
//! engine (Martinez-style), which handles concave outlines and holes. Result
//! contours come back as closed [`Shape`]s; hole rings become their own shapes
//! so the viewport and path follower can treat them like any other contour.

use geo::{BooleanOps, Coord, LineString, MultiPolygon, Polygon};

use crate::core::geo::{Line, PathElement, Point, Shape};

/// Which set operation to apply to a selection of shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Intersect,
    Difference,
}

impl BoolOp {
    pub fn label(self) -> &'static str {
        match self {
            BoolOp::Union => "Union",
            BoolOp::Intersect => "Intersect",
            BoolOp::Difference => "Difference",
        }
    }

    pub const ALL: [BoolOp; 3] = [BoolOp::Union, BoolOp::Intersect, BoolOp::Difference];
}

/// Apply `op` to `shapes` and return the resulting contours.
///
/// At least two closed shapes are required; for [`BoolOp::Difference`] the
/// first shape is the minuend (A − B − C …). If every input shares one layer
/// the results keep it, otherwise they are unlayered.
pub fn boolean_op(shapes: &[Shape], op: BoolOp, tolerance: f64) -> Result<Vec<Shape>, String> {
    if shapes.len() < 2 {
        return Err("select at least two shapes".to_string());
    }
    let tolerance = tolerance.max(1e-4);

    let mut polys: Vec<Polygon<f64>> = Vec::with_capacity(shapes.len());
    for (i, s) in shapes.iter().enumerate() {
        if !s.closed {
            return Err(format!("shape {} is not closed", i + 1));
        }
        polys.push(to_polygon(s, tolerance)?);
    }

    let uniform_layer = match shapes.first() {
        Some(first) if shapes.iter().all(|s| s.layer == first.layer) => first.layer.clone(),
        _ => None,
    };

    let mut iter = polys.into_iter();
    let mut acc = MultiPolygon(vec![iter.next().expect("checked non-empty")]);
    for p in iter {
        let other = MultiPolygon(vec![p]);
        acc = match op {
            BoolOp::Union => acc.union(&other),
            BoolOp::Intersect => acc.intersection(&other),
            BoolOp::Difference => acc.difference(&other),
        };
    }

    Ok(from_multipolygon(&acc, uniform_layer))
}

/// Convert a shape to a geo polygon (its outline becomes the exterior ring).
fn to_polygon(shape: &Shape, tolerance: f64) -> Result<Polygon<f64>, String> {
    let ring = shape.ring(tolerance);
    if ring.len() < 3 {
        return Err("shape outline is degenerate".to_string());
    }
    let coords: Vec<Coord<f64>> = ring.iter().map(|p| Coord { x: p.x, y: p.y }).collect();
    Ok(Polygon::new(LineString::new(coords), Vec::new()))
}

/// Convert overlay results back to shapes: one closed shape per ring.
fn from_multipolygon(mp: &MultiPolygon<f64>, layer: Option<String>) -> Vec<Shape> {
    let mut out = Vec::new();
    for poly in &mp.0 {
        for ring in std::iter::once(poly.exterior()).chain(poly.interiors().iter()) {
            if let Some(s) = ring_to_shape(ring, &layer) {
                out.push(s);
            }
        }
    }
    out
}

fn ring_to_shape(ring: &LineString<f64>, layer: &Option<String>) -> Option<Shape> {
    let mut pts: Vec<Point> = ring.0.iter().map(|c| Point::new(c.x, c.y)).collect();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    if pts.len() < 3 {
        return None;
    }
    let elements = (0..pts.len())
        .map(|i| PathElement::Line(Line::new(pts[i], pts[(i + 1) % pts.len()])))
        .collect();
    Some(Shape {
        elements,
        closed: true,
        layer: layer.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axis-aligned square with lower-left corner at `(x, y)`.
    fn square(x: f64, y: f64, size: f64) -> Shape {
        let c = [
            Point::new(x, y),
            Point::new(x + size, y),
            Point::new(x + size, y + size),
            Point::new(x, y + size),
        ];
        let elements = (0..4)
            .map(|i| PathElement::Line(Line::new(c[i], c[(i + 1) % c.len()])))
            .collect();
        Shape::new(elements, true)
    }

    /// Shoelace area of a shape's outline (absolute value).
    fn area(s: &Shape) -> f64 {
        let ring = s.ring(1e-3);
        let mut a = 0.0;
        for i in 0..ring.len() {
            let p = ring[i];
            let q = ring[(i + 1) % ring.len()];
            a += p.x * q.y - q.x * p.y;
        }
        a.abs() / 2.0
    }

    fn total_area(shapes: &[Shape]) -> f64 {
        shapes.iter().map(area).sum()
    }

    #[test]
    fn union_merges_overlapping_squares() {
        let out = boolean_op(
            &[square(0.0, 0.0, 10.0), square(5.0, 0.0, 10.0)],
            BoolOp::Union,
            1e-3,
        )
        .unwrap();
        assert_eq!(out.len(), 1, "overlap must merge into one contour");
        assert!(
            (total_area(&out) - 150.0).abs() < 1e-6,
            "area {}",
            total_area(&out)
        );
        assert!(out[0].closed);
    }

    #[test]
    fn intersection_keeps_only_the_overlap() {
        let out = boolean_op(
            &[square(0.0, 0.0, 10.0), square(5.0, 0.0, 10.0)],
            BoolOp::Intersect,
            1e-3,
        )
        .unwrap();
        assert_eq!(out.len(), 1);
        assert!(
            (total_area(&out) - 50.0).abs() < 1e-6,
            "area {}",
            total_area(&out)
        );
    }

    #[test]
    fn difference_subtracts_the_second_shape() {
        let out = boolean_op(
            &[square(0.0, 0.0, 10.0), square(5.0, 0.0, 10.0)],
            BoolOp::Difference,
            1e-3,
        )
        .unwrap();
        assert!(
            (total_area(&out) - 50.0).abs() < 1e-6,
            "area {}",
            total_area(&out)
        );
    }

    #[test]
    fn union_keeps_disjoint_shapes_apart() {
        let out = boolean_op(
            &[square(0.0, 0.0, 10.0), square(20.0, 0.0, 10.0)],
            BoolOp::Union,
            1e-3,
        )
        .unwrap();
        assert_eq!(out.len(), 2, "disjoint inputs stay separate contours");
        assert!((total_area(&out) - 200.0).abs() < 1e-6);
    }

    #[test]
    fn uniform_layer_is_preserved() {
        let mut a = square(0.0, 0.0, 10.0);
        let mut b = square(5.0, 0.0, 10.0);
        a.layer = Some("top".to_string());
        b.layer = Some("top".to_string());
        let out = boolean_op(&[a.clone(), b], BoolOp::Union, 1e-3).unwrap();
        assert!(out.iter().all(|s| s.layer.as_deref() == Some("top")));

        let mut c = square(0.0, 0.0, 10.0);
        c.layer = Some("bottom".to_string());
        let mixed = boolean_op(&[a, c], BoolOp::Union, 1e-3).unwrap();
        assert!(mixed.iter().all(|s| s.layer.is_none()), "mixed layers drop");
    }

    #[test]
    fn rejects_open_or_few_shapes() {
        let open = square(0.0, 0.0, 10.0);
        let closed = square(1.0, 1.0, 5.0);
        let mut open = open;
        open.closed = false;
        assert!(boolean_op(&[open, closed.clone()], BoolOp::Union, 1e-3).is_err());
        assert!(boolean_op(&[closed], BoolOp::Union, 1e-3).is_err());
    }
}
