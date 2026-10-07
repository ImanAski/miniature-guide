//! Shape editing operators: transforms (translate / mirror / rotate) and
//! layer bookkeeping (assign / rename / delete / list).

use crate::core::geo::{Line, PathElement, Point, Shape, Transform};

// ─── Transforms ──────────────────────────────────────────────────────

/// Translate a shape by `(dx, dy)`; Z-independent, arcs keep their curvature.
pub fn translate(shape: &Shape, dx: f64, dy: f64) -> Shape {
    shape.transform(&Transform::translation(dx, dy))
}

/// Rotate a shape around `center` by `angle_rad` (counter-clockwise).
pub fn rotate(shape: &Shape, center: Point, angle_rad: f64) -> Shape {
    let t = Transform::translation(center.x, center.y)
        .compose(Transform::rotation(angle_rad))
        .compose(Transform::translation(-center.x, -center.y));
    shape.transform(&t)
}

/// Mirror across the vertical line `x = axis` (`x' = 2·axis − x`).
pub fn mirror_vertical(shape: &Shape, axis: f64) -> Shape {
    map_points(shape, |p| Point::new(2.0 * axis - p.x, p.y))
}

/// Mirror across the horizontal line `y = axis` (`y' = 2·axis − y`).
pub fn mirror_horizontal(shape: &Shape, axis: f64) -> Shape {
    map_points(shape, |p| Point::new(p.x, 2.0 * axis - p.y))
}

/// Apply an arbitrary point mapping, handling arcs exactly (a reflection
/// negates the sweep and re-derives the start angle from the mapped start).
fn map_points(shape: &Shape, f: impl Fn(Point) -> Point) -> Shape {
    let elements = shape
        .elements
        .iter()
        .map(|e| match e {
            PathElement::Line(l) => PathElement::Line(Line::new(f(l.start), f(l.end))),
            PathElement::Arc(a) => {
                let center = f(a.center);
                let start = f(a.start());
                // Reflection reverses orientation → sweep flips sign.
                PathElement::Arc(crate::core::geo::Arc {
                    center,
                    radius: a.radius,
                    start_angle: center.angle_to(start),
                    sweep: -a.sweep,
                })
            }
        })
        .collect();
    Shape {
        elements,
        closed: shape.closed,
        layer: shape.layer.clone(),
    }
}

/// Bounding-box centre of a set of shapes (origin if empty).
pub fn selection_center(shapes: &[Shape], indices: &[usize]) -> Point {
    let mut bounds: Option<crate::core::geo::Rect> = None;
    for &i in indices {
        if let Some(b) = shapes.get(i).and_then(|s| s.bounds()) {
            bounds = Some(match bounds {
                Some(cur) => cur.union(&b),
                None => b,
            });
        }
    }
    bounds.map(|b| b.center()).unwrap_or(Point::ORIGIN)
}

// ─── Layers ──────────────────────────────────────────────────────────

/// Distinct layers in first-seen order with their shape counts.
/// `None` represents unlayered shapes.
pub fn layer_counts(shapes: &[Shape]) -> Vec<(Option<String>, usize)> {
    let mut out: Vec<(Option<String>, usize)> = Vec::new();
    for s in shapes {
        match out.iter_mut().find(|(name, _)| *name == s.layer) {
            Some((_, count)) => *count += 1,
            None => out.push((s.layer.clone(), 1)),
        }
    }
    out
}

/// Indices of every shape on `layer` (`None` matches unlayered shapes).
pub fn shapes_on_layer(shapes: &[Shape], layer: &Option<String>) -> Vec<usize> {
    shapes
        .iter()
        .enumerate()
        .filter(|(_, s)| &s.layer == layer)
        .map(|(i, _)| i)
        .collect()
}

/// Assign a layer to the given shape indices.
pub fn set_layer(shapes: &mut [Shape], indices: &[usize], layer: Option<String>) {
    for &i in indices {
        if let Some(s) = shapes.get_mut(i) {
            s.layer = layer.clone();
        }
    }
}

/// Rename a layer across all shapes; returns how many shapes were updated.
pub fn rename_layer(shapes: &mut [Shape], from: &str, to: &str) -> usize {
    let to = if to.is_empty() {
        None
    } else {
        Some(to.to_string())
    };
    let mut n = 0;
    for s in shapes.iter_mut() {
        if s.layer.as_deref() == Some(from) {
            s.layer = to.clone();
            n += 1;
        }
    }
    n
}

/// Remove every shape on `layer`; returns how many were removed.
pub fn delete_layer(shapes: &mut Vec<Shape>, layer: &str) -> usize {
    let before = shapes.len();
    shapes.retain(|s| s.layer.as_deref() != Some(layer));
    before - shapes.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::Arc;

    fn line_shape(x0: f64, x1: f64, y: f64) -> Shape {
        Shape::new(
            vec![PathElement::Line(Line::new(
                Point::new(x0, y),
                Point::new(x1, y),
            ))],
            true,
        )
    }

    #[test]
    fn translate_moves_endpoints() {
        let s = translate(&line_shape(0.0, 10.0, 0.0), 5.0, -2.0);
        match &s.elements[0] {
            PathElement::Line(l) => {
                assert_eq!(l.start, Point::new(5.0, -2.0));
                assert_eq!(l.end, Point::new(15.0, -2.0));
            }
            _ => panic!("expected line"),
        }
    }

    #[test]
    fn translate_keeps_arcs_on_their_circle() {
        let s = Shape::new(
            vec![PathElement::Arc(Arc::full_circle(
                Point::new(25.0, 0.0),
                8.0,
            ))],
            true,
        );
        let moved = translate(&s, 10.0, 5.0);
        match &moved.elements[0] {
            PathElement::Arc(a) => {
                assert_eq!(a.center, Point::new(35.0, 5.0));
                assert_eq!(a.radius, 8.0);
                // The start point must still sit on the circle.
                assert!((a.start().distance(a.center) - 8.0).abs() < 1e-9);
                // Full circles stay full.
                assert!(a.is_full_circle());
            }
            _ => panic!("expected arc"),
        }
    }

    #[test]
    fn rotate_turns_a_horizontal_line_vertical() {
        use std::f64::consts::FRAC_PI_2;
        let s = rotate(&line_shape(0.0, 10.0, 0.0), Point::ORIGIN, FRAC_PI_2);
        match &s.elements[0] {
            PathElement::Line(l) => {
                assert!((l.start.x).abs() < 1e-9 && (l.start.y).abs() < 1e-9);
                assert!(l.end.x.abs() < 1e-9 && (l.end.y - 10.0).abs() < 1e-9);
            }
            _ => panic!("expected line"),
        }
    }

    #[test]
    fn mirror_vertical_flips_x_and_reverses_arc_winding() {
        let s = Shape::new(
            vec![PathElement::Arc(Arc::full_circle(
                Point::new(25.0, 0.0),
                8.0,
            ))],
            true,
        );
        let m = mirror_vertical(&s, 0.0);
        match &m.elements[0] {
            PathElement::Arc(a) => {
                assert_eq!(a.center, Point::new(-25.0, 0.0));
                assert_eq!(a.radius, 8.0);
                assert!((a.start().distance(a.center) - 8.0).abs() < 1e-9);
                assert!(a.sweep < 0.0, "reflection flips winding");
            }
            _ => panic!("expected arc"),
        }

        let l = mirror_vertical(&line_shape(0.0, 10.0, 0.0), 0.0);
        match &l.elements[0] {
            PathElement::Line(l) => {
                assert_eq!(l.start, Point::new(0.0, 0.0));
                assert_eq!(l.end, Point::new(-10.0, 0.0));
            }
            _ => panic!("expected line"),
        }
    }

    #[test]
    fn layer_counts_and_selection() {
        let mut a = line_shape(0.0, 1.0, 0.0);
        let mut b = line_shape(0.0, 1.0, 1.0);
        let c = line_shape(0.0, 1.0, 2.0);
        a.layer = Some("top".to_string());
        b.layer = Some("top".to_string());
        let mut shapes = vec![a, b, c];

        assert_eq!(
            layer_counts(&shapes),
            vec![(Some("top".to_string()), 2), (None, 1)]
        );
        assert_eq!(
            shapes_on_layer(&shapes, &Some("top".to_string())),
            vec![0, 1]
        );
        assert_eq!(shapes_on_layer(&shapes, &None), vec![2]);

        set_layer(&mut shapes, &[2], Some("bot".to_string()));
        assert_eq!(shapes[2].layer.as_deref(), Some("bot"));

        assert_eq!(rename_layer(&mut shapes, "bot", "bottom"), 1);
        assert_eq!(shapes[2].layer.as_deref(), Some("bottom"));

        assert_eq!(delete_layer(&mut shapes, "top"), 2);
        assert_eq!(shapes.len(), 1);
    }
}
