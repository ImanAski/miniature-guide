//! DXF front end — flattens a DXF drawing into [`Shape`] contours.
//!
//! Geometry is projected onto the XY plane (Z is dropped), block references
//! (`INSERT`) are expanded in place with their scale, rotation and array offsets
//! applied, and curves with no exact [`PathElement`] counterpart (`ELLIPSE`,
//! `SPLINE`, arcs distorted by a non-uniform scale) are flattened into line
//! segments.

use std::collections::BTreeSet;
use std::f64::consts::TAU;
use std::path::Path;

use acadrust::entities::{Ellipse, Insert, Polyline2D, PolylineFlags, Spline, VertexFlags};
use acadrust::{CadDocument, DxfError, DxfReader, EntityType, Handle, LwPolyline, Vector3};

use crate::core::geo::{Arc, Line, PathElement, Point, Shape};

use super::affine::Affine;

/// Absolute chord tolerance (mm) used when a curve has to be flattened.
const FLATTEN_TOLERANCE: f64 = 0.01;
/// Chord tolerance relative to the curve radius, so huge curves stay bounded.
const FLATTEN_REL_TOLERANCE: f64 = 1e-5;
/// Upper bound on the segments produced by flattening a single curve.
const MAX_FLATTEN_SEGMENTS: usize = 1024;
/// Chords sampled per knot span when flattening a spline.
const SPLINE_SAMPLES_PER_SPAN: usize = 8;
/// How deep `INSERT` and `EXPLODE` nesting is followed.
const MAX_NESTING: usize = 8;
/// Upper bound on the instances produced by a single MINSERT array.
const MAX_INSERT_INSTANCES: usize = 4096;
/// Points closer than this are treated as coincident.
const WELD_EPSILON: f64 = 1e-9;
/// Bulges below this are treated as straight segments.
const BULGE_EPSILON: f64 = 1e-12;

/// Read a DXF file and return its drawable contours.
pub fn parse(path: &Path) -> Result<Vec<Shape>, DxfError> {
    let doc = DxfReader::from_file(path)?.read()?;
    Ok(collect(&doc))
}

/// Convert every model-space entity of `doc` into shapes.
pub fn collect(doc: &CadDocument) -> Vec<Shape> {
    let roots: Vec<&EntityType> = if doc.model_space_entities().next().is_some() {
        doc.model_space_entities().collect()
    } else {
        // Some writers never attach the ENTITIES section to a `*Model_Space`
        // block record. Fall back to the raw entity list, minus everything that
        // belongs to a block definition — that arrives through its `INSERT`.
        let definitions: BTreeSet<Handle> = doc
            .block_records
            .iter()
            .filter(|record| !record.is_model_space())
            .flat_map(|record| record.entity_handles.iter().copied())
            .collect();
        doc.entities()
            .filter(|entity| !definitions.contains(&entity.common().handle))
            .collect()
    };

    let mut walker = Walker {
        doc,
        shapes: Vec::new(),
    };
    for entity in roots {
        walker.entity(entity, &Place::root());
    }
    walker.shapes
}

// ─── Document walk ─────────────────────────────────────────────────────────

struct Walker<'a> {
    doc: &'a CadDocument,
    shapes: Vec<Shape>,
}

/// Placement state threaded through the recursion.
#[derive(Clone)]
struct Place {
    xf: Affine,
    depth: usize,
    /// Layer inherited from the enclosing `INSERT`. Block members left on layer
    /// `"0"` inherit it, the same way CAD does.
    layer: Option<String>,
}

impl Place {
    fn root() -> Self {
        Place {
            xf: Affine::IDENTITY,
            depth: 0,
            layer: None,
        }
    }
}

impl<'a> Walker<'a> {
    fn entity(&mut self, entity: &'a EntityType, place: &Place) {
        let common = entity.as_entity();
        if common.is_invisible() {
            return;
        }

        let own = common.layer();
        let layer = match place.layer.as_deref() {
            Some(_) if own.is_empty() || own == "0" => place.layer.clone(),
            _ => layer_name(own),
        };

        match entity {
            EntityType::Insert(insert) => self.insert(insert, place, &layer),
            other => self
                .shapes
                .extend(convert(other, place.xf, &layer, place.depth)),
        }
    }

    /// Expand a block reference into its definition, once per array instance.
    fn insert(&mut self, insert: &'a Insert, place: &Place, layer: &Option<String>) {
        if place.depth >= MAX_NESTING {
            return;
        }
        let columns = usize::from(insert.column_count.max(1));
        let rows = usize::from(insert.row_count.max(1));
        if columns.saturating_mul(rows) > MAX_INSERT_INSTANCES {
            return;
        }

        // Copy the document reference out of `self` so the block members can be
        // walked while `self` is borrowed mutably.
        let doc = self.doc;
        let nested = Place {
            depth: place.depth + 1,
            layer: layer.clone(),
            ..place.clone()
        };

        for column in 0..columns {
            for row in 0..rows {
                let inner = Place {
                    xf: place.xf.then(placement(insert, column as f64, row as f64)),
                    ..nested.clone()
                };
                for member in doc.entities_in_block(&insert.block_name) {
                    self.entity(member, &inner);
                }
            }
        }
    }
}

/// Where one MINSERT instance of `insert` lands.
///
/// DXF order: array offset and block scale act first, then rotation, then the
/// move to the insert point. Spacing is given in block coordinates.
fn placement(insert: &Insert, column: f64, row: f64) -> Affine {
    Affine::translation(insert.insert_point.x, insert.insert_point.y)
        .then(Affine::rotation(insert.rotation))
        .then(Affine::scale(insert.x_scale(), insert.y_scale()))
        .then(Affine::translation(
            column * insert.column_spacing,
            row * insert.row_spacing,
        ))
}

fn layer_name(name: &str) -> Option<String> {
    if name.is_empty() {
        None
    } else {
        Some(name.to_string())
    }
}

// ─── Entity conversion ──────────────────────────────────────────────────────

/// Convert one entity into shapes. `INSERT` is resolved by [`Walker`] instead.
fn convert(entity: &EntityType, xf: Affine, layer: &Option<String>, depth: usize) -> Vec<Shape> {
    if depth >= MAX_NESTING {
        return Vec::new();
    }

    match geometry(entity, xf, depth) {
        Some((elements, closed)) => {
            if elements.is_empty() {
                return Vec::new();
            }
            let mut shape = Shape::new(elements, closed);
            shape.layer = layer.clone();
            vec![shape]
        }
        // Not directly drawable — let acadrust break it into primitives and
        // regroup whatever comes out into contours.
        None => {
            let parts = entity.explode();
            if parts.is_empty() {
                return Vec::new();
            }
            let elements: Vec<PathElement> = parts
                .iter()
                .filter_map(|part| geometry(part, xf, depth + 1))
                .flat_map(|(elements, _)| elements)
                .collect();
            chain(elements, layer)
        }
    }
}

/// Elements for a single entity. `None` means "explode me first".
fn geometry(entity: &EntityType, xf: Affine, depth: usize) -> Option<(Vec<PathElement>, bool)> {
    if depth >= MAX_NESTING {
        return None;
    }

    Some(match entity {
        EntityType::Line(l) => (
            vec![PathElement::Line(Line::new(
                xf.point(l.start),
                xf.point(l.end),
            ))],
            false,
        ),

        EntityType::Circle(c) => (RawArc::full_circle(c.center, c.radius).elements(xf), true),

        EntityType::Arc(a) => (
            RawArc::new(a.center, a.radius, a.start_angle, a.sweep_angle()).elements(xf),
            false,
        ),

        EntityType::Ellipse(e) => (ellipse_elements(e, xf), e.is_full()),

        EntityType::Polyline2D(p) => polyline2d_elements(p, xf),
        EntityType::LwPolyline(p) => (lwpolyline_elements(p, xf), p.is_closed),
        EntityType::Polyline(p) => {
            let vertices: Vec<Vector3> = p.vertices.iter().map(|v| v.location).collect();
            (
                vertex_lines(&vertices, p.flags.is_closed(), xf),
                p.flags.is_closed(),
            )
        }
        EntityType::Polyline3D(p) => {
            let vertices: Vec<Vector3> = p.vertices.iter().map(|v| v.position).collect();
            (vertex_lines(&vertices, p.flags.closed, xf), p.flags.closed)
        }

        EntityType::Solid(s) => (polygon(s.boundary_corners(), xf), true),
        EntityType::Face3D(f) => (polygon(f.corners(), xf), true),

        EntityType::Spline(s) => (spline_elements(s, xf), s.flags.closed),
        EntityType::Helix(h) => (spline_elements(&h.spline, xf), h.spline.flags.closed),

        EntityType::Leader(l) => (vertex_lines(&l.vertices, false, xf), false),
        EntityType::MLine(m) => {
            let vertices: Vec<Vector3> = m.vertices.iter().map(|v| v.position).collect();
            let closed = m.is_closed();
            (vertex_lines(&vertices, closed, xf), closed)
        }

        // Text, points, rays, block markers and anything else acadrust can
        // decompose is handled by the explode path.
        _ => return None,
    })
}

// ─── Curve primitives ───────────────────────────────────────────────────────

/// A circular arc expressed in the coordinate system of its DXF entity.
struct RawArc {
    center: Vector3,
    radius: f64,
    start_angle: f64,
    /// Signed sweep in radians; positive is counter-clockwise.
    sweep: f64,
}

impl RawArc {
    fn new(center: Vector3, radius: f64, start_angle: f64, sweep: f64) -> Self {
        RawArc {
            center,
            radius,
            start_angle,
            sweep,
        }
    }

    fn full_circle(center: Vector3, radius: f64) -> Self {
        RawArc::new(center, radius, 0.0, TAU)
    }

    /// Arc joining two polyline vertices that carry a DXF bulge value.
    ///
    /// A bulge is `tan(θ / 4)` for the included angle `θ`, so a positive bulge
    /// bulges counter-clockwise and a negative one clockwise.
    fn from_bulge(start: Vector3, end: Vector3, bulge: f64) -> Option<Self> {
        if bulge.abs() <= BULGE_EPSILON {
            return None;
        }
        let (dx, dy) = (end.x - start.x, end.y - start.y);
        let chord = dx.hypot(dy);
        if chord <= WELD_EPSILON {
            return None;
        }

        let squared = bulge * bulge;
        let radius = chord * (1.0 + squared) / (4.0 * bulge.abs());
        // Centre offset from the chord midpoint, along its left normal.
        let offset = chord * (1.0 - squared) / (4.0 * bulge);
        let center = Vector3::new(
            0.5 * (start.x + end.x) - dy / chord * offset,
            0.5 * (start.y + end.y) + dx / chord * offset,
            start.z,
        );
        let start_angle = (start.y - center.y).atan2(start.x - center.x);

        Some(RawArc::new(center, radius, start_angle, 4.0 * bulge.atan()))
    }

    /// Map into world space: an exact arc while `xf` keeps circles circular,
    /// flattened segments otherwise.
    fn elements(self, xf: Affine) -> Vec<PathElement> {
        match xf.similarity() {
            Some((scale, true)) => vec![PathElement::Arc(Arc {
                center: xf.point(self.center),
                radius: self.radius * scale,
                start_angle: xf.angle() + self.start_angle,
                sweep: self.sweep,
            })],
            // A reflection swaps the winding direction of the arc.
            Some((scale, false)) => vec![PathElement::Arc(Arc {
                center: xf.point(self.center),
                radius: self.radius * scale,
                start_angle: xf.angle() - self.start_angle,
                sweep: -self.sweep,
            })],
            None => self.flatten(xf),
        }
    }

    /// Sample the arc into chords.
    fn flatten(self, xf: Affine) -> Vec<PathElement> {
        let segments = arc_segments(self.sweep.abs(), self.radius * xf.stretch_bound());
        let step = self.sweep / segments as f64;
        (0..segments)
            .map(|i| {
                let from = self.start_angle + step * i as f64;
                let to = from + step;
                PathElement::Line(Line::new(
                    xf.point(self.point_at(from)),
                    xf.point(self.point_at(to)),
                ))
            })
            .collect()
    }

    fn point_at(&self, angle: f64) -> Vector3 {
        Vector3::new(
            self.center.x + self.radius * angle.cos(),
            self.center.y + self.radius * angle.sin(),
            self.center.z,
        )
    }
}

fn ellipse_elements(e: &Ellipse, xf: Affine) -> Vec<PathElement> {
    let major = e.major_axis_length();
    if major <= WELD_EPSILON || e.minor_axis_ratio <= 0.0 {
        return Vec::new();
    }
    let span = e.end_parameter - e.start_parameter;
    if span.abs() <= WELD_EPSILON {
        return Vec::new();
    }

    // An ellipse with equal axes is a circle — keep it exact when possible.
    if (e.minor_axis_ratio - 1.0).abs() <= WELD_EPSILON
        && let Some((scale, keeps_circle)) = xf.similarity()
    {
        let sweep = if keeps_circle { span } else { -span };
        let start = if keeps_circle {
            xf.angle() + e.start_parameter
        } else {
            xf.angle() - e.start_parameter
        };
        return vec![PathElement::Arc(Arc {
            center: xf.point(e.center),
            radius: major * scale,
            start_angle: start,
            sweep,
        })];
    }

    let major_dir = e.major_axis.normalize();
    let minor_dir = e.normal.cross(&major_dir).normalize();
    let minor = e.minor_axis_length();
    let segments = arc_segments(span.abs(), major.max(minor) * xf.stretch_bound());
    let step = span / segments as f64;

    (0..segments)
        .map(|i| {
            let from = e.start_parameter + step * i as f64;
            let to = from + step;
            PathElement::Line(Line::new(
                xf.point(ellipse_point(e, &major_dir, &minor_dir, major, minor, from)),
                xf.point(ellipse_point(e, &major_dir, &minor_dir, major, minor, to)),
            ))
        })
        .collect()
}

fn ellipse_point(
    e: &Ellipse,
    major_dir: &Vector3,
    minor_dir: &Vector3,
    major: f64,
    minor: f64,
    t: f64,
) -> Vector3 {
    e.center + *major_dir * (major * t.cos()) + *minor_dir * (minor * t.sin())
}

fn spline_elements(s: &Spline, xf: Affine) -> Vec<PathElement> {
    vertex_lines(&sample_spline(s), s.flags.closed, xf)
}

/// Evaluate a spline into a dense point chain. Rational splines are evaluated
/// through de Boor's algorithm on homogeneous control points.
fn sample_spline(s: &Spline) -> Vec<Vector3> {
    let controls = &s.control_points;
    let degree = s.degree.max(1) as usize;

    if controls.len() >= 2 && degree < controls.len() {
        let knots = if s.knots.len() == controls.len() + degree + 1 {
            s.knots.clone()
        } else {
            Spline::generate_clamped_knots(degree, controls.len())
        };
        let weights = (s.weights.len() == controls.len()).then_some(s.weights.as_slice());
        let spans = knots.len().saturating_sub(degree + 1);
        let segments = spans
            .saturating_mul(SPLINE_SAMPLES_PER_SPAN)
            .clamp(2, MAX_FLATTEN_SEGMENTS);
        let (from, to) = (knots[degree], knots[controls.len()]);
        return (0..=segments)
            .map(|i| {
                let u = from + (to - from) * (i as f64 / segments as f64);
                de_boor(&knots, degree, controls, weights, u)
            })
            .collect();
    }

    s.fit_points.clone()
}

/// de Boor's algorithm for one parameter value.
fn de_boor(
    knots: &[f64],
    degree: usize,
    controls: &[Vector3],
    weights: Option<&[f64]>,
    u: f64,
) -> Vector3 {
    let count = controls.len();
    let u = u.clamp(knots[degree], knots[count]);

    // Knot span `degree..=count - 1`; `count` would index past the last point.
    let mut span = degree;
    while span + 1 < count && u >= knots[span + 1] {
        span += 1;
    }

    // Homogeneous control points (x·w, y·w, w); w = 1 for non-rational curves.
    let mut ring = vec![Vector3::ZERO; degree + 1];
    for (offset, slot) in ring.iter_mut().enumerate() {
        let index = span - degree + offset;
        let weight = weights.map_or(1.0, |w| w[index]);
        *slot = controls[index] * weight;
    }

    for level in 1..=degree {
        for offset in (level..=degree).rev() {
            let index = span - degree + offset;
            let low = knots[index];
            let high = knots[index + degree - level + 1];
            let alpha = if high - low > WELD_EPSILON {
                (u - low) / (high - low)
            } else {
                0.0
            };
            ring[offset] = ring[offset - 1] + (ring[offset] - ring[offset - 1]) * alpha;
        }
    }

    let weight = ring[degree].z;
    if weight.abs() > WELD_EPSILON {
        ring[degree] / weight
    } else {
        Vector3::new(ring[degree].x, ring[degree].y, 0.0)
    }
}

// ─── Chains ─────────────────────────────────────────────────────────────────

/// Elements for `vertices` joined by straight segments.
fn vertex_lines(vertices: &[Vector3], closed: bool, xf: Affine) -> Vec<PathElement> {
    let count = vertices.len();
    let segments = if closed {
        count
    } else {
        count.saturating_sub(1)
    };
    (0..segments)
        .map(|i| {
            PathElement::Line(Line::new(
                xf.point(vertices[i]),
                xf.point(vertices[(i + 1) % count]),
            ))
        })
        .collect()
}

/// Elements for a closed or open outline of `corners`.
fn polygon(corners: Vec<Vector3>, xf: Affine) -> Vec<PathElement> {
    let mut corners = corners;
    corners.dedup_by(|a, b| coincident(a.x, a.y, b.x, b.y));
    vertex_lines(&corners, true, xf)
}

/// Elements for `vertices[i].1` shaping the segment that starts at vertex `i`.
fn vertex_chain(vertices: &[(Vector3, f64)], closed: bool, xf: Affine) -> Vec<PathElement> {
    let count = vertices.len();
    let segments = if closed {
        count
    } else {
        count.saturating_sub(1)
    };
    let mut elements = Vec::with_capacity(segments);
    for i in 0..segments {
        let (start, bulge) = vertices[i];
        let end = vertices[(i + 1) % count].0;
        match RawArc::from_bulge(start, end, bulge) {
            Some(arc) => elements.extend(arc.elements(xf)),
            None => elements.push(PathElement::Line(Line::new(xf.point(start), xf.point(end)))),
        }
    }
    elements
}

fn polyline2d_elements(p: &Polyline2D, xf: Affine) -> (Vec<PathElement>, bool) {
    let closed = p.flags.is_closed();
    if is_curve_fit(&p.flags) && p.vertices.len() > 2 {
        // Curve-fit polylines carry generated vertices; the real control points
        // are the ones without the "extra vertex" flag.
        let fit: Vec<Vector3> = p
            .vertices
            .iter()
            .filter(|v| v.flags.bits() & VertexFlags::EXTRA_VERTEX.bits() == 0)
            .map(|v| v.location)
            .collect();
        if fit.len() > 2 {
            let spline = Spline::from_fit_points(fit);
            return (spline_elements(&spline, xf), closed);
        }
    }

    let vertices: Vec<(Vector3, f64)> = p.vertices.iter().map(|v| (v.location, v.bulge)).collect();
    (vertex_chain(&vertices, closed, xf), closed)
}

fn lwpolyline_elements(p: &LwPolyline, xf: Affine) -> Vec<PathElement> {
    let vertices: Vec<(Vector3, f64)> = p
        .vertices
        .iter()
        .map(|v| {
            (
                Vector3::new(v.location.x, v.location.y, p.elevation),
                v.bulge,
            )
        })
        .collect();
    vertex_chain(&vertices, p.is_closed, xf)
}

/// Group loose elements into shapes, joining segments that share an endpoint.
fn chain(elements: Vec<PathElement>, layer: &Option<String>) -> Vec<Shape> {
    let mut shapes = Vec::new();
    let mut run: Vec<PathElement> = Vec::new();

    for element in elements {
        match run.last() {
            Some(previous) if coincident_points(end_of(previous), start_of(&element)) => {
                run.push(element);
            }
            _ => {
                shapes.push(finish(std::mem::take(&mut run), layer));
                run.push(element);
            }
        }
    }
    shapes.push(finish(run, layer));
    shapes.retain(|shape| !shape.elements.is_empty());
    shapes
}

fn finish(elements: Vec<PathElement>, layer: &Option<String>) -> Shape {
    let closed = elements.len() >= 3
        && elements.iter().all(|e| matches!(e, PathElement::Line(_)))
        && coincident_points(start_of(&elements[0]), end_of(elements.last().unwrap()));
    let mut shape = Shape::new(elements, closed);
    shape.layer = layer.clone();
    shape
}

fn start_of(element: &PathElement) -> Point {
    match element {
        PathElement::Line(l) => l.start,
        PathElement::Arc(a) => a.start(),
    }
}

fn end_of(element: &PathElement) -> Point {
    match element {
        PathElement::Line(l) => l.end,
        PathElement::Arc(a) => a.end(),
    }
}

fn coincident_points(a: Point, b: Point) -> bool {
    coincident(a.x, a.y, b.x, b.y)
}

fn coincident(ax: f64, ay: f64, bx: f64, by: f64) -> bool {
    (ax - bx).abs() <= WELD_EPSILON && (ay - by).abs() <= WELD_EPSILON
}

fn is_curve_fit(flags: &PolylineFlags) -> bool {
    let curve_fit = PolylineFlags::CURVE_FIT.bits() | PolylineFlags::SPLINE_FIT.bits();
    flags.bits() & curve_fit != 0
}

// ─── Flattening helpers ─────────────────────────────────────────────────────

/// Chord tolerance for a curve of `radius` — absolute near the origin, relative
/// once the curve grows large.
fn tolerance(radius: f64) -> f64 {
    FLATTEN_TOLERANCE.max(radius.abs() * FLATTEN_REL_TOLERANCE)
}

/// Largest angle a single chord may subtend before it exceeds the tolerance.
fn chord_angle(radius: f64) -> f64 {
    let radius = radius.abs().max(WELD_EPSILON);
    let ratio = (tolerance(radius) / radius).clamp(WELD_EPSILON, 1.0 - WELD_EPSILON);
    2.0 * (1.0 - ratio).acos()
}

/// Number of chords needed to approximate an arc covering `sweep` radians.
fn arc_segments(sweep: f64, radius: f64) -> usize {
    let step = chord_angle(radius);
    if step <= WELD_EPSILON || !step.is_finite() {
        return MAX_FLATTEN_SEGMENTS;
    }
    (sweep.abs() / step)
        .ceil()
        .clamp(1.0, MAX_FLATTEN_SEGMENTS as f64) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EXAMPLE: &str = "tests/data/example.dxf";

    /// `tests/` is not tracked in git, so a fresh checkout (CI) may lack the
    /// fixture. `None` means "skip"; local runs still execute the test.
    fn example() -> Option<&'static Path> {
        let p = Path::new(EXAMPLE);
        if p.exists() {
            Some(p)
        } else {
            eprintln!("skipping {EXAMPLE}: fixture not present in this checkout");
            None
        }
    }

    #[test]
    fn parse_dxf_file() {
        let Some(path) = example() else { return };
        let shapes = parse(path).expect("example.dxf should parse");

        assert!(!shapes.is_empty(), "expected drawable geometry");
        assert!(
            shapes
                .iter()
                .any(|s| s.elements.iter().any(|e| matches!(e, PathElement::Arc(_)))),
            "example.dxf contains circles, expected at least one arc"
        );
        assert!(
            shapes.iter().all(|s| !s.elements.is_empty()),
            "shapes never carry empty element lists"
        );
        assert!(
            shapes.iter().all(|s| s
                .bounds()
                .is_some_and(|b| b.min.x.is_finite() && b.max.x.is_finite())),
            "every shape should report finite bounds"
        );
    }

    #[test]
    fn blocks_are_expanded_in_place() {
        let Some(path) = example() else { return };
        let doc = DxfReader::from_file(path)
            .expect("open")
            .read()
            .expect("read");
        let raw = doc.entity_count();
        let shapes = collect(&doc);

        let inserts = doc.model_space_entities().count();
        assert!(inserts > 0, "example.dxf references blocks");
        assert!(
            shapes.len() > raw / 2,
            "block geometry should add shapes beyond the {} raw entities",
            raw
        );
    }

    #[test]
    fn layers_are_carried_over() {
        let Some(path) = example() else { return };
        let shapes = parse(path).expect("parse");
        assert!(
            shapes.iter().any(|s| s.layer.as_deref() == Some("Layer1")),
            "expected shapes tagged with the Layer1 layer"
        );
    }

    #[test]
    fn circle_stays_an_arc_without_distortion() {
        let elements =
            RawArc::full_circle(Vector3::new(3.0, 4.0, 0.0), 2.5).elements(Affine::IDENTITY);

        assert_eq!(elements.len(), 1);
        let PathElement::Arc(arc) = &elements[0] else {
            panic!("expected an arc");
        };
        assert_eq!(arc.center, Point::new(3.0, 4.0));
        assert!((arc.radius - 2.5).abs() < 1e-12);
        assert!(arc.is_full_circle());
    }

    #[test]
    fn mirrored_circle_keeps_its_radius_but_flips_winding() {
        let mirror = Affine::scale(1.0, -1.0);
        let elements = RawArc::full_circle(Vector3::ZERO, 2.0).elements(mirror);

        assert_eq!(elements.len(), 1);
        let PathElement::Arc(arc) = &elements[0] else {
            panic!("expected an arc");
        };
        assert!((arc.radius - 2.0).abs() < 1e-12);
        assert!((arc.sweep + TAU).abs() < 1e-12, "winding must reverse");
    }

    #[test]
    fn non_uniform_scale_flattens_arcs() {
        let squash = Affine::scale(2.0, 1.0);
        let elements = RawArc::full_circle(Vector3::ZERO, 1.0).elements(squash);

        assert!(
            elements.len() > 1,
            "arcs cannot survive a non-uniform scale"
        );
        assert!(elements.iter().all(|e| matches!(e, PathElement::Line(_))));
    }

    #[test]
    fn bulge_becomes_an_arc_between_its_vertices() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let end = Vector3::new(10.0, 0.0, 0.0);
        let arc = RawArc::from_bulge(start, end, 1.0).expect("bulge 1.0 is a half circle");

        let elements = arc.elements(Affine::IDENTITY);
        let PathElement::Arc(mapped) = &elements[0] else {
            panic!("expected an arc");
        };
        assert!((mapped.radius - 5.0).abs() < 1e-12);
        assert!((mapped.sweep - PI).abs() < 1e-12);
        assert!(coincident_points(mapped.start(), xy(start)));
        assert!(coincident_points(mapped.end(), xy(end)));

        assert!(
            RawArc::from_bulge(start, end, 0.0).is_none(),
            "zero bulge is a line"
        );
        assert!(
            RawArc::from_bulge(start, start, 1.0).is_none(),
            "coincident vertices have no arc"
        );
    }

    #[test]
    fn negative_bulge_bulges_clockwise() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let end = Vector3::new(10.0, 0.0, 0.0);
        let arc = RawArc::from_bulge(start, end, -1.0).expect("negative bulge is an arc");

        assert!((arc.sweep + PI).abs() < 1e-12);
        assert!(
            (arc.start_angle - PI).abs() < 1e-12,
            "the arc starts at (0, 0), which sits at π rad from its centre"
        );
    }

    #[test]
    fn bulged_chain_reproduces_its_endpoints() {
        let vertices = vec![
            (Vector3::new(0.0, 0.0, 0.0), 0.0),
            (Vector3::new(10.0, 0.0, 0.0), 0.5),
            (Vector3::new(10.0, 10.0, 0.0), 0.0),
        ];
        let elements = vertex_chain(&vertices, false, Affine::IDENTITY);

        assert_eq!(elements.len(), 2);
        assert!(matches!(elements[0], PathElement::Line(_)));
        assert!(matches!(elements[1], PathElement::Arc(_)));
        assert!(coincident_points(start_of(&elements[0]), xy(vertices[0].0)));
        assert!(coincident_points(end_of(&elements[1]), xy(vertices[2].0)));
    }

    #[test]
    fn spline_through_control_points_hits_its_ends() {
        let controls = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(5.0, 10.0, 0.0),
            Vector3::new(15.0, 10.0, 0.0),
            Vector3::new(20.0, 0.0, 0.0),
        ];
        let spline = Spline::from_control_points(3, controls.clone());
        let sampled = sample_spline(&spline);

        assert!(sampled.len() > 4, "spline should be sampled finely");
        assert!(coincident_points(xy(sampled[0]), xy(controls[0])));
        assert!(coincident_points(
            xy(*sampled.last().unwrap()),
            xy(*controls.last().unwrap())
        ));
    }

    #[test]
    fn quadratic_spline_follows_the_bezier_midpoint() {
        // A clamped quadratic B-spline is a Bézier, so the midpoint of the
        // parameter range is fixed by the Bernstein weights.
        let controls = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 20.0, 0.0),
            Vector3::new(20.0, 0.0, 0.0),
        ];
        let spline = Spline::from_control_points(2, controls.clone());

        let middle = de_boor(&spline.knots, 2, &spline.control_points, None, 0.5);

        assert!(
            coincident(middle.x, middle.y, 10.0, 10.0),
            "midpoint should be (10, 10), got ({}, {})",
            middle.x,
            middle.y
        );
        let sampled = sample_spline(&spline);
        assert!(
            sampled.iter().any(|p| coincident(p.x, p.y, 10.0, 10.0)),
            "the sampled polyline should pass through the true midpoint"
        );
    }

    #[test]
    fn chained_elements_form_closed_contours() {
        let square = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
            Vector3::new(0.0, 4.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        ];
        let elements: Vec<PathElement> = square
            .windows(2)
            .map(|w| {
                PathElement::Line(Line::new(
                    Affine::IDENTITY.point(w[0]),
                    Affine::IDENTITY.point(w[1]),
                ))
            })
            .collect();

        let shapes = chain(elements, &Some("cut".to_string()));

        assert_eq!(shapes.len(), 1);
        assert!(
            shapes[0].closed,
            "a contour that returns to its start is closed"
        );
        assert_eq!(shapes[0].layer.as_deref(), Some("cut"));
        assert!((shapes[0].total_length() - 16.0).abs() < 1e-9);
    }

    #[test]
    fn disconnected_chains_stay_separate() {
        let elements = vec![
            PathElement::Line(Line::new(Point::new(0.0, 0.0), Point::new(1.0, 0.0))),
            PathElement::Line(Line::new(Point::new(9.0, 9.0), Point::new(10.0, 9.0))),
        ];

        let shapes = chain(elements, &None);

        assert_eq!(shapes.len(), 2);
        assert!(!shapes[0].closed);
        assert!(!shapes[1].closed);
        assert!(shapes[0].layer.is_none());
    }

    #[test]
    fn placement_maps_are_composed_in_order() {
        let xf = Affine::translation(10.0, 20.0)
            .then(Affine::rotation(PI / 2.0))
            .then(Affine::scale(2.0, 3.0));
        let mapped = xf.point(Vector3::new(1.0, 0.0, 0.0));

        // scale → rotate by 90° → translate
        assert!(coincident(mapped.x, mapped.y, 10.0, 22.0));
    }

    #[test]
    fn insert_array_offset_follows_the_block_axes() {
        let insert = Insert::new("cell", Vector3::new(1.0, 2.0, 0.0))
            .with_rotation(PI / 2.0)
            .with_array(3, 2, 10.0, 4.0);

        // Column 1 sits one spacing along the block X axis, which the rotation
        // turns into world +Y; the origin block lands on the insert point.
        let first = placement(&insert, 0.0, 0.0).point(Vector3::ZERO);
        let column = placement(&insert, 1.0, 0.0).point(Vector3::ZERO);
        let row = placement(&insert, 0.0, 1.0).point(Vector3::ZERO);

        assert!(coincident(first.x, first.y, 1.0, 2.0));
        assert!(coincident(column.x, column.y, 1.0, 12.0));
        assert!(coincident(row.x, row.y, -3.0, 2.0));
    }

    fn xy(v: Vector3) -> Point {
        Point::new(v.x, v.y)
    }
}
