use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};

/// Type alias for Point — used throughout the codebase
pub type Pt = Point;

/// Directional vector (unit-length not enforced)
pub type Vec2 = Point;

/// 2D transformation matrix (column-major: rotation+translation)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub cos_a: f64,
    pub sin_a: f64,
    pub tx: f64,
    pub ty: f64,
}

impl Transform {
    pub fn identity() -> Self {
        Transform {
            cos_a: 1.0,
            sin_a: 0.0,
            tx: 0.0,
            ty: 0.0,
        }
    }

    pub fn rotation(angle: f64) -> Self {
        Transform {
            cos_a: angle.cos(),
            sin_a: angle.sin(),
            tx: 0.0,
            ty: 0.0,
        }
    }

    pub fn translation(x: f64, y: f64) -> Self {
        Self {
            cos_a: 1.0,
            sin_a: 0.0,
            tx: x,
            ty: y,
        }
    }

    pub fn compose(self, other: Self) -> Self {
        // other is applied first, then self
        Transform {
            cos_a: self.cos_a * other.cos_a - self.sin_a * other.sin_a,
            sin_a: self.cos_a * other.sin_a + self.sin_a * other.cos_a,
            tx: self.cos_a * other.tx - self.sin_a * other.ty + self.tx,
            ty: self.sin_a * other.tx + self.cos_a * other.ty + self.ty,
        }
    }

    pub fn transform_point(&self, p: Point) -> Point {
        Point::new(
            self.cos_a * p.x - self.sin_a * p.y + self.tx,
            self.sin_a * p.x + self.cos_a * p.y + self.ty,
        )
    }

    pub fn transform_vec(&self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.cos_a * v.x - self.sin_a * v.y,
            self.sin_a * v.x + self.cos_a * v.y,
        )
    }
}

// ─── Point ───────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const ORIGIN: Point = Point { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }

    pub fn distance(self, other: Point) -> f64 {
        (self - other).length()
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn angle_to(self, other: Point) -> f64 {
        (other.y - self.y).atan2(other.x - self.x)
    }

    /// Rotate point around a center
    pub fn rotate_around(self, center: Point, angle: f64) -> Point {
        let dx = self.x - center.x;
        let dy = self.y - center.y;
        Point::new(
            center.x + dx * angle.cos() - dy * angle.sin(),
            center.y + dx * angle.sin() + dy * angle.cos(),
        )
    }

    /// Snap to grid
    pub fn snap(self, grid: f64) -> Point {
        Point::new(
            (self.x / grid).round() * grid,
            (self.y / grid).round() * grid,
        )
    }
}

impl std::ops::Add for Point {
    type Output = Point;
    fn add(self, other: Point) -> Point {
        Point::new(self.x + other.x, self.y + other.y)
    }
}

impl std::ops::Sub for Point {
    type Output = Point;
    fn sub(self, other: Point) -> Point {
        Point::new(self.x - other.x, self.y - other.y)
    }
}

impl std::ops::Mul<f64> for Point {
    type Output = Point;
    fn mul(self, k: f64) -> Point {
        Point::new(self.x * k, self.y * k)
    }
}

impl std::ops::Div<f64> for Point {
    type Output = Point;
    fn div(self, k: f64) -> Point {
        Point::new(self.x / k, self.y / k)
    }
}

impl std::ops::Neg for Point {
    type Output = Point;
    fn neg(self) -> Point {
        Point::new(-self.x, -self.y)
    }
}

impl std::ops::AddAssign for Point {
    fn add_assign(&mut self, other: Point) {
        *self = *self + other;
    }
}

impl std::ops::SubAssign for Point {
    fn sub_assign(&mut self, other: Point) {
        *self = *self - other;
    }
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

// ─── Rect ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Rect {
    pub min: Point,
    pub max: Point,
}

impl Rect {
    pub fn new(a: Point, b: Point) -> Self {
        Rect {
            min: Point::new(a.x.min(b.x), a.y.min(b.y)),
            max: Point::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    pub fn from_points<'a>(pts: impl IntoIterator<Item = &'a Point>) -> Option<Self> {
        let mut r: Option<Rect> = None;
        for p in pts {
            r = Some(match r {
                None => Rect::new(*p, *p),
                Some(r) => Rect::new(
                    Point::new(r.min.x.min(p.x), r.min.y.min(p.y)),
                    Point::new(r.max.x.max(p.x), r.max.y.max(p.y)),
                ),
            });
        }
        r
    }

    pub fn from_min_max(x_min: f64, y_min: f64, x_max: f64, y_max: f64) -> Self {
        Rect {
            min: Point::new(x_min, y_min),
            max: Point::new(x_max, y_max),
        }
    }

    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }

    pub fn center(&self) -> Point {
        (self.min + self.max) * 0.5
    }

    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    pub fn is_empty(&self) -> bool {
        self.width() <= 0.0 || self.height() <= 0.0
    }

    pub fn contains(&self, point: Point) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    pub fn union(&self, other: &Rect) -> Rect {
        Rect::new(
            Point::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            Point::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        )
    }

    pub fn intersect(&self, other: &Rect) -> Option<Rect> {
        let x_min = self.min.x.max(other.min.x);
        let y_min = self.min.y.max(other.min.y);
        let x_max = self.max.x.min(other.max.x);
        let y_max = self.max.y.min(other.max.y);
        if x_min < x_max && y_min < y_max {
            Some(Rect::from_min_max(x_min, y_min, x_max, y_max))
        } else {
            None
        }
    }

    pub fn grow(&self, amount: f64) -> Rect {
        Rect::new(
            Point::new(self.min.x - amount, self.min.y - amount),
            Point::new(self.max.x + amount, self.max.y + amount),
        )
    }

    pub fn clamp(&self, amount: f64) -> Rect {
        Rect::new(
            Point::new(self.min.x + amount, self.min.y + amount),
            Point::new(self.max.x - amount, self.max.y - amount),
        )
    }

    pub fn transform(&self, t: &Transform) -> Rect {
        let corners = self.corners();
        let transformed: Vec<Point> = corners.iter().map(|c| t.transform_point(*c)).collect();
        Rect::from_points(&transformed).expect("transformed corners should produce a valid rect")
    }

    pub fn corners(&self) -> [Point; 4] {
        [
            self.min,
            Point::new(self.max.x, self.min.y),
            self.max,
            Point::new(self.min.x, self.max.y),
        ]
    }

    /// Convert to center-size representation
    pub fn to_center_size(&self) -> (Point, Point) {
        (
            self.center(),
            Point::new(self.width() * 0.5, self.height() * 0.5),
        )
    }
}

// ─── Arc ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Arc {
    pub center: Pt,
    pub radius: f64,
    pub start_angle: f64,
    /// Sweep in radians. Sign carries direction. `|sweep|` may exceed 2π.
    pub sweep: f64,
}

impl Arc {
    pub fn from_arc_points(start: Point, end: Point, center: Point) -> Self {
        let a0 = center.angle_to(start);
        let a1 = center.angle_to(end);
        Arc {
            center,
            radius: center.distance(start),
            start_angle: a0,
            sweep: norm_sweep(a1 - a0),
        }
    }

    pub fn full_circle(center: Point, radius: f64) -> Self {
        Arc {
            center,
            radius,
            start_angle: 0.0,
            sweep: TAU,
        }
    }

    pub fn start(&self) -> Point {
        self.point_at(0.0)
    }

    pub fn end(&self) -> Point {
        self.point_at(1.0)
    }

    pub fn is_full_circle(&self) -> bool {
        (self.sweep.abs() - TAU).abs() < 1e-9
    }

    pub fn point_at(&self, t: f64) -> Point {
        let a = self.start_angle + self.sweep * t;
        self.center + Point::new(self.radius * a.cos(), self.radius * a.sin())
    }

    /// Approximate arc with line segments
    pub fn to_segments(&self, segments: usize) -> Vec<Point> {
        let mut pts = Vec::with_capacity(segments + 1);
        for i in 0..=segments {
            let t = i as f64 / segments as f64;
            pts.push(self.point_at(t));
        }
        pts
    }
}

// ─── Line Segment ────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Line {
    pub start: Point,
    pub end: Point,
}

impl Line {
    pub fn new(start: Point, end: Point) -> Self {
        Line { start, end }
    }

    pub fn length(&self) -> f64 {
        self.start.distance(self.end)
    }

    pub fn midpoint(&self) -> Point {
        (self.start + self.end) * 0.5
    }

    pub fn direction(&self) -> Vec2 {
        (self.end - self.start).normalize()
    }

    /// Whether point is on this line segment (with epsilon)
    pub fn contains_point(&self, p: Point, epsilon: f64) -> bool {
        let dx = (self.end - self.start).length();
        if dx < epsilon {
            return (p - self.start).length() < epsilon;
        }
        // Project p onto line, check if 0 <= t <= 1
        let t = ((p.x - self.start.x) * (self.end.x - self.start.x)
            + (p.y - self.start.y) * (self.end.y - self.start.y))
            / (dx * dx);
        if t < -epsilon || t > 1.0 + epsilon {
            return false;
        }
        // Check perpendicular distance
        let proj = self.start + self.direction() * t * dx;
        (p - proj).length() < epsilon
    }
}

impl Vec2 {
    pub fn normalize(self) -> Vec2 {
        let len = self.length();
        if len < 1e-12 {
            Vec2::default()
        } else {
            self / len
        }
    }
}

// ─── Geometry Primitives (used by engine) ────────────────────────────

/// A single path element — either a line segment or an arc
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PathElement {
    Line(Line),
    Arc(Arc),
}

impl PathElement {
    pub fn bounds(&self) -> Rect {
        match self {
            PathElement::Line(l) => Rect::new(l.start, l.end),
            PathElement::Arc(a) => {
                // Sample multiple points to get accurate bounds
                let samples = 32;
                let pts: Vec<Point> = (0..=samples)
                    .map(|i| a.point_at(i as f64 / samples as f64))
                    .collect();
                Rect::from_points(&pts).expect("arc samples should produce a valid rect")
            }
        }
    }

    pub fn approx_segments(&self, tolerance: f64) -> Vec<Point> {
        match self {
            PathElement::Line(l) => vec![l.start, l.end],
            PathElement::Arc(a) => {
                // Estimate number of segments needed based on curvature and tolerance
                let arc_length = a.sweep.abs() * a.radius;
                if arc_length < tolerance {
                    return vec![a.start(), a.end()];
                }
                let seg_len = (2.0 * tolerance * arc_length.min(1.0)).sqrt().max(0.001);
                let segments = (arc_length / seg_len).ceil().max(1.0) as usize;
                a.to_segments(segments)
            }
        }
    }
}

/// A complete shape — a sequence of connected path elements (a contour)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Shape {
    pub elements: Vec<PathElement>,
    pub closed: bool,
    pub layer: Option<String>,
}

impl Shape {
    pub fn new(elements: Vec<PathElement>, closed: bool) -> Self {
        Shape {
            elements,
            closed,
            layer: None,
        }
    }

    pub fn bounds(&self) -> Option<Rect> {
        Rect::from_points(&self.vertices())
    }

    pub fn vertices(&self) -> Vec<Point> {
        let mut verts = Vec::new();
        for elem in &self.elements {
            match elem {
                PathElement::Line(l) => {
                    if verts.is_empty() || verts.last().copied() != Some(l.start) {
                        verts.push(l.start);
                    }
                    verts.push(l.end);
                }
                PathElement::Arc(a) => {
                    if verts.is_empty() || verts.last().copied() != Some(a.start()) {
                        verts.push(a.start());
                    }
                    verts.push(a.end());
                }
            }
        }
        verts
    }

    pub fn total_length(&self) -> f64 {
        self.elements
            .iter()
            .map(|e| match e {
                PathElement::Line(l) => l.length(),
                PathElement::Arc(a) => a.sweep.abs() * a.radius,
            })
            .sum()
    }

    pub fn transform(&self, t: &Transform) -> Shape {
        Shape {
            elements: self.elements.iter().map(|e| e.transform(t)).collect(),
            closed: self.closed,
            layer: self.layer.clone(),
        }
    }
}

impl PathElement {
    pub fn transform(&self, t: &Transform) -> PathElement {
        match self {
            PathElement::Line(l) => PathElement::Line(Line {
                start: t.transform_point(l.start),
                end: t.transform_point(l.end),
            }),
            PathElement::Arc(a) => {
                // Arc transform: rotate center, keep radius, adjust angles
                let new_center = t.transform_point(a.center);
                let dir0 = (a.start() - a.center).normalize();
                let dir1 = (a.end() - a.center).normalize();
                let _transformed_dir0 = t.transform_vec(dir0);
                let _transformed_dir1 = t.transform_vec(dir1);
                let start_angle = (t.transform_point(a.start()) - new_center).angle_to(new_center);
                PathElement::Arc(Arc {
                    center: new_center,
                    radius: a.radius,
                    start_angle,
                    sweep: a.sweep, // rotation preserves sweep
                })
            }
        }
    }
}

// ─── Hatch / Raster patterns ─────────────────────────────────────────

/// Hatch pattern type for raster fill
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HatchPattern {
    Solid,
    Lines { spacing: f64, angle: f64 },      // parallel lines
    CrossHatch { spacing: f64, angle: f64 }, // 45° crossed
}

impl HatchPattern {
    pub fn spacing(&self) -> f64 {
        match self {
            HatchPattern::Solid => 0.0,
            HatchPattern::Lines { spacing, .. } | HatchPattern::CrossHatch { spacing, .. } => {
                *spacing
            }
        }
    }
}

/// A rasterized band (horizontal strip)
#[derive(Debug, Clone)]
pub struct RasterBand {
    pub y: f64,
    pub passes: Vec<RasterPass>,
}

#[derive(Debug, Clone)]
pub struct RasterPass {
    pub start_x: f64,
    pub end_x: f64,
    pub direction: i32, // 1 = left→right, -1 = right→left
}

/// Complete raster plan for a shape
#[derive(Debug, Clone)]
pub struct RasterPlan {
    pub bands: Vec<RasterBand>,
    pub hatch: HatchPattern,
    pub bbox: Rect,
    pub layer: Option<String>,
}

// ─── Helpers ──────────────────────────────────────────────────────────

/// Normalise a sweep to (-π, π]
pub fn norm_sweep(s: f64) -> f64 {
    let mut s = s % TAU;
    if s > PI {
        s -= TAU;
    }
    if s <= -PI {
        s += TAU;
    }
    s
}

/// Round a float to N decimal places
pub fn round_to(val: f64, places: i32) -> f64 {
    let factor = 10_f64.powi(places);
    (val * factor).round() / factor
}

/// Format a float for display (trim trailing zeros)
pub fn fmt_f(val: f64) -> String {
    if val.abs() < 1e-9 {
        return "0".to_string();
    }
    if val.abs() < 0.01 {
        return format!("{:.4}", val);
    }
    if val.abs() < 1.0 {
        return format!("{:.3}", val);
    }
    format!("{:.2}", val)
}

// ─── 3D Preview (isometric camera, used by main.rs UI) ─────────────────

/// Orbit camera state for 3D preview
#[derive(Debug, Clone, Copy)]
pub struct OrbitCamera {
    pub azimuth: f64,   // horizontal rotation (radians)
    pub elevation: f64, // vertical rotation (radians)
    pub distance: f64,  // zoom
    pub center: Point3D,
}

/// 3D point
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Point3D { x, y, z }
    }

    fn to_2d(self) -> Point {
        Point::new(self.x, self.y)
    }

    fn from_2d(p: Point, z: f64) -> Self {
        Point3D { x: p.x, y: p.y, z }
    }
}

/// Transform a 3D point by the orbit camera
impl OrbitCamera {
    pub fn new(bounds: &Rect, z: f64) -> Self {
        let center = bounds.center().to_3d(z * 0.5);
        let diag = ((bounds.width().powi(2) + bounds.height().powi(2)).sqrt() * 0.8)
            .max(100.0)
            .min(5000.0);
        OrbitCamera {
            azimuth: -std::f64::consts::FRAC_PI_4, // -45° for isometric feel
            elevation: std::f64::consts::FRAC_PI_6, // 30° up
            distance: diag,
            center,
        }
    }

    /// Rotate a 3D point into camera view and project to 2D screen coords
    /// (screen_x, screen_y) — egui coordinate system (Y down)
    pub fn project(&self, pt: Point3D, canvas_size: egui::Vec2) -> (f32, f32) {
        // Translate relative to center
        let dx = pt.x - self.center.x;
        let dy = pt.y - self.center.y;
        let dz = pt.z - self.center.z;

        // Rotate around Y axis (azimuth)
        let cos_a = self.azimuth.cos();
        let sin_a = self.azimuth.sin();
        let rx = dx * cos_a - dz * sin_a;
        let rz = dx * sin_a + dz * cos_a;

        // Rotate around X axis (elevation)
        let cos_e = self.elevation.cos();
        let sin_e = self.elevation.sin();
        let ry = dy * cos_e - rz * sin_e;
        let rz2 = dy * sin_e + rz * cos_e;

        // Perspective divide (weak perspective)
        let depth = self.distance + rz2;
        let scale = self.distance / depth.max(0.1);

        // Project to screen (flip Y for egui)
        let cw = canvas_size.x as f64;
        let ch = canvas_size.y as f64;
        let sx = cw * 0.5 + rx * scale * cw * 0.4;
        let sy = ch * 0.5 - ry * scale * ch * 0.4;

        (sx as f32, sy as f32)
    }

    /// Get Z-depth for painter's algorithm sorting (higher = closer to camera)
    pub fn depth(&self, pt: Point3D) -> f64 {
        let dx = pt.x - self.center.x;
        let dy = pt.y - self.center.y;
        let dz = pt.z - self.center.z;
        let cos_a = self.azimuth.cos();
        let sin_a = self.azimuth.sin();
        let rz = dx * sin_a + dz * cos_a;
        let cos_e = self.elevation.cos();
        let sin_e = self.elevation.sin();
        dy * sin_e + rz * cos_e
    }
}

impl Point {
    fn to_3d(self, z: f64) -> Point3D {
        Point3D::new(self.x, self.y, z)
    }
}
