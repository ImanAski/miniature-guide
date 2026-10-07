//! Geometry analysis and tile partitioning.
//!
//! `segment.rs` owns the analysis that turns shapes into tileable rings:
//! area/perimeter/fill factor, minimum feature width estimate, connected
//! components, hole counts, and spatial tiling. Analysis is independent of the
//! parser and of the cost model so it can be unit-tested directly on synthetic
//! geometry.

use crate::core::geo::{Point, Rect, Shape};
use crate::hybrid::Tile;

#[derive(Debug, Clone, PartialEq)]
pub struct GeometryStats {
    pub area: f64,
    pub perimeter: f64,
    pub bounding_box: Rect,
    pub fill_factor: f64,
    pub min_feature_width: f64,
    pub polygon_count: usize,
    pub vertex_count: usize,
    pub hole_count: usize,
    pub component_count: usize,
}

/// Approximate minimum feature width from edge-to-edge distance across the
/// shape's thin parts: for each interior vertex, the distance to the opposite
/// edge of the thin strip is bounded by the local geometry; as a first
/// analytical estimate we take the smallest distance from any vertex to its
/// nearest non-adjacent edge. This is deterministic and cheap. If no such
/// distance exists (e.g. solid square), fall back to the raster pitch or the
/// beam spot: the caller enforces the minimum-feature constraint anyway.
pub fn min_feature_width_from_rings(rings: &[Vec<Point>], bbox: &Rect) -> f64 {
    if rings.is_empty() {
        return bbox.width().max(bbox.height());
    }
    let mut min_d = f64::INFINITY;
    for ring in rings {
        let n = ring.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let p = ring[i];
            // Search all other segments to find the closest edge not adjacent.
            for j in 0..n {
                let k = (j + 1) % n;
                if j == i || j == (i + n - 1) % n || k == i {
                    continue;
                }
                let a = ring[j];
                let b = ring[k];
                let d = point_to_segment_distance(p, a, b);
                if d < min_d {
                    min_d = d;
                }
            }
        }
    }
    if !min_d.is_finite() {
        bbox.width().min(bbox.height()).max(1e-9)
    } else {
        min_d
    }
}

/// Split `rings` into (bulk, narrow) by hydraulic feature width vs `threshold`.
///
/// Bulk rings are wide enough for the beam to sweep safely at the raster
/// pitch; narrow rings must be traced with the beam. This is the hybrid
/// strategy: rasterise the bulk, vector-trace the narrow, never rasterise a
/// feature the optics cannot resolve. Deterministic, preserves input order.
pub fn split_bulk_narrow(
    rings: &[Vec<Point>],
    bbox: &Rect,
    threshold: f64,
) -> (Vec<Vec<Point>>, Vec<Vec<Point>>) {
    let mut bulk = Vec::new();
    let mut narrow = Vec::new();
    for r in rings {
        let w = min_feature_width_from_rings(std::slice::from_ref(r), bbox);
        if w.is_finite() && w >= threshold {
            bulk.push(r.clone());
        } else {
            narrow.push(r.clone());
        }
    }
    (bulk, narrow)
}

fn point_to_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len2 = abx * abx + aby * aby;
    if len2 <= f64::EPSILON {
        return p.distance(a);
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0);
    let proj = Point::new(a.x + t * abx, a.y + t * aby);
    p.distance(proj)
}

/// Normalize a closed ring by removing the closing duplicate and asserting
/// non-degenerate. Rings are treated as closed.
fn to_closed_ring(shape: &Shape, tolerance: f64) -> Vec<Point> {
    let mut ring = shape.ring(tolerance);
    if ring.len() > 1 && ring.first() == ring.last() {
        ring.pop();
    }
    ring
}

pub fn analyze_shape(shape: &Shape, tolerance: f64) -> Option<GeometryStats> {
    if !shape.closed {
        return None;
    }
    let ring = to_closed_ring(shape, tolerance);
    if ring.len() < 3 {
        return None;
    }
    let bbox = crate::core::geo::Rect::from_points(&ring)?;
    let area = ring_area(&ring);
    let perimeter = ring_perimeter(&ring);
    let fill_factor = if bbox.is_empty() {
        0.0
    } else {
        (area.abs() / (bbox.width() * bbox.height())).clamp(0.0, 1.0)
    };
    Some(GeometryStats {
        area,
        perimeter,
        bounding_box: bbox,
        fill_factor,
        min_feature_width: min_feature_width_from_rings(&[ring.clone()], &bbox),
        polygon_count: 1,
        vertex_count: ring.len(),
        hole_count: 0,
        component_count: 1,
    })
}

fn ring_area(r: &[Point]) -> f64 {
    let mut a = 0.0;
    for i in 0..r.len() {
        let p = r[i];
        let q = r[(i + 1) % r.len()];
        a += p.x * q.y - q.x * p.y;
    }
    a.abs() / 2.0
}
fn ring_perimeter(r: &[Point]) -> f64 {
    let mut l = 0.0;
    for i in 0..r.len() {
        l += r[i].distance(r[(i + 1) % r.len()]);
    }
    l
}

/// Partition `shapes` into rectangular tiles aligned to `tile_size_mm`.
/// For every tile, rings are clipped to the tile bounds: rings that produce no
/// interior points are omitted, keeping tiles sparse. Deterministic.
pub fn partition_into_tiles(shapes: &[Shape], tile_size_mm: f64, max_tiles: usize) -> Vec<Tile> {
    if tile_size_mm <= 0.0 {
        return Vec::new();
    }
    let world = crate::core::geo::Rect::from_points(
        &shapes
            .iter()
            .filter_map(|s| s.bounds())
            .flat_map(|b| vec![b.min, b.max])
            .collect::<Vec<_>>(),
    );
    if world.is_none() {
        return Vec::new();
    }
    let mut world = world.expect("checked");
    if world.width() <= 0.0 {
        world.max.x = world.min.x + tile_size_mm;
    }
    if world.height() <= 0.0 {
        world.max.y = world.min.y + tile_size_mm;
    }

    let mut ts = tile_size_mm;
    loop {
        let nx = ((world.width() / ts).ceil() as usize).max(1);
        let ny = ((world.height() / ts).ceil() as usize).max(1);
        if nx * ny <= max_tiles || ts > world.width().max(world.height()) * 4.0 {
            break;
        }
        ts *= 2.0;
    }

    let nx = ((world.width() / ts).ceil() as usize).max(1);
    let ny = ((world.height() / ts).ceil() as usize).max(1);
    let mut tiles: Vec<Tile> = Vec::with_capacity(nx * ny);

    for ty in 0..ny {
        for tx in 0..nx {
            let tmin = Point::new(world.min.x + tx as f64 * ts, world.min.y + ty as f64 * ts);
            let tmax = Point::new(tmin.x + ts, tmin.y + ts);
            let tb = Rect::new(tmin, tmax);
            let mut rings: Vec<Vec<Point>> = Vec::new();
            for s in shapes.iter().filter(|s| s.closed) {
                let r = to_closed_ring(s, 1e-6);
                if r.len() < 3 {
                    continue;
                }
                let cr = super::clip_ring_to_rect(&r, tb);
                if cr.len() >= 3 {
                    rings.push(cr);
                }
            }
            tiles.push(Tile {
                index: (tx, ty),
                bounds: tb,
                rings,
            });
        }
    }
    tiles
}
