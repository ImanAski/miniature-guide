//! GDSII front end — flattens a GDSII library into [`Shape`] contours.
//!
//! Every cell that no other cell references is walked as a root; `SREF` and
//! `AREF` instances are expanded in place with their `STRANS` reflection,
//! magnification and rotation applied, and database-unit coordinates are
//! converted to millimetres through the library's `UNITS` record. `BOUNDARY`
//! and `BOX` elements become closed outlines, `PATH` elements become polylines
//! along their centre line — the stroke width has no [`Shape`] counterpart,
//! just as a DXF polyline drops its width. `TEXT` and `NODE` elements carry no
//! drawable geometry and are skipped.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use gds21::{
    GdsArrayRef, GdsElement, GdsError, GdsLibrary, GdsPath, GdsPoint, GdsStrans, GdsStruct,
    GdsStructRef,
};

use crate::core::geo::{Line, PathElement, Point, Shape};

use super::affine::Affine;

/// How deep `SREF`/`AREF` nesting is followed; also bounds reference cycles.
const MAX_NESTING: usize = 16;
/// Upper bound on the instances produced by a single `AREF`.
const MAX_ARRAY_INSTANCES: usize = 4096;
/// Points closer than this are treated as coincident — millimetres once a shape
/// has been placed, database units for raw coordinates.
const WELD_EPSILON: f64 = 1e-9;

/// Read a GDSII file and return its drawable contours.
pub fn parse(path: &Path) -> Result<Vec<Shape>, GdsError> {
    Ok(collect(&GdsLibrary::load(path)?))
}

/// Convert the geometry of `lib`'s root cells into shapes, references expanded.
pub fn collect(lib: &GdsLibrary) -> Vec<Shape> {
    let cells = lib
        .structs
        .iter()
        .map(|cell| (cell.name.as_str(), cell))
        .collect();
    let mut walker = Walker {
        cells,
        shapes: Vec::new(),
    };

    // Everything downstream is laid out in database units; scaling the root
    // keeps the placements in the units the file was written in.
    let scale = millimetres_per_db_unit(lib);
    let root = Place {
        xf: Affine::scale(scale, scale),
        depth: 0,
    };
    for cell in root_cells(lib) {
        walker.cell(cell, &root);
    }
    walker.shapes
}

/// Cells no other cell references — the library's entry points. When the
/// references form a cycle every cell is referenced, so fall back to all of them.
fn root_cells(lib: &GdsLibrary) -> Vec<&GdsStruct> {
    let referenced: HashSet<&str> = lib
        .structs
        .iter()
        .flat_map(|cell| cell.elems.iter())
        .filter_map(|element| match element {
            GdsElement::GdsStructRef(reference) => Some(reference.name.as_str()),
            GdsElement::GdsArrayRef(reference) => Some(reference.name.as_str()),
            _ => None,
        })
        .collect();

    let roots: Vec<&GdsStruct> = lib
        .structs
        .iter()
        .filter(|cell| !referenced.contains(cell.name.as_str()))
        .collect();
    if roots.is_empty() {
        lib.structs.iter().collect()
    } else {
        roots
    }
}

/// Millimetres per database unit, from the library's `UNITS` record. Falls back
/// to the GDSII default of one nanometre when the record is unusable.
fn millimetres_per_db_unit(lib: &GdsLibrary) -> f64 {
    let db_unit = lib.units.db_unit();
    if db_unit.is_finite() && db_unit > 0.0 {
        db_unit * 1000.0
    } else {
        1e-6
    }
}

// ─── Document walk ─────────────────────────────────────────────────────────

struct Walker<'a> {
    cells: HashMap<&'a str, &'a GdsStruct>,
    shapes: Vec<Shape>,
}

/// Placement state threaded through the recursion.
struct Place {
    xf: Affine,
    depth: usize,
}

impl<'a> Walker<'a> {
    fn cell(&mut self, cell: &'a GdsStruct, place: &Place) {
        for element in &cell.elems {
            match element {
                GdsElement::GdsBoundary(boundary) => {
                    self.push(contour(&boundary.xy, true, place.xf, boundary.layer));
                }
                GdsElement::GdsBox(bxl) => self.push(contour(&bxl.xy, true, place.xf, bxl.layer)),
                GdsElement::GdsPath(path) => self.push(path_shape(path, place.xf)),
                GdsElement::GdsStructRef(reference) => self.sref(reference, place),
                GdsElement::GdsArrayRef(reference) => self.aref(reference, place),
                // Labels and electrical nodes have no outline to expose.
                GdsElement::GdsTextElem(_) | GdsElement::GdsNode(_) => {}
            }
        }
    }

    fn push(&mut self, shape: Option<Shape>) {
        self.shapes.extend(shape);
    }

    /// Expand a cell instance into its definition.
    fn sref(&mut self, reference: &'a GdsStructRef, place: &Place) {
        if place.depth >= MAX_NESTING {
            return;
        }
        let Some(target) = self.cells.get(reference.name.as_str()).copied() else {
            return;
        };

        let inner = Place {
            xf: place
                .xf
                .then(placement(point(&reference.xy), reference.strans.as_ref())),
            depth: place.depth + 1,
        };
        self.cell(target, &inner);
    }

    /// Expand every instance of an `AREF` array.
    fn aref(&mut self, reference: &'a GdsArrayRef, place: &Place) {
        if place.depth >= MAX_NESTING {
            return;
        }
        let columns = reference.cols.max(1) as usize;
        let rows = reference.rows.max(1) as usize;
        if columns.saturating_mul(rows) > MAX_ARRAY_INSTANCES {
            return;
        }
        let Some(target) = self.cells.get(reference.name.as_str()).copied() else {
            return;
        };

        // The three lattice points are absolute: the second sits `columns`
        // pitches along the column vector from the reference point, the third
        // `rows` along the row vector (GDSII Stream Format Manual).
        let origin = point(&reference.xy[0]);
        let column = (point(&reference.xy[1]) - origin) / columns as f64;
        let row = (point(&reference.xy[2]) - origin) / rows as f64;

        for r in 0..rows {
            for c in 0..columns {
                let at = origin + column * (c as f64) + row * (r as f64);
                let inner = Place {
                    xf: place.xf.then(placement(at, reference.strans.as_ref())),
                    depth: place.depth + 1,
                };
                self.cell(target, &inner);
            }
        }
    }
}

/// Where a reference with `origin` lands, its `STRANS` options applied.
///
/// GDSII reflects about the x-axis first, then magnifies, then rotates, then
/// moves to the reference point — the reverse of the order they are composed in.
fn placement(origin: Point, strans: Option<&GdsStrans>) -> Affine {
    let mut xf = Affine::translation(origin.x, origin.y);
    let Some(strans) = strans else {
        return xf;
    };
    if let Some(angle) = strans.angle {
        xf = xf.then(Affine::rotation(angle.to_radians()));
    }
    if let Some(mag) = strans.mag {
        xf = xf.then(Affine::scale(mag, mag));
    }
    if strans.reflected {
        xf = xf.then(Affine::scale(1.0, -1.0));
    }
    xf
}

/// A raw database-unit coordinate.
fn point(p: &GdsPoint) -> Point {
    Point::new(p.x as f64, p.y as f64)
}

// ─── Element conversion ────────────────────────────────────────────────────

/// Join `vertices` into one shape, tagged with `layer`.
///
/// A closed contour repeats its first vertex at the end; the repeat is dropped
/// here and the ring is closed segment by segment instead, so a boundary that
/// fails to repeat it still comes out as a closed outline.
fn contour(vertices: &[GdsPoint], closed: bool, xf: Affine, layer: i16) -> Option<Shape> {
    let mut points: Vec<Point> = vertices
        .iter()
        .map(|p| xf.map(p.x as f64, p.y as f64))
        .collect();
    points.dedup_by(|a, b| coincident(*a, *b));
    if closed && points.len() >= 2 && coincident(points[0], points[points.len() - 1]) {
        points.pop();
    }

    let segments = if closed {
        if points.len() < 3 {
            return None;
        }
        points.len()
    } else {
        if points.len() < 2 {
            return None;
        }
        points.len() - 1
    };

    let elements = (0..segments)
        .map(|i| PathElement::Line(Line::new(points[i], points[(i + 1) % points.len()])))
        .collect();
    let mut shape = Shape::new(elements, closed);
    shape.layer = Some(layer.to_string());
    Some(shape)
}

/// The centre line of a `PATH` element; a path that returns to its start is a
/// closed loop. The stroke width and end extensions are not modelled.
fn path_shape(path: &GdsPath, xf: Affine) -> Option<Shape> {
    let closed =
        path.xy.len() >= 3 && coincident(point(&path.xy[0]), point(&path.xy[path.xy.len() - 1]));
    contour(&path.xy, closed, xf, path.layer)
}

fn coincident(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() <= WELD_EPSILON && (a.y - b.y).abs() <= WELD_EPSILON
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = "tests/data/gds_example.gds";

    /// The fixture lives under `tests/`, which is not tracked in git, so a
    /// fresh checkout (CI) may not have it. Returns `None` in that case and
    /// the test skips; local runs with the fixture present still execute.
    fn library() -> Option<GdsLibrary> {
        if !Path::new(EXAMPLE).exists() {
            eprintln!("skipping {EXAMPLE}: fixture not present in this checkout");
            return None;
        }
        Some(GdsLibrary::load(EXAMPLE).expect("gds_example.gds should load"))
    }

    fn bounds_of(shapes: &[Shape]) -> crate::core::geo::Rect {
        shapes
            .iter()
            .filter_map(|shape| shape.bounds())
            .reduce(|a, b| a.union(&b))
            .expect("expected drawable geometry")
    }

    #[test]
    fn parse_gds_file() {
        let Some(path) = Path::new(EXAMPLE).exists().then_some(EXAMPLE) else {
            eprintln!("skipping {EXAMPLE}: fixture not present in this checkout");
            return;
        };
        let shapes = parse(Path::new(path)).expect("gds_example.gds should parse");

        assert!(!shapes.is_empty(), "expected drawable geometry");
        assert!(
            shapes.iter().all(|s| !s.elements.is_empty()),
            "shapes never carry empty element lists"
        );
        assert!(
            shapes.iter().all(|s| s.closed),
            "every boundary in the example is a closed outline"
        );
        assert!(
            shapes.iter().all(|s| s
                .bounds()
                .is_some_and(|b| b.min.x.is_finite() && b.max.x.is_finite())),
            "every shape should report finite bounds"
        );
        assert!(
            shapes
                .iter()
                .all(|s| s.elements.iter().all(|e| matches!(e, PathElement::Line(_)))),
            "GDSII carries polygons, not arcs"
        );
    }

    #[test]
    fn only_the_unreferenced_cell_is_a_root() {
        let Some(lib) = library() else { return };
        let roots: Vec<&str> = root_cells(&lib)
            .iter()
            .map(|cell| cell.name.as_str())
            .collect();

        assert_eq!(roots, ["TOP"], "TOP is the only cell nothing references");
    }

    #[test]
    fn hierarchy_is_expanded_from_the_root_cell() {
        let Some(lib) = library() else { return };
        let shapes = collect(&lib);

        // TOP's 2 boundaries + FABRIC_OETS' 14, then one SWITCH_2X2 (19) with
        // its PHASE_SHIFTER_SCALN (12), four GC_TE1550 (22 each) and three
        // PAD_BOND (3 each): 2 + 14 + 19 + 12 + 4·22 + 3·3 = 144 outlines.
        // The five TEXT labels add nothing.
        assert_eq!(shapes.len(), 144);
    }

    #[test]
    fn database_units_are_converted_to_millimetres() {
        let Some(lib) = library() else { return };
        let bounds = bounds_of(&collect(&lib));

        // The layout spans ~3.2 mm; left in database units it would span ~3.2e6.
        assert!(
            bounds.width() > 1.0 && bounds.width() < 10.0,
            "expected a chip-sized drawing, got {} mm wide",
            bounds.width()
        );
        assert!(
            bounds.height() < 10.0,
            "expected a chip-sized drawing, got {} mm tall",
            bounds.height()
        );
    }

    #[test]
    fn strans_rotation_moves_the_instance() {
        let Some(lib) = library() else { return };
        let shapes = collect(&lib);
        let vertices: Vec<Point> = shapes.iter().flat_map(|s| s.vertices()).collect();
        let at = |x: f64, y: f64| {
            let target = Point::new(x, y);
            vertices
                .iter()
                .any(|v| (v.x - target.x).abs() < 1e-9 && (v.y - target.y).abs() < 1e-9)
        };

        // GC_TE1550's first corner sits at (150300, -5000) database units;
        // FABRIC_OETS references the cell four times, twice upright and twice
        // rotated by 180°. Coordinates are millimetres.
        assert!(
            at(1.5703, -0.0435),
            "the upright instance should land at (1420000, -38500)"
        );
        assert!(
            at(-0.7503, -0.0335),
            "the rotated instance should land at (-600000, -38500)"
        );
    }

    #[test]
    fn layers_are_carried_over() {
        let Some(lib) = library() else { return };
        let shapes = collect(&lib);

        assert!(
            shapes.iter().any(|s| s.layer.as_deref() == Some("1")),
            "expected shapes tagged with GDSII layer 1"
        );
        assert!(
            shapes.iter().all(|s| s.layer.is_some()),
            "every GDSII element names its layer"
        );
        assert!(
            shapes.iter().all(|s| s.layer.as_deref() != Some("99")),
            "layer 99 only carries labels, which are not drawable"
        );
    }
}
