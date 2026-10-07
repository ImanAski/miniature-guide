//! 2D placement shared by the file-format front ends.
//!
//! Each reader threads one of these through its document walk so that block
//! and cell references land in world coordinates.

use acadrust::Vector3;

use crate::core::geo::Point;

/// 2D affine map, `[a b tx; c d ty]`, applied to the XY plane of the source.
#[derive(Clone, Copy, Debug)]
pub struct Affine {
    a: f64,
    b: f64,
    tx: f64,
    c: f64,
    d: f64,
    ty: f64,
}

impl Affine {
    pub const IDENTITY: Affine = Affine {
        a: 1.0,
        b: 0.0,
        tx: 0.0,
        c: 0.0,
        d: 1.0,
        ty: 0.0,
    };

    pub fn translation(tx: f64, ty: f64) -> Self {
        Affine {
            tx,
            ty,
            ..Affine::IDENTITY
        }
    }

    pub fn scale(sx: f64, sy: f64) -> Self {
        Affine {
            a: sx,
            d: sy,
            ..Affine::IDENTITY
        }
    }

    pub fn rotation(angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();
        Affine {
            a: cos,
            b: -sin,
            c: sin,
            d: cos,
            ..Affine::IDENTITY
        }
    }

    /// `self ∘ other` — `other` acts on the point first.
    pub fn then(self, other: Self) -> Self {
        Affine {
            a: self.a * other.a + self.b * other.c,
            b: self.a * other.b + self.b * other.d,
            tx: self.a * other.tx + self.b * other.ty + self.tx,
            c: self.c * other.a + self.d * other.c,
            d: self.c * other.b + self.d * other.d,
            ty: self.c * other.tx + self.d * other.ty + self.ty,
        }
    }

    /// Map a coordinate pair onto the XY plane.
    pub fn map(&self, x: f64, y: f64) -> Point {
        Point::new(
            self.a * x + self.b * y + self.tx,
            self.c * x + self.d * y + self.ty,
        )
    }

    pub fn point(&self, v: Vector3) -> Point {
        self.map(v.x, v.y)
    }

    /// Rotation angle of the linear part; only meaningful when [`Self::similarity`]
    /// reports a uniform scale.
    pub fn angle(&self) -> f64 {
        self.c.atan2(self.a)
    }

    /// Upper bound on how much the map can stretch a length.
    pub fn stretch_bound(&self) -> f64 {
        (self.a.abs() + self.b.abs()).max(self.c.abs() + self.d.abs())
    }

    /// Uniform scale and winding direction when the map keeps circles circular,
    /// `None` when an arc would turn into an ellipse.
    pub fn similarity(&self) -> Option<(f64, bool)> {
        let column = self.a.hypot(self.c);
        let row = self.b.hypot(self.d);
        if column <= WELD_EPSILON || row <= WELD_EPSILON {
            return None;
        }
        let dot = self.a * self.b + self.c * self.d;
        if (column - row).abs() > 1e-9 * column.max(row) || dot.abs() > 1e-9 * column * row {
            return None;
        }
        Some((column, self.a * self.d - self.b * self.c > 0.0))
    }
}

/// Degenerate-scale cutoff used by [`Affine::similarity`].
const WELD_EPSILON: f64 = 1e-9;
