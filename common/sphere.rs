use bvh::{BoundingVolume, Enlarge, IntersectSelf, RayCast, Vector};
use glam::Vec3;

#[derive(Clone, Copy)]
#[repr(align(16))]
pub struct BoundingSphere {
    pos: Vec3,
    r: f32,
}

impl BoundingSphere {
    pub fn new(pos: Vec3, r: f32) -> Self {
        debug_assert!(r >= 0.0);
        Self { pos, r }
    }
    #[inline]
    pub fn intersects_scalar(&self, other: &BoundingSphere) -> bool {
        let dist2 = (self.pos - other.pos).length_squared();

        let max_dist = self.r + other.r;
        let max_dist2 = max_dist * max_dist;

        dist2 <= max_dist2
    }
}

pub struct BSVector(Vec3);
impl Vector for BSVector {
    fn distance_heuristic(&self, other: &Self) -> f32 {
        self.0.distance_squared(other.0)
    }
}

impl BoundingVolume for BoundingSphere {
    type Point = BSVector;
    fn center(&self) -> Self::Point {
        BSVector(self.pos)
    }
    fn contains(&self, other: &Self) -> bool {
        self.r >= self.pos.distance(other.pos) + other.r
    }
    fn surface_area_heuristic(&self) -> f32 {
        self.r * self.r
    }
    fn union(&self, other: &Self) -> Self {
        let dir = other.pos - self.pos;
        let dist = dir.length();

        // One sphere already contains the other.
        if dist + other.r <= self.r {
            return *self;
        }
        if dist + self.r <= other.r {
            return *other;
        }

        if dist >= f32::EPSILON {
            let dir = dir * (1.0 / dist);
            let p2 = other.pos + dir * other.r;
            let p1 = self.pos - dir * self.r;

            let center = 0.5 * (p1 + p2);
            let r = (self.r + other.r + dist) * 0.5;

            Self { pos: center, r }
        } else {
            // centers are too close.
            let r = self.r.max(other.r) + f32::EPSILON;
            Self { pos: self.pos, r }
        }
    }
}

impl IntersectSelf for BoundingSphere {
    fn intersects(&self, other: &Self) -> bool {
        self.intersects_scalar(other)
    }
}

impl Enlarge for BoundingSphere {
    fn enlarge(&self, r: f32) -> Self {
        Self {
            pos: self.pos,
            r: self.r + r,
        }
    }
}

impl RayCast for BoundingSphere {
    type Vector = BSVector;
    /// `dir` need to be normalized before passing to this function.
    fn ray_cast(&self, origin: &Self::Vector, dir: &Self::Vector) -> Option<f32> {
        let origin = origin.0;
        let dir = dir.0;

        let origin_center = self.pos - origin;
        let oc2 = origin_center.length_squared();
        let r2 = self.r * self.r;

        if r2 >= oc2 {
            // origin is inside the bounding sphere
            return Some(0.0);
        }

        let origin_proj_l = origin_center.dot(dir);
        if origin_proj_l < 0.0 {
            // ray is shooting away from the sphere,
            // and the origin is not inside it.
            return None;
        }

        let op2 = origin_proj_l * origin_proj_l;

        let dist2 = oc2 - op2;
        let touch_proj_l2 = r2 - dist2;

        if touch_proj_l2 < 0.0 {
            // distance between the sphere center and the line
            // is larger than the radius.
            return None;
        }

        let t = op2.sqrt() - touch_proj_l2.sqrt();
        Some(t)
    }
}
