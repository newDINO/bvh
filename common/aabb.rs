use bvh::{BoundingVolume, Vec3};
use nalgebra as na;

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: na::Vector3<f32>,
    pub max: na::Vector3<f32>,
}

impl Aabb {
    pub fn intersection(&self, other: &Aabb) -> Option<Aabb> {
        let result = Aabb {
            min: self.min.sup(&other.min),
            max: self.max.inf(&other.max),
        };

        for i in 0..3 {
            if result.min[i] > result.max[i] {
                return None;
            }
        }

        Some(result)
    }
    pub fn size(&self) -> na::Vector3<f32> {
        self.max - self.min
    }
}

pub struct AabbPoint(na::Vector3<f32>);

impl Vec3 for AabbPoint {
    fn distance_heuristic(&self, other: &Self) -> f32 {
        (self.0 - other.0).norm_squared()
    }
}

impl BoundingVolume for Aabb {
    type Point = AabbPoint;
    fn contains(&self, other: &Self) -> bool {
        self.max >= other.max && self.min <= other.min
    }
    fn enlarge(&self, r: f32) -> Self {
        Self {
            min: self.min.add_scalar(-r),
            max: self.max.add_scalar(r),
        }
    }
    fn intersects(&self, other: &Self) -> bool {
        self.intersection(other).is_some()
    }
    fn surface_area_heuristic(&self) -> f32 {
        let size = self.size();
        size.x * size.y + size.x * size.z + size.y * size.z
    }
    fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.inf(&other.min),
            max: self.max.sup(&other.max),
        }
    }
    fn center(&self) -> Self::Point {
        AabbPoint(0.5 * (self.min + self.max))
    }
}
