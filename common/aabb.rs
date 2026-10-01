use bvh::{BoundingVolume, RayCast, Vector};
use nalgebra as na;

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: na::Vector3<f32>,
    pub max: na::Vector3<f32>,
}

impl Aabb {
    #[inline]
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

    #[inline]
    pub fn size(&self) -> na::Vector3<f32> {
        self.max - self.min
    }
}

pub struct AabbVector(na::Vector3<f32>);

impl Vector for AabbVector {
    #[inline]
    fn distance_heuristic(&self, other: &Self) -> f32 {
        (self.0 - other.0).norm_squared()
    }
}

impl BoundingVolume for Aabb {
    type Point = AabbVector;

    #[inline]
    fn contains(&self, other: &Self) -> bool {
        self.max >= other.max && self.min <= other.min
    }

    #[inline]
    fn enlarge(&self, r: f32) -> Self {
        Self {
            min: self.min.add_scalar(-r),
            max: self.max.add_scalar(r),
        }
    }

    #[inline]
    fn intersects(&self, other: &Self) -> bool {
        self.intersection(other).is_some()
    }

    #[inline]
    fn surface_area_heuristic(&self) -> f32 {
        let size = self.size();
        size.x * size.y + size.x * size.z + size.y * size.z
    }

    #[inline]
    fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.inf(&other.min),
            max: self.max.sup(&other.max),
        }
    }

    #[inline]
    fn center(&self) -> Self::Point {
        AabbVector(0.5 * (self.min + self.max))
    }
}

impl RayCast for Aabb {
    type Vector = AabbVector;
    fn ray_cast(&self, origin: &Self::Vector, dir: &Self::Vector) -> Option<f32> {
        let dir = dir.0;
        let origin = origin.0;

        let mut t_min = 0.0f32;
        let mut t_max = f32::MAX;

        for i in 0..3 {
            if dir[i].abs() < f32::EPSILON {
                if origin[i] > self.max[i] || origin[i] < self.min[i] {
                    return None;
                }
            } else {
                let inv = 1.0 / dir[i];
                let t_near = inv * (self.min[i] - origin[i]);
                let t_far = inv * (self.max[i] - origin[i]);

                let (t_near, t_far) = if t_near > t_far {
                    (t_far, t_near)
                } else {
                    (t_near, t_far)
                };

                t_min = t_min.max(t_near);
                t_max = t_max.min(t_far);

                if t_min > t_max {
                    return None;
                }
            }
        }

        Some(t_min)
    }
}
