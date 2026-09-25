use bvh::BoundingVolume;
use nalgebra as na;

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: na::Vector3<f32>,
    pub max: na::Vector3<f32>,
}

impl Aabb {
    fn intersection(&self, other: &Aabb) -> Option<Aabb> {
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
    fn size(&self) -> na::Vector3<f32> {
        self.max - self.min
    }
}

impl BoundingVolume for Aabb {
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
}

#[cfg(test)]
mod tests {
    use super::Aabb;
    use bvh::BoundingVolume;
    use nalgebra as na;

    fn v(x: f32, y: f32, z: f32) -> na::Vector3<f32> {
        na::Vector3::new(x, y, z)
    }

    fn aabb(min: (f32, f32, f32), max: (f32, f32, f32)) -> Aabb {
        Aabb {
            min: v(min.0, min.1, min.2),
            max: v(max.0, max.1, max.2),
        }
    }

    fn assert_aabb_eq(actual: &Aabb, min: (f32, f32, f32), max: (f32, f32, f32)) {
        assert_eq!(actual.min, v(min.0, min.1, min.2));
        assert_eq!(actual.max, v(max.0, max.1, max.2));
    }

    fn assert_option_aabb_eq(actual: Option<Aabb>, min: (f32, f32, f32), max: (f32, f32, f32)) {
        let actual = actual.unwrap();
        assert_aabb_eq(&actual, min, max);
    }

    #[test]
    fn size_returns_max_minus_min() {
        let a = aabb((1.0, 2.0, 3.0), (4.0, 6.0, 8.0));
        assert_eq!(a.size(), v(3.0, 4.0, 5.0));
    }

    #[test]
    fn size_zero_for_point() {
        let a = aabb((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert_eq!(a.size(), v(0.0, 0.0, 0.0));
    }

    #[test]
    fn size_can_be_negative_for_invalid_aabb() {
        let invalid = aabb((5.0, 5.0, 5.0), (1.0, 2.0, 3.0));
        assert_eq!(invalid.size(), v(-4.0, -3.0, -2.0));
    }

    #[test]
    fn intersection_overlap_returns_intersection() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));
        let b = aabb((1.0, 1.0, 1.0), (3.0, 3.0, 3.0));

        assert_option_aabb_eq(a.intersection(&b), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0));
    }

    #[test]
    fn intersection_touching_face_is_some() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((1.0, 0.0, 0.0), (2.0, 1.0, 1.0));

        assert_option_aabb_eq(a.intersection(&b), (1.0, 0.0, 0.0), (1.0, 1.0, 1.0));
    }

    #[test]
    fn intersection_touching_edge_is_some() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((1.0, 1.0, 0.0), (2.0, 2.0, 1.0));

        assert_option_aabb_eq(a.intersection(&b), (1.0, 1.0, 0.0), (1.0, 1.0, 1.0));
    }

    #[test]
    fn intersection_touching_point_is_some() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((1.0, 1.0, 1.0), (2.0, 2.0, 2.0));

        assert_option_aabb_eq(a.intersection(&b), (1.0, 1.0, 1.0), (1.0, 1.0, 1.0));
    }

    #[test]
    fn intersection_disjoint_is_none() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((2.0, 0.0, 0.0), (3.0, 1.0, 1.0));

        assert!(a.intersection(&b).is_none());
    }

    #[test]
    fn intersection_contained_returns_inner() {
        let outer = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
        let inner = aabb((2.0, 2.0, 2.0), (3.0, 3.0, 3.0));

        assert_option_aabb_eq(outer.intersection(&inner), (2.0, 2.0, 2.0), (3.0, 3.0, 3.0));
    }

    #[test]
    fn intersection_identical_returns_same() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));

        assert_option_aabb_eq(a.intersection(&b), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
    }

    #[test]
    fn intersection_invalid_aabb_can_return_none() {
        let invalid = aabb((2.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let valid = aabb((0.0, 0.0, 0.0), (3.0, 3.0, 3.0));

        assert!(invalid.intersection(&valid).is_none());
    }

    #[test]
    fn intersects_matches_intersection() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));

        let overlapping = aabb((1.0, 1.0, 1.0), (3.0, 3.0, 3.0));
        assert!(a.intersects(&overlapping));
        assert!(a.intersection(&overlapping).is_some());

        let touching = aabb((2.0, 0.0, 0.0), (3.0, 1.0, 1.0));
        assert!(a.intersects(&touching));
        assert!(a.intersection(&touching).is_some());

        let disjoint = aabb((5.0, 5.0, 5.0), (6.0, 6.0, 6.0));
        assert!(!a.intersects(&disjoint));
        assert!(a.intersection(&disjoint).is_none());
    }

    #[test]
    fn contains_strictly_inside() {
        let outer = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
        let inner = aabb((1.0, 1.0, 1.0), (9.0, 9.0, 9.0));

        assert!(outer.contains(&inner));
    }

    #[test]
    fn contains_equal_boundary_should_be_true() {
        let outer = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
        let inner = aabb((0.0, 1.0, 1.0), (9.0, 9.0, 9.0));

        assert!(outer.contains(&inner));
    }

    #[test]
    fn contains_same_should_be_true() {
        let a = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));

        assert!(a.contains(&a));
    }

    #[test]
    fn contains_disjoint_is_false() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((2.0, 2.0, 2.0), (3.0, 3.0, 3.0));

        assert!(!a.contains(&b));
    }

    #[test]
    fn contains_partial_overlap_is_false() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));
        let b = aabb((1.0, 1.0, 1.0), (3.0, 3.0, 3.0));

        assert!(!a.contains(&b));
    }

    #[test]
    fn contains_other_larger_is_false() {
        let a = aabb((1.0, 1.0, 1.0), (2.0, 2.0, 2.0));
        let b = aabb((0.0, 0.0, 0.0), (3.0, 3.0, 3.0));

        assert!(!a.contains(&b));
    }

    #[test]
    fn enlarge_positive_expands() {
        let a = aabb((1.0, 2.0, 3.0), (4.0, 5.0, 6.0));
        let enlarged = a.enlarge(2.0);

        assert_aabb_eq(&enlarged, (-1.0, 0.0, 1.0), (6.0, 7.0, 8.0));
    }

    #[test]
    fn enlarge_zero_noop() {
        let a = aabb((1.0, 2.0, 3.0), (4.0, 5.0, 6.0));
        let enlarged = a.enlarge(0.0);

        assert_aabb_eq(&enlarged, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0));
    }

    #[test]
    fn enlarge_negative_shrinks() {
        let a = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
        let shrunk = a.enlarge(-1.0);

        assert_aabb_eq(&shrunk, (1.0, 1.0, 1.0), (9.0, 9.0, 9.0));
    }

    #[test]
    fn surface_area_heuristic_unit_cube() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));

        assert_eq!(a.surface_area_heuristic(), 3.0);
    }

    #[test]
    fn surface_area_heuristic_non_cube() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 3.0, 4.0));

        assert_eq!(a.surface_area_heuristic(), 26.0);
    }

    #[test]
    fn surface_area_heuristic_zero_volume() {
        let a = aabb((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));

        assert_eq!(a.surface_area_heuristic(), 0.0);
    }

    #[test]
    fn surface_area_heuristic_flat() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 3.0, 0.0));

        assert_eq!(a.surface_area_heuristic(), 6.0);
    }

    #[test]
    fn union_disjoint() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((2.0, 3.0, 4.0), (3.0, 4.0, 5.0));

        let u = a.union(&b);
        assert_aabb_eq(&u, (0.0, 0.0, 0.0), (3.0, 4.0, 5.0));
    }

    #[test]
    fn union_overlap() {
        let a = aabb((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));
        let b = aabb((1.0, 1.0, 1.0), (3.0, 3.0, 3.0));

        let u = a.union(&b);
        assert_aabb_eq(&u, (0.0, 0.0, 0.0), (3.0, 3.0, 3.0));
    }

    #[test]
    fn union_contained_returns_outer() {
        let outer = aabb((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
        let inner = aabb((2.0, 2.0, 2.0), (3.0, 3.0, 3.0));

        let u = outer.union(&inner);
        assert_aabb_eq(&u, (0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
    }

    #[test]
    fn union_identical_returns_same() {
        let a = aabb((1.0, 2.0, 3.0), (4.0, 5.0, 6.0));
        let b = aabb((1.0, 2.0, 3.0), (4.0, 5.0, 6.0));

        let u = a.union(&b);
        assert_aabb_eq(&u, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0));
    }

    #[test]
    fn union_touching() {
        let a = aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let b = aabb((1.0, 1.0, 1.0), (2.0, 2.0, 2.0));

        let u = a.union(&b);
        assert_aabb_eq(&u, (0.0, 0.0, 0.0), (2.0, 2.0, 2.0));
    }
}
