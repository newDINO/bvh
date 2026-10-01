use crate::{Bvh, EnlargedBvh, NodeIndex, NodeType, Vector};

/// A trait for types that can perform ray casting against a bounding volume.
pub trait RayCast {
    type Vector: Vector;

    /// * returns `Some(t)` if the ray intersects the volume, where `t`
    ///   is the entry parameter (or a lower bound of hit distances). Returns `None`
    ///   otherwise.
    /// * Only non-negative `t` values should be considered as valid hits.
    fn ray_cast(&self, origin: &Self::Vector, dir: &Self::Vector) -> Option<f32>;
}

impl<V: Vector, B: RayCast<Vector = V>, D> Bvh<B, D> {
    /// This method traverses the BVH and returns the smallest ray parameter `t`
    /// for which the leaf callback reports a hit.
    ///
    /// # Parameters
    ///
    /// * `origin` — The origin of the ray.
    /// * `dir` — The direction of the ray. **Note:** `dir` is not normalized
    ///   automatically.
    /// * `leaf_ray_cast_f` — A callback invoked for each leaf node that may
    ///   contain a hit. It receives `(origin, dir, leaf_data)` and must return
    ///   `Some(t)` for a valid hit with `t >= 0`, or `None` if the ray does not
    ///   hit the leaf's contents. The returned `t` must be in the same parameter
    ///   space as the values returned by `B::ray_cast`.
    ///
    pub fn ray_cast(
        &self,
        origin: &V,
        dir: &V,
        mut leaf_ray_cast_f: impl FnMut(&V, &V, &D) -> Option<f32>,
    ) -> Option<f32> {
        if self.root_index == NodeIndex::NULL {
            return None;
        }
        let root = &self.nodes[self.root_index.0];

        if root.bounding_volume.ray_cast(origin, dir).is_some() {
            self.ray_cast_rec(self.root_index, origin, dir, &mut leaf_ray_cast_f)
        } else {
            None
        }
    }

    fn ray_cast_rec(
        &self,
        index: NodeIndex,
        origin: &V,
        dir: &V,
        leaf_ray_cast_f: &mut impl FnMut(&V, &V, &D) -> Option<f32>,
    ) -> Option<f32> {
        let node = &self.nodes[index.0];

        match &node.ty {
            NodeType::Leaf(data) => leaf_ray_cast_f(origin, dir, data),
            NodeType::Internal { child1, child2 } => {
                let node1 = &self.nodes[child1.0];
                let cast1 = node1.bounding_volume.ray_cast(origin, dir);

                let node2 = &self.nodes[child2.0];
                let cast2 = node2.bounding_volume.ray_cast(origin, dir);

                match (cast1, cast2) {
                    (None, None) => None,
                    (Some(_), None) => self.ray_cast_rec(*child1, origin, dir, leaf_ray_cast_f),
                    (None, Some(_)) => self.ray_cast_rec(*child2, origin, dir, leaf_ray_cast_f),
                    (Some(r1), Some(r2)) => {
                        let (ia, ib, rb) = if r1 <= r2 {
                            (*child1, *child2, r2)
                        } else {
                            (*child2, *child1, r1)
                        };

                        let cast_a = self.ray_cast_rec(ia, origin, dir, leaf_ray_cast_f);
                        if let Some(ra_inner) = cast_a {
                            let r = if ra_inner <= rb {
                                // ra_inner is smaller than or equal to the smallest distance of ray casting on another branch.
                                ra_inner
                            } else {
                                let cast_b = self.ray_cast_rec(ib, origin, dir, leaf_ray_cast_f);
                                if let Some(rb_inner) = cast_b {
                                    if ra_inner <= rb_inner {
                                        ra_inner
                                    } else {
                                        rb_inner
                                    }
                                } else {
                                    ra_inner
                                }
                            };
                            Some(r)
                        } else {
                            self.ray_cast_rec(ib, origin, dir, leaf_ray_cast_f)
                        }
                    }
                }
            }
        }
    }
}

impl<V: Vector, B: RayCast<Vector = V>, D> EnlargedBvh<B, D> {
    pub fn ray_cast(
        &self,
        origin: &V,
        dir: &V,
        leaf_ray_cast_f: impl FnMut(&V, &V, &D) -> Option<f32>,
    ) -> Option<f32> {
        self.bvh.ray_cast(origin, dir, leaf_ray_cast_f)
    }
}
