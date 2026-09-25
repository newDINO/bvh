mod common;

use std::collections::HashSet;

use bvh::{BoundingVolume, EnlargedBvh};
use common::Aabb;
use nalgebra as na;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;

fn rand_vec3(rng: &mut impl Rng, min: na::Vector3<f32>, max: na::Vector3<f32>) -> na::Vector3<f32> {
    na::Vector3::new(
        rng.random_range::<f32, _>(min.x..max.x),
        rng.random_range::<f32, _>(min.y..max.y),
        rng.random_range::<f32, _>(min.z..max.z),
    )
}

#[test]
fn test() {
    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(5.6, 9.7, 4.2);

    let max_size = na::Vector3::new(3.0, 3.0, 3.0);

    let mut rng = ChaCha8Rng::from_seed([123; 32]);

    let mut rand_aabb = || -> Aabb {
        let min = rand_vec3(&mut rng, min_start, max_start);
        let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
        Aabb { min, max }
    };

    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);

    let mut aabbs: Vec<Aabb> = Vec::new();

    for i in 0..300 {
        let aabb = rand_aabb();
        aabbs.push(aabb);
        bvh.insert_leaf(aabb, i);
    }

    let mut intersection_bf: HashSet<usize> = HashSet::new();
    let mut intersection_bvh: HashSet<usize> = HashSet::new();
    for _ in 0..100 {
        let aabb = rand_aabb();

        aabbs.iter().enumerate().for_each(|(index, other)| {
            if aabb.intersects(other) {
                intersection_bf.insert(index);
            }
        });

        bvh.find_intersecting_leafs(aabb, |index| {
            if aabb.intersects(&aabbs[index]) {
                intersection_bvh.insert(index);
            }
        });

        assert_eq!(intersection_bf, intersection_bvh);

        intersection_bf.clear();
        intersection_bvh.clear();
    }
}
