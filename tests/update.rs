#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;

use std::collections::HashSet;

use bvh::{BoundingVolume, EnlargedBvh, NodeIndex};
use nalgebra as na;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;

use aabb::Aabb;
use rand_vec3::rand_vec3;

#[test]
fn update() {
    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);

    let mut aabbs: Vec<(Aabb, NodeIndex)> = Vec::new();

    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(10.1, 9.7, 4.2);

    let max_size = na::Vector3::new(2.0, 2.0, 2.0);

    let mut rng = ChaCha8Rng::from_seed([111; 32]);

    for i in 0..1000 {
        let aabb = {
            let min = rand_vec3(&mut rng, min_start, max_start);
            let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
            Aabb { min, max }
        };
        let node_index = bvh.insert_leaf(aabb, i);
        aabbs.push((aabb, node_index));
    }

    let mut intersection_bf: HashSet<usize> = HashSet::new();
    let mut intersection_bvh: HashSet<usize> = HashSet::new();

    for _ in 0..3 {
        aabbs.iter_mut().for_each(|(aabb, node_index)| {
            let v = rand_vec3(
                &mut rng,
                na::Vector3::repeat(-0.1),
                na::Vector3::repeat(0.1),
            );
            aabb.min += v;
            aabb.max += v;

            bvh.update_leaf(*node_index, *aabb);
        });

        for _ in 0..100 {
            let aabb = {
                let min = rand_vec3(&mut rng, min_start, max_start);
                let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
                Aabb { min, max }
            };
            aabbs.iter().enumerate().for_each(|(index, (other, _))| {
                if aabb.intersects(other) {
                    intersection_bf.insert(index);
                }
            });

            bvh.query_intersection(aabb, |index| {
                if aabb.intersects(&aabbs[index].0) {
                    intersection_bvh.insert(index);
                }
            });
            assert_eq!(intersection_bf, intersection_bvh);
            intersection_bf.clear();
            intersection_bvh.clear();
        }
    }
}
