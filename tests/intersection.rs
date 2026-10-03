#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;
#[path = "../common/sphere.rs"]
mod sphere;

use std::collections::HashSet;

use dbvh::{EnlargedBvh, IntersectSelf};
use nalgebra as na;
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;

use aabb::Aabb;
use rand_vec3::rand_vec3;

use crate::sphere::BoundingSphere;

#[test]
fn intersection_aabb() {
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

    for i in 0..1000 {
        let aabb = rand_aabb();
        aabbs.push(aabb);
        bvh.insert_leaf(aabb, i);
    }

    let mut intersection_bf: HashSet<usize> = HashSet::new();
    let mut intersection_bvh: HashSet<usize> = HashSet::new();
    let mut intersection_bvh_stack: HashSet<usize> = HashSet::new();

    let mut query_stack = Vec::new();

    for _ in 0..300 {
        let aabb = rand_aabb();

        aabbs.iter().enumerate().for_each(|(index, other)| {
            if aabb.intersects(other) {
                intersection_bf.insert(index);
            }
        });

        bvh.query_intersection(aabb, |index| {
            if aabb.intersects(&aabbs[*index]) {
                intersection_bvh.insert(*index);
            }
        });

        bvh.query_intersection_stack(&mut query_stack, aabb, |index| {
            if aabb.intersects(&aabbs[*index]) {
                intersection_bvh_stack.insert(*index);
            }
        });

        assert_eq!(intersection_bf, intersection_bvh_stack);
        assert_eq!(intersection_bf, intersection_bvh);

        intersection_bf.clear();
        intersection_bvh.clear();
        intersection_bvh_stack.clear();
    }
}

#[test]
fn intersection_sphere() {
    let min_pos = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_pos = na::Vector3::new(5.6, 9.7, 4.2);
    let max_size = 1.1;

    let mut rng = ChaCha8Rng::from_seed([12; _]);
    let mut list: Vec<BoundingSphere> = Vec::new();
    let mut bvh: EnlargedBvh<BoundingSphere, usize> = EnlargedBvh::new(0.01);

    for _ in 0..500 {
        let sphere = BoundingSphere::new(
            rand_vec3(&mut rng, min_pos, max_pos).into(),
            rng.random::<f32>() * max_size,
        );
        let index = list.len();
        list.push(sphere);
        bvh.insert_leaf(sphere, index);
    }

    let mut intersections_bvh = HashSet::new();
    let mut intersections_bf = HashSet::new();
    for _ in 0..1000 {
        let sphere = BoundingSphere::new(
            rand_vec3(&mut rng, min_pos, max_pos).into(),
            rng.random::<f32>() * max_size,
        );
        bvh.query_intersection(sphere, |index| {
            let other = list[*index];
            if sphere.intersects(&other) {
                intersections_bvh.insert(*index);
            }
        });
        list.iter().enumerate().for_each(|(index, other)| {
            if sphere.intersects(&other) {
                intersections_bf.insert(index);
            }
        });

        assert_eq!(intersections_bf, intersections_bvh);
        intersections_bf.clear();
        intersections_bvh.clear();
    }
}
