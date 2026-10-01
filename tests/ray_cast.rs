#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;

use aabb::Aabb;
use bvh::{EnlargedBvh, RayCast};
use rand_vec3::rand_vec3;

use nalgebra as na;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::aabb::AabbVector;

#[test]
fn ray_cast_fuzz() {
    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(5.6, 9.7, 4.2);

    let max_size = na::Vector3::new(1.0, 1.0, 1.0);

    let mut rng = ChaCha8Rng::from_seed([123; 32]);

    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);

    let mut aabbs: Vec<Aabb> = Vec::new();

    for i in 0..1000 {
        let aabb = {
            let min = rand_vec3(&mut rng, min_start, max_start);
            let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
            Aabb { min, max }
        };
        aabbs.push(aabb);
        bvh.insert_leaf(aabb, i);
    }

    let mut tested_ray = 0;
    let n_tests = 1000;
    for _ in 0..n_tests {
        let origin = rand_vec3(&mut rng, min_start, max_start);

        let dir = rand_vec3(
            &mut rng,
            na::Vector3::repeat(-1.0),
            na::Vector3::repeat(1.0),
        );

        let l = dir.norm();
        if l < f32::EPSILON {
            continue;
        }
        tested_ray += 1;

        let dir = dir * (1.0 / l);

        let origin = AabbVector(origin);
        let dir = AabbVector(dir);

        let bf_result = aabbs.iter().fold(None, |acc, aabb| {
            let result = aabb.ray_cast(&origin, &dir);
            if let Some(result) = result {
                let t = if let Some(acc) = acc {
                    result.min(acc)
                } else {
                    result
                };
                Some(t)
            } else {
                acc
            }
        });

        let bvh_result = bvh.ray_cast(&origin, &dir, |origin, dir, index| {
            let aabb = aabbs[*index];
            aabb.ray_cast(origin, dir)
        });

        assert_eq!(bf_result, bvh_result);
    }

    assert!(tested_ray as f32 > n_tests as f32 * 0.9);
}
