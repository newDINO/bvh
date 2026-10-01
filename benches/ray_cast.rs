#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;
use criterion::{Criterion, criterion_group, criterion_main};

use aabb::{Aabb, AabbVector};
use bvh::{EnlargedBvh, RayCast};
use rand_vec3::rand_vec3;

use nalgebra as na;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

fn ray_cast(c: &mut Criterion) {
    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(10.6, 9.7, 7.2);

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

    let mut rand_rays: Vec<(na::Vector3<f32>, na::Vector3<f32>)> = Vec::new();
    const N_RAYS: usize = 128;
    while rand_rays.len() < N_RAYS {
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
        let dir = dir * (1.0 / l);

        rand_rays.push((origin, dir));
    }

    c.bench_function("ray_cast bvh", |b| {
        let mut i = 0;
        b.iter(|| {
            let (origin, dir) = rand_rays[i];

            let result = bvh.ray_cast(
                &AabbVector(origin),
                &AabbVector(dir),
                |origin, dir, index| {
                    let aabb = aabbs[*index];
                    aabb.ray_cast(origin, dir).map(|t| (t, ()))
                },
            );
            core::hint::black_box(result);

            i = (i + 1) % N_RAYS;
        });
    });

    c.bench_function("ray_cast bf", |b| {
        let mut i = 0;
        b.iter(|| {
            let (origin, dir) = rand_rays[i];

            let bf_result = aabbs.iter().fold(None, |acc, aabb| {
                let result = aabb.ray_cast(&AabbVector(origin), &AabbVector(dir));
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
            core::hint::black_box(bf_result);

            i = (i + 1) % N_RAYS;
        });
    });
}

criterion_group!(benches, ray_cast);
criterion_main!(benches);
