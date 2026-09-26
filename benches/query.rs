#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;

use std::hint::black_box;

use bvh::{BoundingVolume, EnlargedBvh};
use criterion::{Criterion, criterion_group, criterion_main};
use nalgebra as na;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use aabb::Aabb;
use rand_vec3::rand_vec3;

fn for_in_size(size: na::Vector3<usize>, mut f: impl FnMut(na::Vector3<usize>)) {
    for z in 0..size.z {
        for y in 0..size.y {
            for x in 0..size.x {
                f(na::Vector3::new(x, y, z));
            }
        }
    }
}

fn regular_cube(c: &mut Criterion, a: usize) {
    let aabb_size = na::Vector3::new(1.0, 1.0, 1.0);

    let range = na::Vector3::repeat(a);

    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);
    let mut list: Vec<Aabb> = Vec::new();

    for_in_size(range, |coord| {
        let pos = coord.cast::<f32>();
        let aabb = Aabb {
            min: pos,
            max: pos + aabb_size,
        };
        bvh.insert_leaf(aabb, list.len());
        list.push(aabb);
    });

    let mut rng = ChaCha8Rng::from_seed([123; _]);
    let samples: Vec<Aabb> = (0..256)
        .map(|_| {
            let point = rand_vec3(
                &mut rng,
                na::Vector3::zeros(),
                na::Vector3::repeat(a as f32),
            );
            Aabb {
                min: point,
                max: point + aabb_size,
            }
        })
        .collect();

    c.bench_function(&format!("regular {}^3 bvh", a), |b| {
        let mut i = 0;

        let mut query_stack = Vec::new();

        b.iter(|| {
            i = (i + 1) % samples.len();
            let aabb = samples[i];
            bvh.query_intersection_stack(&mut query_stack, aabb, |id| {
                if aabb.intersects(&list[id]) {
                    black_box(id);
                }
            });
        })
    });

    // c.bench_function(&format!("regular {}^3 bf", a), |b| {
    //     let mut i = 0;
    //     b.iter(|| {
    //         i = (i + 1) % samples.len();
    //         let aabb = samples[i];
    //         list.iter().enumerate().for_each(|(id, other)| {
    //             if aabb.intersects(other) {
    //                 black_box(id);
    //             }
    //         });
    //     })
    // });
}

fn regular_arrange(c: &mut Criterion) {
    regular_cube(c, 10);
    regular_cube(c, 100);
}

criterion_group!(benches, regular_arrange);
criterion_main!(benches);
