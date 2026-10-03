#[path = "../common/aabb.rs"]
mod aabb;

use criterion::{Criterion, criterion_group, criterion_main};
use dbvh::EnlargedBvh;

use nalgebra as na;

use crate::aabb::Aabb;

fn for_in_size(size: na::Vector3<usize>, mut f: impl FnMut(na::Vector3<usize>)) {
    for z in 0..size.z {
        for y in 0..size.y {
            for x in 0..size.x {
                f(na::Vector3::new(x, y, z));
            }
        }
    }
}

fn regular_cube(a: usize, c: &mut Criterion) {
    c.bench_function(&format!("spawn regular {}^3", a), |b| {
        b.iter(|| {
            let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);
            let mut i = 0;
            for_in_size(na::Vector3::repeat(a), |coord| {
                let pos = coord.cast::<f32>();
                let aabb = Aabb {
                    min: pos,
                    max: pos + na::Vector3::repeat(1.0),
                };
                bvh.insert_leaf(aabb, i);
                i += 1;
            });
        });
    });
}

fn regular(c: &mut Criterion) {
    regular_cube(3, c);
    regular_cube(10, c);
    regular_cube(15, c);
}

criterion_group!(benches, regular);
criterion_main!(benches);
