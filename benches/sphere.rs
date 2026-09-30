#[path = "../common/sphere.rs"]
mod sphere;
use sphere::Sphere;

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

fn criterion_benchmark(c: &mut Criterion) {
    let s1 = Sphere::new(glam::vec3(0.0, 0.0, 0.0), 1.0);
    let s2 = Sphere::new(glam::vec3(0.5, 0.0, 0.0), 1.0);
    c.bench_function("sphere intersects", |b| {
        b.iter(|| {
            let result = Sphere::intersects(black_box(s1), black_box(s2));
            black_box(result);
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
