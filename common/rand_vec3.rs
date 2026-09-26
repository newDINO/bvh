use nalgebra as na;
use rand::{Rng, RngExt};

pub fn rand_vec3(
    rng: &mut impl Rng,
    min: na::Vector3<f32>,
    max: na::Vector3<f32>,
) -> na::Vector3<f32> {
    na::Vector3::new(
        rng.random_range::<f32, _>(min.x..max.x),
        rng.random_range::<f32, _>(min.y..max.y),
        rng.random_range::<f32, _>(min.z..max.z),
    )
}
