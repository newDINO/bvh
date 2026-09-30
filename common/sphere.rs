#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct Sphere(glam::Vec4);

impl Sphere {
    pub fn new(pos: glam::Vec3, r: f32) -> Self {
        Self(pos.extend(r))
    }
    #[inline(always)]
    pub fn intersects(self, mut other: Sphere) -> bool {
        // (x2, y2, z2, -r2)
        other.0.w = -other.0.w;

        // (dx, dy, dz, r1 + r2)s
        let diff = self.0 - other.0;

        // (dx^2, dy^2, dz^2, r^2)
        let mut product = diff * diff;
        // (dx^2, dy^2, dz^2, -r^2)
        product.w = -product.w;

        // dx^2 + dy^2 + dz^2 - r^2
        let sum = product.element_sum();
        sum <= 0.0
    }
}

// #[derive(Clone, Copy)]
// #[repr(align(16))]
// pub struct Sphere {
//     pos: glam::Vec3,
//     r: f32,
// }

// impl Sphere {
//     pub fn new(pos: glam::Vec3, r: f32) -> Self {
//         Self { pos, r }
//     }
//     #[inline]
//     pub fn intersects(self, other: Sphere) -> bool {
//         let dist2 = (self.pos - other.pos).length_squared();

//         let max_dist = self.r + other.r;
//         let max_dist2 = max_dist * max_dist;

//         dist2 <= max_dist2
//     }
// }
