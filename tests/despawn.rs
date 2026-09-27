#[path = "../common/aabb.rs"]
mod aabb;
#[path = "../common/rand_vec3.rs"]
mod rand_vec3;
use rand_vec3::rand_vec3;

use std::collections::{HashMap, HashSet};

use aabb::Aabb;
use bvh::{BoundingVolume, EnlargedBvh, NodeIndex};

use nalgebra as na;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

#[test]
fn fuzz() {
    let mut id: usize = 0;

    let mut aabbs: HashMap<usize, (Aabb, NodeIndex)> = HashMap::new();
    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);

    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(5.6, 9.7, 4.2);

    let max_size = na::Vector3::new(3.0, 3.0, 3.0);

    let mut rng = ChaCha8Rng::from_seed([123; 32]);

    let mut intersection_bf: HashSet<usize> = HashSet::new();
    let mut intersection_bvh: HashSet<usize> = HashSet::new();

    for _ in 0..3 {
        // spawn new object
        for _ in 0..500 {
            let aabb = {
                let min = rand_vec3(&mut rng, min_start, max_start);
                let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
                Aabb { min, max }
            };
            let node_index = bvh.insert_leaf(aabb, id);
            aabbs.insert(id, (aabb, node_index));
            id += 1;
        }

        // remove some of them
        let mut n_removed = 0;
        for _ in 0..500 {
            let to_remove = rng.random_range(0..aabbs.len());
            if let Some((_, node_index)) = aabbs.remove(&to_remove) {
                bvh.remove_leaf(node_index);
                n_removed += 1;
            }
        }
        assert!(n_removed > 0);

        // test the query result with brute force result
        for _ in 0..100 {
            let aabb = {
                let min = rand_vec3(&mut rng, min_start, max_start);
                let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
                Aabb { min, max }
            };
            aabbs.iter().for_each(|(index, (other, _))| {
                if aabb.intersects(other) {
                    intersection_bf.insert(*index);
                }
            });

            bvh.query_intersection(aabb, |index| {
                if aabb.intersects(&aabbs[&index].0) {
                    intersection_bvh.insert(index);
                }
            });
        }
    }
}

#[test]
fn remove_all() {
    let mut id: usize = 0;

    let mut aabbs: HashMap<usize, (Aabb, NodeIndex)> = HashMap::new();
    let mut bvh: EnlargedBvh<Aabb, usize> = EnlargedBvh::new(0.1);

    let min_start = na::Vector3::new(-11.0, -11.3, -9.8);
    let max_start = na::Vector3::new(5.6, 9.7, 4.2);

    let max_size = na::Vector3::new(3.0, 3.0, 3.0);

    let mut rng = ChaCha8Rng::from_seed([123; 32]);

    // spawn some
    for _ in 0..100 {
        let aabb = {
            let min = rand_vec3(&mut rng, min_start, max_start);
            let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
            Aabb { min, max }
        };
        let node_index = bvh.insert_leaf(aabb, id);
        aabbs.insert(id, (aabb, node_index));
        id += 1;
    }

    // remove all
    for i in 0..id {
        let (_, node_index) = aabbs.remove(&i).unwrap();
        bvh.remove_leaf(node_index);
    }
    assert!(aabbs.is_empty());

    // spawn some
    for _ in 0..100 {
        let aabb = {
            let min = rand_vec3(&mut rng, min_start, max_start);
            let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
            Aabb { min, max }
        };
        let node_index = bvh.insert_leaf(aabb, id);
        aabbs.insert(id, (aabb, node_index));
        id += 1;
    }

    // test the query result with brute force result
    let mut intersection_bf: HashSet<usize> = HashSet::new();
    let mut intersection_bvh: HashSet<usize> = HashSet::new();
    for _ in 0..100 {
        let aabb = {
            let min = rand_vec3(&mut rng, min_start, max_start);
            let max = min + rand_vec3(&mut rng, na::Vector3::zeros(), max_size);
            Aabb { min, max }
        };
        aabbs.iter().for_each(|(index, (other, _))| {
            if aabb.intersects(other) {
                intersection_bf.insert(*index);
            }
        });

        bvh.query_intersection(aabb, |index| {
            if aabb.intersects(&aabbs[&index].0) {
                intersection_bvh.insert(index);
            }
        });
    }
}
