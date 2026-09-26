mod slot_pool;

use slot_pool::{SlotPool, SlotPoolHandle};
use std::fmt::Debug;

pub trait BoundingVolume {
    type Point: Vec3;
    fn intersects(&self, other: &Self) -> bool;
    fn contains(&self, other: &Self) -> bool;
    fn union(&self, other: &Self) -> Self;
    fn enlarge(&self, r: f32) -> Self;
    fn surface_area_heuristic(&self) -> f32;
    fn center(&self) -> Self::Point;
}

pub trait Vec3 {
    fn distance_heuristic(&self, other: &Self) -> f32;
}

#[derive(Clone, Copy, Debug)]
enum NodeType<D> {
    Internal {
        child1: NodeIndex,
        child2: NodeIndex,
    },
    Leaf(D),
}
impl<D> NodeType<D> {
    fn is_leaf(&self) -> bool {
        match self {
            Self::Internal { .. } => false,
            Self::Leaf(_) => true,
        }
    }
    fn is_internal(&self) -> bool {
        match self {
            Self::Internal { .. } => true,
            Self::Leaf(_) => false,
        }
    }
    fn as_internal(&self) -> (NodeIndex, NodeIndex) {
        match self {
            Self::Internal { child1, child2 } => (*child1, *child2),
            _ => unreachable!(),
        }
    }
    fn as_internal_mut(&mut self) -> (&mut NodeIndex, &mut NodeIndex) {
        match self {
            Self::Internal { child1, child2 } => (child1, child2),
            _ => unreachable!(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeIndex(SlotPoolHandle);

impl NodeIndex {
    const NULL: Self = Self(SlotPoolHandle::NULL);
}

#[derive(Clone, Debug)]
pub struct Node<B, D> {
    ty: NodeType<D>,
    parent_index: NodeIndex,
    bounding_volume: B,
}

#[derive(Debug)]
pub struct Bvh<B, D> {
    root_index: NodeIndex,
    nodes: SlotPool<Node<B, D>>,
}

impl<B: BoundingVolume + Copy + Debug, D: Copy + Debug> Bvh<B, D> {
    pub fn new() -> Self {
        Self {
            root_index: NodeIndex::NULL,
            nodes: SlotPool::new(),
        }
    }

    #[inline]
    pub fn query_intersection_stack(&self, stack: &mut Vec<NodeIndex>, q: B, mut f: impl FnMut(D)) {
        if self.root_index == NodeIndex::NULL {
            return;
        }

        let root = unsafe { self.nodes.get_unchecked(self.root_index.0) };
        if q.intersects(&root.bounding_volume) {
            stack.push(self.root_index);
        }

        while let Some(index) = stack.pop() {
            let node = unsafe { self.nodes.get_unchecked(index.0) };

            match node.ty {
                NodeType::Internal { child1, child2 } => {
                    let c1 = unsafe { self.nodes.get_unchecked(child1.0) };
                    if q.intersects(&c1.bounding_volume) {
                        stack.push(child1);
                    }

                    let c2 = unsafe { self.nodes.get_unchecked(child2.0) };
                    if q.intersects(&c2.bounding_volume) {
                        stack.push(child2);
                    }
                }
                NodeType::Leaf(entity) => f(entity),
            }
        }
    }

    #[inline]
    pub fn query_intersection(&self, q: B, mut f: impl FnMut(D)) {
        if self.root_index == NodeIndex::NULL {
            return;
        }

        #[cfg(debug_assertions)]
        let root = &self.nodes[self.root_index.0];

        #[cfg(not(debug_assertions))]
        let root = unsafe { self.nodes.get_unchecked(self.root_index.0) };

        if q.intersects(&root.bounding_volume) {
            self.query_intersection_rec(root, &q, &mut f);
        }
    }

    fn query_intersection_rec(&self, node: &Node<B, D>, q: &B, f: &mut impl FnMut(D)) {
        match node.ty {
            NodeType::Internal { child1, child2 } => {
                #[cfg(debug_assertions)]
                let c1 = &self.nodes[child1.0];
                #[cfg(not(debug_assertions))]
                let c1 = unsafe { self.nodes.get_unchecked(child1.0) };

                if q.intersects(&c1.bounding_volume) {
                    self.query_intersection_rec(c1, q, f);
                }

                #[cfg(debug_assertions)]
                let c2 = &self.nodes[child2.0];
                #[cfg(not(debug_assertions))]
                let c2 = unsafe { self.nodes.get_unchecked(child2.0) };

                if q.intersects(&c2.bounding_volume) {
                    self.query_intersection_rec(c2, q, f);
                }
            }
            NodeType::Leaf(entity) => f(entity),
        }
    }

    // pub fn nodes(&self) -> &SlotPool<Node> {
    //     &self.nodes
    // }

    pub fn update_leaf(&mut self, index: NodeIndex, bounding_volume: B) {
        let leaf = &self.nodes[index.0];
        let NodeType::Leaf(entity) = leaf.ty else {
            unreachable!()
        };
        self.remove_leaf(index);
        self.insert_leaf_at(bounding_volume, entity, index);
    }

    pub fn insert_leaf(&mut self, bounding_volume: B, entity: D) -> NodeIndex {
        if self.root_index == NodeIndex::NULL {
            core::hint::cold_path();

            let node = Node {
                ty: NodeType::Leaf(entity),
                parent_index: NodeIndex::NULL,
                bounding_volume: bounding_volume,
            };
            let index = NodeIndex(self.nodes.insert(node));
            self.root_index = index;

            return index;
        }

        let new_leaf_index = NodeIndex(self.nodes.allocate_slot());

        self.insert_leaf_at(bounding_volume, entity, new_leaf_index);

        new_leaf_index
    }

    fn insert_leaf_at(&mut self, bounding_volume: B, entity: D, leaf_index: NodeIndex) {
        debug_assert_ne!(self.root_index, NodeIndex::NULL);

        // Stage 1: find the best sibling for the new leaf
        let best_sibling = self.find_best_sibling(bounding_volume);

        // Stage 2: create a new parent
        let new_parent_index = NodeIndex(self.nodes.allocate_slot());

        let sibling = &mut self.nodes[best_sibling.0];

        let old_parent_index = sibling.parent_index;
        sibling.parent_index = new_parent_index;

        let new_parent = Node {
            ty: NodeType::Internal {
                child1: best_sibling,
                child2: leaf_index,
            },
            parent_index: old_parent_index,
            bounding_volume: bounding_volume.union(&sibling.bounding_volume),
        };
        let new_leaf = Node {
            ty: NodeType::Leaf(entity),
            parent_index: new_parent_index,
            bounding_volume: bounding_volume,
        };

        if old_parent_index == NodeIndex::NULL {
            core::hint::cold_path();
            self.root_index = new_parent_index;
        } else {
            let old_parent = self.nodes.get_mut(old_parent_index.0).unwrap();

            let (child1, child2) = old_parent.ty.as_internal_mut();

            if *child1 == best_sibling {
                *child1 = new_parent_index;
            } else if *child2 == best_sibling {
                *child2 = new_parent_index;
            } else {
                unreachable!();
            }
        }

        self.nodes
            .insert_at(new_parent, new_parent_index.0)
            .unwrap();
        self.nodes.insert_at(new_leaf, leaf_index.0).unwrap();

        // Stage 3: walk back up the tree refitting AABBs
        let mut index = self.nodes.get(leaf_index.0).unwrap().parent_index;
        while index != NodeIndex::NULL {
            let node = &self.nodes.get(index.0).unwrap();
            let parent_index = node.parent_index;

            let (child1, child2) = node.ty.as_internal();
            let aabb1 = self.nodes.get(child1.0).unwrap().bounding_volume;
            let aabb2 = self.nodes.get(child2.0).unwrap().bounding_volume;

            self.nodes.get_mut(index.0).unwrap().bounding_volume = aabb1.union(&aabb2);

            self.rotate_to_balance(index);

            index = parent_index;
        }
    }

    fn find_best_sibling(&self, bounding_volume: B) -> NodeIndex {
        let center = bounding_volume.center();

        let area = bounding_volume.surface_area_heuristic();

        let mut inherited_cost = 0.0;

        let mut best_sibling = self.root_index();
        let mut best_cost = f32::MAX;

        let mut index = self.root_index();

        let root_node = self.nodes.get(index.0).unwrap();
        let mut area_base = root_node.bounding_volume.surface_area_heuristic();
        let mut direct_cost = root_node
            .bounding_volume
            .union(&bounding_volume)
            .surface_area_heuristic();

        loop {
            let node = self.nodes.get(index.0).unwrap();
            let NodeType::Internal {
                child1: child1_index,
                child2: child2_index,
            } = node.ty
            else {
                break;
            };

            let cost = direct_cost + inherited_cost;

            if cost < best_cost {
                best_cost = cost;
                best_sibling = index;
            }

            inherited_cost += direct_cost - area_base;

            let child1 = self.nodes.get(child1_index.0).unwrap();
            let child2 = self.nodes.get(child2_index.0).unwrap();

            let leaf1 = child1.ty.is_leaf();
            let leaf2 = child2.ty.is_leaf();

            let direct_cost1 = child1
                .bounding_volume
                .union(&bounding_volume)
                .surface_area_heuristic();
            let (area1, lower_cost1) = if leaf1 {
                let cost1 = direct_cost1 + inherited_cost;
                if cost1 < best_cost {
                    best_cost = cost1;
                    best_sibling = child1_index;
                }
                (0.0, f32::MAX)
            } else {
                let area1 = child1.bounding_volume.surface_area_heuristic();
                let lower_cost1 = inherited_cost + direct_cost1 + (area - area1).min(0.0);
                (area1, lower_cost1)
            };

            let direct_cost2 = child2
                .bounding_volume
                .union(&bounding_volume)
                .surface_area_heuristic();
            let (area2, lower_cost2) = if leaf2 {
                let cost2 = direct_cost2 + inherited_cost;
                if cost2 < best_cost {
                    best_cost = cost2;
                    best_sibling = child2_index;
                }
                (0.0, f32::MAX)
            } else {
                let area2 = child2.bounding_volume.surface_area_heuristic();
                let lower_cost2 = inherited_cost + direct_cost2 + (area - area2).min(0.0);
                (area2, lower_cost2)
            };

            if leaf1 && leaf2 || best_cost <= lower_cost1 && best_cost <= lower_cost2 {
                break;
            }

            let (lower_cost1, lower_cost2) = if lower_cost1 == lower_cost2 && leaf1 == false {
                core::hint::cold_path();

                debug_assert!(lower_cost1 < f32::MAX);
                debug_assert!(lower_cost2 < f32::MAX);

                let d1 = child1.bounding_volume.center().distance_heuristic(&center);
                let d2 = child2.bounding_volume.center().distance_heuristic(&center);

                (d1, d2)
            } else {
                (lower_cost1, lower_cost2)
            };

            if lower_cost1 < lower_cost2 && leaf1 == false {
                index = child1_index;
                area_base = area1;
                direct_cost = direct_cost1;
            } else {
                index = child2_index;
                area_base = area2;
                direct_cost = direct_cost2;
            }

            // // If index is leaf, it can only be child2_index,
            // // and best_cost > lower_cost2 || best_cost > lower_cost1
            // debug_assert!(self.nodes[index.0].ty.is_internal());
        }

        best_sibling
    }

    fn rotate_to_balance(&mut self, index: NodeIndex) {
        let i = index.0;
        let node = &self.nodes[i];

        let NodeType::Internal {
            child1: child1_index,
            child2: child2_index,
        } = node.ty
        else {
            return;
        };
        let (c1i, c2i) = (child1_index.0, child2_index.0);

        let child1 = &self.nodes[c1i];
        let child2 = &self.nodes[c2i];

        match (child1.ty, child2.ty) {
            (NodeType::Leaf(_), NodeType::Leaf(_)) => {}
            (
                NodeType::Leaf(_),
                NodeType::Internal {
                    child1: grandchild3_index,
                    child2: grandchild4_index,
                },
            ) => {
                let (g3i, g4i) = (grandchild3_index.0, grandchild4_index.0);

                let base_cost = child2.bounding_volume.surface_area_heuristic();

                let aabb_c1g4 = child1
                    .bounding_volume
                    .union(&self.nodes[g4i].bounding_volume);
                let cost_c1g3 = aabb_c1g4.surface_area_heuristic();

                let aabb_c1g3 = child1
                    .bounding_volume
                    .union(&self.nodes[g3i].bounding_volume);
                let cost_c1g4 = aabb_c1g3.surface_area_heuristic();

                if base_cost < cost_c1g3 && base_cost < cost_c1g4 {
                    return;
                }

                self.nodes[c1i].parent_index = child2_index;

                if cost_c1g3 < cost_c1g4 {
                    *self.nodes[i].ty.as_internal_mut().0 = grandchild3_index;
                    *self.nodes[c2i].ty.as_internal_mut().0 = child1_index;

                    self.nodes[g3i].parent_index = index;

                    self.nodes[c2i].bounding_volume = aabb_c1g4;
                } else {
                    *self.nodes[i].ty.as_internal_mut().0 = grandchild4_index;
                    *self.nodes[c2i].ty.as_internal_mut().1 = child1_index;

                    self.nodes[g4i].parent_index = index;

                    self.nodes[c2i].bounding_volume = aabb_c1g3;
                }
            }
            (
                NodeType::Internal {
                    child1: grandchild1_index,
                    child2: grandchild2_index,
                },
                NodeType::Leaf(_),
            ) => {
                let (g1i, g2i) = (grandchild1_index.0, grandchild2_index.0);

                let base_cost = child1.bounding_volume.surface_area_heuristic();

                let aabb_c2g2 = child2
                    .bounding_volume
                    .union(&self.nodes[g2i].bounding_volume);
                let cost_c2g1 = aabb_c2g2.surface_area_heuristic();

                let aabb_c2g1 = child2
                    .bounding_volume
                    .union(&self.nodes[g1i].bounding_volume);
                let cost_c2g2 = aabb_c2g1.surface_area_heuristic();

                if base_cost < cost_c2g1 && base_cost < cost_c2g2 {
                    return;
                }

                self.nodes[c2i].parent_index = child1_index;

                if cost_c2g1 < cost_c2g2 {
                    *self.nodes[i].ty.as_internal_mut().1 = grandchild1_index;
                    *self.nodes[c1i].ty.as_internal_mut().0 = child2_index;

                    self.nodes[g1i].parent_index = index;

                    self.nodes[c1i].bounding_volume = aabb_c2g2;
                } else {
                    *self.nodes[i].ty.as_internal_mut().1 = grandchild2_index;
                    *self.nodes[c1i].ty.as_internal_mut().1 = child2_index;

                    self.nodes[g2i].parent_index = index;

                    self.nodes[c1i].bounding_volume = aabb_c2g1;
                }
            }
            (
                NodeType::Internal {
                    child1: grandchild1_index,
                    child2: grandchild2_index,
                },
                NodeType::Internal {
                    child1: grandchild3_index,
                    child2: grandchild4_index,
                },
            ) => {
                let g1i = grandchild1_index.0;
                let g2i = grandchild2_index.0;
                let g3i = grandchild3_index.0;
                let g4i = grandchild4_index.0;

                let area_c1 = child1.bounding_volume.surface_area_heuristic();
                let area_c2 = child2.bounding_volume.surface_area_heuristic();
                let base_cost = area_c1 + area_c2;

                enum RotationType {
                    None,
                    C1G3,
                    C1G4,
                    C2G1,
                    G2C2,
                }

                let mut best_cost = base_cost;
                let mut best_rotation = RotationType::None;

                let aabb_c1g4 = child1
                    .bounding_volume
                    .union(&self.nodes[g4i].bounding_volume);
                let cost_c1g3 = area_c1 + aabb_c1g4.surface_area_heuristic();
                if cost_c1g3 < best_cost {
                    best_cost = cost_c1g3;
                    best_rotation = RotationType::C1G3;
                }

                let aabb_c1g3 = child1
                    .bounding_volume
                    .union(&self.nodes[g3i].bounding_volume);
                let cost_c1g4 = area_c1 + aabb_c1g3.surface_area_heuristic();
                if cost_c1g4 < best_cost {
                    best_cost = cost_c1g4;
                    best_rotation = RotationType::C1G4;
                }

                let aabb_c2g2 = child2
                    .bounding_volume
                    .union(&self.nodes[g2i].bounding_volume);
                let cost_c2g1 = area_c2 + aabb_c2g2.surface_area_heuristic();
                if cost_c2g1 < best_cost {
                    best_cost = cost_c2g1;
                    best_rotation = RotationType::C2G1;
                }

                let aabb_c2g1 = child2
                    .bounding_volume
                    .union(&self.nodes[g1i].bounding_volume);
                let cost_c2g2 = area_c2 + aabb_c2g1.surface_area_heuristic();
                if cost_c2g2 < best_cost {
                    // best_cost = cost_c2g2;
                    best_rotation = RotationType::G2C2;
                }

                match best_rotation {
                    RotationType::None => {}
                    RotationType::C1G3 => {
                        *self.nodes[i].ty.as_internal_mut().0 = grandchild3_index;
                        *self.nodes[c2i].ty.as_internal_mut().0 = child1_index;

                        self.nodes[c1i].parent_index = child2_index;
                        self.nodes[g3i].parent_index = index;

                        self.nodes[c2i].bounding_volume = aabb_c1g4;
                    }
                    RotationType::C1G4 => {
                        *self.nodes[i].ty.as_internal_mut().0 = grandchild4_index;
                        *self.nodes[c2i].ty.as_internal_mut().1 = child1_index;

                        self.nodes[c1i].parent_index = child2_index;
                        self.nodes[g4i].parent_index = index;

                        self.nodes[c2i].bounding_volume = aabb_c1g3;
                    }
                    RotationType::C2G1 => {
                        *self.nodes[i].ty.as_internal_mut().1 = grandchild1_index;
                        *self.nodes[c1i].ty.as_internal_mut().0 = child2_index;

                        self.nodes[c2i].parent_index = child1_index;
                        self.nodes[g1i].parent_index = index;

                        self.nodes[c1i].bounding_volume = aabb_c2g2;
                    }
                    RotationType::G2C2 => {
                        *self.nodes[i].ty.as_internal_mut().1 = grandchild2_index;
                        *self.nodes[c1i].ty.as_internal_mut().1 = child2_index;

                        self.nodes[c2i].parent_index = child1_index;
                        self.nodes[g2i].parent_index = index;

                        self.nodes[c1i].bounding_volume = aabb_c2g1;
                    }
                }
            }
        }
    }

    fn root_index(&self) -> NodeIndex {
        self.root_index
    }

    pub fn remove_leaf(&mut self, index: NodeIndex) {
        let node = &self.nodes[index.0];

        if node.ty.is_internal() {
            panic!("Node {:?} to be removed is not leaf!", index.0);
        }

        if index == self.root_index() {
            self.root_index = NodeIndex::NULL;
            self.nodes.remove(index.0);
            return;
        }

        let parent_index = node.parent_index;

        let parent = &self.nodes[parent_index.0];

        let (child1, child2) = parent.ty.as_internal();

        let sibling_index = if child1 == index {
            child2
        } else if child2 == index {
            child1
        } else {
            unreachable!()
        };

        let new_parent_index = if node.parent_index == self.root_index() {
            self.root_index = sibling_index;
            NodeIndex::NULL
        } else {
            let grand_parent_index = parent.parent_index;
            let grand_parent = &mut self.nodes[grand_parent_index.0];

            let (child1, child2) = grand_parent.ty.as_internal_mut();

            if *child1 == parent_index {
                *child1 = sibling_index;
            } else if *child2 == parent_index {
                *child2 = sibling_index;
            } else {
                unreachable!()
            }

            grand_parent_index
        };

        self.nodes[sibling_index.0].parent_index = new_parent_index;

        self.nodes.remove(index.0);
        self.nodes.remove(parent_index.0);
    }
}

#[derive(Debug)]
pub struct EnlargedBvh<B, D> {
    bvh: Bvh<B, D>,
    enlargement: f32,
}
impl<B: BoundingVolume + Copy + Debug, D: Copy + Debug> EnlargedBvh<B, D> {
    pub fn new(enlargement: f32) -> Self {
        Self {
            bvh: Bvh::new(),
            enlargement,
        }
    }

    #[inline]
    pub fn query_intersection_stack(
        &self,
        stack: &mut Vec<NodeIndex>,
        bounding_volume: B,
        f: impl FnMut(D),
    ) {
        self.bvh.query_intersection_stack(stack, bounding_volume, f)
    }

    #[inline]
    pub fn query_intersection(&self, bounding_volume: B, f: impl FnMut(D)) {
        self.bvh.query_intersection(bounding_volume, f);
    }
    pub fn remove_leaf(&mut self, index: NodeIndex) {
        self.bvh.remove_leaf(index);
    }
    pub fn insert_leaf(&mut self, bounding_volume: B, entity: D) -> NodeIndex {
        self.bvh
            .insert_leaf(bounding_volume.enlarge(self.enlargement), entity)
    }
    pub fn update_leaf(&mut self, index: NodeIndex, bounding_volume: B) {
        let node = &self.bvh.nodes[index.0];
        if !node.bounding_volume.contains(&bounding_volume) {
            self.bvh
                .update_leaf(index, bounding_volume.enlarge(self.enlargement));
        }
    }
}
