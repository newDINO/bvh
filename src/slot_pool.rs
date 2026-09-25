pub struct SlotPool<T> {
    slots: Vec<Option<T>>,
    frees: Vec<SlotPoolHandle>,
}

impl<T: Debug> Debug for SlotPool<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.iter().for_each(|value| {
            writeln!(f, "{:?}", value).unwrap();
        });
        Ok(())
    }
}

impl<T> Default for SlotPool<T> {
    fn default() -> Self {
        Self::new()
    }
}

mod handle {

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct SlotPoolHandle(u32);

    impl SlotPoolHandle {
        pub const NULL: Self = Self(u32::MAX);
        pub(super) fn from_usize(index: usize) -> Option<Self> {
            let index = index as u32;
            if index >= u32::MAX {
                None
            } else {
                Some(Self(index))
            }
        }
        pub(super) fn as_usize(&self) -> usize {
            self.0 as usize
        }
    }
}

use std::{
    fmt::Debug,
    ops::{Index, IndexMut},
};

pub use handle::SlotPoolHandle;

impl<T> SlotPool<T> {
    pub fn new() -> Self {
        Self {
            slots: Vec::new(),
            frees: Vec::new(),
        }
    }

    pub fn insert(&mut self, value: T) -> SlotPoolHandle {
        if let Some(free) = self.frees.pop() {
            self.slots[free.as_usize()] = Some(value);

            free
        } else {
            let index = self.slots.len();
            self.slots.push(Some(value));

            SlotPoolHandle::from_usize(index).unwrap()
        }
    }

    pub fn allocate_slot(&mut self) -> SlotPoolHandle {
        if let Some(free) = self.frees.pop() {
            free
        } else {
            let index = self.slots.len();
            self.slots.push(None);

            SlotPoolHandle::from_usize(index).unwrap()
        }
    }

    pub fn insert_at(&mut self, value: T, handle: SlotPoolHandle) -> Result<(), T> {
        if let Some(slot) = self.slots.get_mut(handle.as_usize()) {
            if slot.is_none() {
                *slot = Some(value);
                Ok(())
            } else {
                Err(value)
            }
        } else {
            Err(value)
        }
    }

    pub fn get(&self, handle: SlotPoolHandle) -> Option<&T> {
        self.slots.get(handle.as_usize())?.as_ref()
    }

    pub fn get_mut(&mut self, handle: SlotPoolHandle) -> Option<&mut T> {
        self.slots.get_mut(handle.as_usize())?.as_mut()
    }

    pub fn remove(&mut self, handle: SlotPoolHandle) -> Option<T> {
        if let Some(slot) = self.slots.get_mut(handle.as_usize()) {
            if let Some(value) = slot.take() {
                self.frees.push(handle);
                Some(value)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.slots.iter().filter_map(Option::as_ref)
    }
}

impl<T> Index<SlotPoolHandle> for SlotPool<T> {
    type Output = T;
    fn index(&self, index: SlotPoolHandle) -> &Self::Output {
        self.slots[index.as_usize()].as_ref().unwrap()
    }
}

impl<T> IndexMut<SlotPoolHandle> for SlotPool<T> {
    fn index_mut(&mut self, index: SlotPoolHandle) -> &mut Self::Output {
        self.slots[index.as_usize()].as_mut().unwrap()
    }
}
