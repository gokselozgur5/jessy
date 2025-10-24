// Dimension registry
// TODO: Implement registry

use super::Dimension;

pub struct DimensionRegistry {
    dimensions: Vec<Dimension>,
}

impl DimensionRegistry {
    pub fn new() -> Self {
        Self {
            dimensions: Vec::new(),
        }
    }
}
