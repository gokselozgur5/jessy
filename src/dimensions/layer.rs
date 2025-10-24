// Layer type definitions
// TODO: Implement layer structure

pub struct Layer {
    pub id: u8,
    pub dimension_id: u8,
}

impl Layer {
    pub fn new(id: u8, dimension_id: u8) -> Self {
        Self { id, dimension_id }
    }
}
