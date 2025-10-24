// Dimension type definitions
// TODO: Implement dimension structure

pub struct Dimension {
    pub id: u8,
    pub name: String,
}

impl Dimension {
    pub fn new(id: u8, name: String) -> Self {
        Self { id, name }
    }
}
