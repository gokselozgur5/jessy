//! OWL Pattern Encoding for Dimension Activation
//!
//! Binary encoding of active dimensions for efficient representation.
//! Each dimension (1-14) is represented by a single bit.

use crate::DimensionId;

/// OWL (Ontological Web Language) pattern for dimension activation
///
/// Represents which dimensions are active using a 14-bit binary pattern.
/// Example: [2,4,10] → "01010000010000"
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OwlPattern {
    /// Binary pattern string (14 characters of '0' or '1')
    pattern: String,
    /// Active dimension IDs
    dimensions: Vec<DimensionId>,
}

impl OwlPattern {
    /// Encode dimension IDs into OWL pattern
    ///
    /// # Example
    /// ```
    /// use jessy::navigation::OwlPattern;
    /// use jessy::DimensionId;
    ///
    /// let pattern = OwlPattern::encode(&[DimensionId(2), DimensionId(4), DimensionId(10)]);
    /// assert_eq!(pattern.as_str(), "01010000010000");
    /// ```
    pub fn encode(dimensions: &[DimensionId]) -> Self {
        let mut bits = [0u8; 14];
        
        for dim_id in dimensions {
            let idx = dim_id.0 as usize - 1;
            if idx < 14 {
                bits[idx] = 1;
            }
        }
        
        let pattern: String = bits.iter().map(|&b| if b == 1 { '1' } else { '0' }).collect();
        
        Self {
            pattern,
            dimensions: dimensions.to_vec(),
        }
    }
    
    /// Decode OWL pattern into dimension IDs
    ///
    /// # Example
    /// ```
    /// use jessy::navigation::OwlPattern;
    /// use jessy::DimensionId;
    ///
    /// let dims = OwlPattern::decode("01010000010000");
    /// assert_eq!(dims, vec![DimensionId(2), DimensionId(4), DimensionId(10)]);
    /// ```
    pub fn decode(pattern: &str) -> Vec<DimensionId> {
        pattern
            .chars()
            .enumerate()
            .filter_map(|(idx, ch)| {
                if ch == '1' {
                    Some(DimensionId((idx + 1) as u8))
                } else {
                    None
                }
            })
            .collect()
    }
    
    /// Get pattern as string
    pub fn as_str(&self) -> &str {
        &self.pattern
    }
    
    /// Get active dimensions
    pub fn dimensions(&self) -> &[DimensionId] {
        &self.dimensions
    }
    
    /// Convert to u16 for compact representation
    pub fn to_u16(&self) -> u16 {
        let mut value = 0u16;
        for (idx, ch) in self.pattern.chars().enumerate() {
            if ch == '1' {
                value |= 1 << idx;
            }
        }
        value
    }
    
    /// Create from u16
    pub fn from_u16(value: u16) -> Self {
        let mut dimensions = Vec::new();
        let mut pattern = String::with_capacity(14);
        
        for i in 0..14 {
            if value & (1 << i) != 0 {
                pattern.push('1');
                dimensions.push(DimensionId((i + 1) as u8));
            } else {
                pattern.push('0');
            }
        }
        
        Self { pattern, dimensions }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_encode_single_dimension() {
        let pattern = OwlPattern::encode(&[DimensionId(1)]);
        assert_eq!(pattern.as_str(), "10000000000000");
    }
    
    #[test]
    fn test_encode_multiple_dimensions() {
        let pattern = OwlPattern::encode(&[DimensionId(2), DimensionId(4), DimensionId(10)]);
        assert_eq!(pattern.as_str(), "01010000010000");
    }
    
    #[test]
    fn test_decode() {
        let dims = OwlPattern::decode("01010000010000");
        assert_eq!(dims, vec![DimensionId(2), DimensionId(4), DimensionId(10)]);
    }
    
    #[test]
    fn test_roundtrip() {
        let original = vec![DimensionId(1), DimensionId(7), DimensionId(14)];
        let pattern = OwlPattern::encode(&original);
        let decoded = OwlPattern::decode(pattern.as_str());
        assert_eq!(original, decoded);
    }
    
    #[test]
    fn test_to_u16() {
        let pattern = OwlPattern::encode(&[DimensionId(1), DimensionId(2)]);
        let value = pattern.to_u16();
        assert_eq!(value, 0b11); // First two bits set
    }
    
    #[test]
    fn test_from_u16() {
        let pattern = OwlPattern::from_u16(0b11);
        assert_eq!(pattern.dimensions(), &[DimensionId(1), DimensionId(2)]);
    }
}
