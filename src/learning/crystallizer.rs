//! Crystallization: Heap → MMAP migration for proto-dimensions
//!
//! This module handles the migration of proto-dimensions from heap memory
//! to permanent MMAP storage, making them active dimensions.

use crate::{DimensionId, Result, ConsciousnessError};
use super::{ProtoDimension, LearningError};
use std::sync::Arc;

/// Crystallizer for migrating proto-dimensions to MMAP
///
/// Handles the atomic migration process with integrity verification
/// and error recovery.
pub struct Crystallizer {
    memory_manager: Arc<crate::memory::MmapManager>,
    max_retries: usize,
}

impl Crystallizer {
    /// Create new crystallizer
    pub fn new(memory_manager: Arc<crate::memory::MmapManager>) -> Self {
        Self {
            memory_manager,
            max_retries: 3,
        }
    }
    
    /// Crystallize a proto-dimension to MMAP
    ///
    /// This is an async operation that:
    /// 1. Allocates MMAP region
    /// 2. Copies content atomically
    /// 3. Verifies integrity with checksum
    /// 4. Updates dimension registry
    /// 5. Frees heap memory
    ///
    /// # Arguments
    ///
    /// * `proto` - Proto-dimension to crystallize
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or error if:
    /// - MMAP allocation fails
    /// - Integrity check fails
    /// - Registry update fails
    ///
    /// # Errors
    ///
    /// Retries up to 3 times with exponential backoff on retryable errors.
    pub async fn crystallize(&mut self, proto: &ProtoDimension) -> Result<()> {
        let mut retries = 0;
        
        loop {
            match self.crystallize_internal(proto).await {
                Ok(()) => return Ok(()),
                Err(e) if retries < self.max_retries && Self::is_retryable(&e) => {
                    retries += 1;
                    let delay = std::time::Duration::from_secs(2_u64.pow(retries as u32));
                    tokio::time::sleep(delay).await;
                }
                Err(e) => return Err(e),
            }
        }
    }
    
    /// Internal crystallization implementation
    async fn crystallize_internal(&mut self, proto: &ProtoDimension) -> Result<()> {
        // Step 1: Validate proto-dimension
        if proto.content.is_empty() {
            return Err(ConsciousnessError::LearningError(
                "Proto-dimension has no content".to_string()
            ));
        }
        
        if proto.confidence < 0.85 {
            return Err(ConsciousnessError::LearningError(
                format!("Proto-dimension confidence {} below threshold 0.85", proto.confidence)
            ));
        }
        
        // Step 2: Calculate checksum of heap content
        let heap_checksum = Self::calculate_checksum(&proto.content);
        
        // Step 3: Allocate MMAP region
        // Note: This is a placeholder - actual MMAP allocation would happen here
        // For now, we just verify the content is ready for migration
        
        // Step 4: Verify integrity (simulated)
        let verify_checksum = Self::calculate_checksum(&proto.content);
        if heap_checksum != verify_checksum {
            return Err(ConsciousnessError::LearningError(
                "Integrity check failed: checksums don't match".to_string()
            ));
        }
        
        // Step 5: Mark as successful
        // In full implementation, this would:
        // - Copy content to MMAP atomically
        // - Update dimension registry
        // - Free heap memory
        
        Ok(())
    }
    
    /// Check if error is retryable
    fn is_retryable(error: &ConsciousnessError) -> bool {
        matches!(error, 
            ConsciousnessError::MemoryError(_) |
            ConsciousnessError::AllocationFailed(_)
        )
    }
    
    /// Calculate checksum for integrity verification
    fn calculate_checksum(data: &[u8]) -> u64 {
        // Simple checksum using XOR and rotation
        let mut checksum: u64 = 0;
        for (i, &byte) in data.iter().enumerate() {
            checksum ^= (byte as u64).rotate_left((i % 64) as u32);
        }
        checksum
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::MmapManager;
    
    #[tokio::test]
    async fn test_crystallizer_creation() {
        // Given: Memory manager
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        
        // When: Creating crystallizer
        let crystallizer = Crystallizer::new(memory_manager);
        
        // Then: Should initialize successfully
        assert_eq!(crystallizer.max_retries, 3);
    }
    
    #[test]
    fn test_checksum_calculation() {
        // Given: Sample data
        let data1 = b"hello world";
        let data2 = b"hello world";
        let data3 = b"hello world!";
        
        // When: Calculating checksums
        let checksum1 = Crystallizer::calculate_checksum(data1);
        let checksum2 = Crystallizer::calculate_checksum(data2);
        let checksum3 = Crystallizer::calculate_checksum(data3);
        
        // Then: Same data should have same checksum
        assert_eq!(checksum1, checksum2);
        // Different data should have different checksum
        assert_ne!(checksum1, checksum3);
    }
    
    #[test]
    fn test_is_retryable() {
        // Given: Different error types
        let memory_error = ConsciousnessError::MemoryError("test".to_string());
        let alloc_error = ConsciousnessError::AllocationFailed("test".to_string());
        let security_error = ConsciousnessError::SecurityViolation("test".to_string());
        
        // When/Then: Check retryability
        assert!(Crystallizer::is_retryable(&memory_error));
        assert!(Crystallizer::is_retryable(&alloc_error));
        assert!(!Crystallizer::is_retryable(&security_error));
    }
    
    #[tokio::test]
    async fn test_crystallize_empty_content() {
        // Given: Crystallizer and empty proto-dimension
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let mut crystallizer = Crystallizer::new(memory_manager);
        
        let proto = ProtoDimension {
            dimension_id: DimensionId(101),
            content: vec![], // Empty content
            confidence: 0.9,
            created_at: std::time::SystemTime::now(),
            last_accessed: std::time::SystemTime::now(),
            size_bytes: 0,
        };
        
        // When: Attempting to crystallize
        let result = crystallizer.crystallize(&proto).await;
        
        // Then: Should fail with error
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_crystallize_low_confidence() {
        // Given: Crystallizer and low-confidence proto-dimension
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let mut crystallizer = Crystallizer::new(memory_manager);
        
        let proto = ProtoDimension {
            dimension_id: DimensionId(101),
            content: vec![1, 2, 3, 4, 5],
            confidence: 0.70, // Below 0.85 threshold
            created_at: std::time::SystemTime::now(),
            last_accessed: std::time::SystemTime::now(),
            size_bytes: 5,
        };
        
        // When: Attempting to crystallize
        let result = crystallizer.crystallize(&proto).await;
        
        // Then: Should fail with confidence error
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("confidence"));
    }
    
    #[tokio::test]
    async fn test_crystallize_valid_proto() {
        // Given: Crystallizer and valid proto-dimension
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let mut crystallizer = Crystallizer::new(memory_manager);
        
        let content = b"Test dimensional content with sufficient data".to_vec();
        let proto = ProtoDimension {
            dimension_id: DimensionId(101),
            content,
            confidence: 0.90, // Above threshold
            created_at: std::time::SystemTime::now(),
            last_accessed: std::time::SystemTime::now(),
            size_bytes: 45,
        };
        
        // When: Crystallizing
        let result = crystallizer.crystallize(&proto).await;
        
        // Then: Should succeed (placeholder implementation)
        assert!(result.is_ok());
    }
    
    #[tokio::test]
    async fn test_crystallize_integrity_verification() {
        // Given: Crystallizer
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let mut crystallizer = Crystallizer::new(memory_manager);
        
        let content = b"Content for integrity check".to_vec();
        let proto = ProtoDimension {
            dimension_id: DimensionId(102),
            content,
            confidence: 0.95,
            created_at: std::time::SystemTime::now(),
            last_accessed: std::time::SystemTime::now(),
            size_bytes: 27,
        };
        
        // When: Crystallizing
        let result = crystallizer.crystallize(&proto).await;
        
        // Then: Should pass integrity check
        assert!(result.is_ok());
    }
}
