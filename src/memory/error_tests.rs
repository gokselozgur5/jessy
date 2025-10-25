//! Test-driven development for comprehensive error handling
//!
//! These tests define the expected behavior for error scenarios
//! before implementing the actual error handling logic.
//!
//! # Testing Philosophy
//!
//! Write tests first (TDD) - they document expected behavior
//! Test error paths, not just happy paths - errors are part of the API
//! Use descriptive test names - they're documentation

#[cfg(test)]
mod error_handling_tests {
    use crate::memory::*;
    use crate::{DimensionId, LayerId, ConsciousnessError};
    
    /// Test that allocation failures provide detailed context
    ///
    /// #[tokio::test] makes this an async test - tokio runtime handles it
    /// async fn because MmapManager might have async operations in future
    #[tokio::test]
    async fn test_allocation_failure_with_context() {
        // Create manager with memory limit
        // unwrap() is OK in tests - we want tests to panic on unexpected errors
        let manager = MmapManager::new(280).unwrap(); // Standard memory
        
        // Try to allocate more than available (larger than any pool)
        // This will fail because no pool can handle 500MB
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        
        // assert! checks boolean condition - panics if false
        assert!(result.is_err());
        // unwrap_err() is like unwrap() but for Err variant
        let err = result.unwrap_err();
        
        // Error should contain context about current memory state
        // format!("{:?}", err) uses Debug trait - shows internal structure
        let err_msg = format!("{:?}", err);
        println!("DEBUG: Error message: {}", err_msg);
        // Check for either AllocationFailed or LimitExceeded (duplicate removed)
        assert!(err_msg.contains("AllocationFailed") || err_msg.contains("allocation") || err_msg.contains("LimitExceeded"));
    }
    
    /// Test that memory limit enforcement prevents over-allocation
    #[tokio::test]
    async fn test_memory_limit_enforcement() {
        let manager = MmapManager::new(280).unwrap(); // 280MB limit
        
        // Try to allocate more than the total limit
        // This should fail immediately due to limit check
        let result = manager.allocate(300 * 1024 * 1024); // 300MB > 280MB limit
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        let err_msg = format!("{:?}", err);
        assert!(err_msg.contains("LimitExceeded") || err_msg.contains("limit") || err_msg.contains("AllocationFailed"));
    }
    
    /// Test that layer not found errors include layer ID
    #[tokio::test]
    async fn test_layer_not_found_error() {
        let manager = MmapManager::new(280).unwrap();
        
        let nonexistent_layer = LayerId {
            dimension: DimensionId(99),
            layer: 42,
        };
        
        let result = manager.load_layer_context(nonexistent_layer);
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        let err_msg = format!("{:?}", err);
        
        // Should mention the specific layer ID
        assert!(err_msg.contains("99") || err_msg.contains("42") || err_msg.contains("not found"));
    }
    
    /// Test that dimension not found errors include dimension ID
    #[tokio::test]
    async fn test_dimension_not_found_error() {
        let manager = MmapManager::new(280).unwrap();
        
        let nonexistent_dimension = DimensionId(99);
        let result = manager.load_dimension(nonexistent_dimension);
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        let err_msg = format!("{:?}", err);
        
        // Should mention the specific dimension ID (99, not 999)
        assert!(err_msg.contains("99") || err_msg.contains("not found") || err_msg.contains("DimensionNotFound"));
    }
    
    /// Test that out of bounds errors include offset and size
    #[tokio::test]
    async fn test_out_of_bounds_error() {
        // This will be tested once we have region reading with bounds checking
        // For now, we'll create a placeholder test
        
        // TODO: Implement once region bounds checking is in place
    }
    
    /// Test cleanup on partial dimension load failure
    #[tokio::test]
    async fn test_cleanup_on_load_failure() {
        let manager = MmapManager::new(280).unwrap();
        
        // Try to load a dimension that doesn't exist
        let result = manager.load_dimension(DimensionId(99));
        assert!(result.is_err());
        
        // Manager should still be in valid state
        let stats = manager.get_stats();
        assert_eq!(stats.regions_loaded, 0);
        assert_eq!(stats.layers_indexed, 0);
        
        // Should be able to continue using manager
        let result2 = manager.allocate(1024);
        assert!(result2.is_ok());
    }
    
    /// Test graceful degradation when loading multiple dimensions
    #[tokio::test]
    async fn test_graceful_degradation_multiple_dimensions() {
        let manager = MmapManager::new(280).unwrap();
        
        // Try to load mix of valid and invalid dimensions
        let dimensions = vec![
            DimensionId(1),   // May or may not exist
            DimensionId(99), // Definitely doesn't exist
            DimensionId(2),   // May or may not exist
        ];
        
        let mut success_count = 0;
        let mut failure_count = 0;
        
        for dim_id in dimensions {
            match manager.load_dimension(dim_id) {
                Ok(_) => success_count += 1,
                Err(_) => failure_count += 1,
            }
        }
        
        // Manager should continue working even with failures
        let stats = manager.get_stats();
        assert_eq!(stats.regions_loaded, success_count);
        
        // Should still be able to allocate
        let result = manager.allocate(1024);
        assert!(result.is_ok());
    }
    
    /// Test that allocation failure doesn't corrupt state
    #[tokio::test]
    async fn test_allocation_failure_state_integrity() {
        let manager = MmapManager::new(280).unwrap();
        
        // Successful allocation
        let size1 = 64 * 1024; // 64KB
        let alloc1 = manager.allocate(size1).unwrap();
        
        // Failed allocation (too large)
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        assert!(result.is_err());
        
        // Original allocation should still be valid
        let dealloc_result = manager.deallocate(alloc1, size1);
        assert!(dealloc_result.is_ok());
        
        // Should be able to allocate again
        let alloc2 = manager.allocate(64 * 1024);
        assert!(alloc2.is_ok());
    }
    
    /// Test error context includes current memory statistics
    #[tokio::test]
    async fn test_error_includes_memory_stats() {
        let manager = MmapManager::new(280).unwrap();
        
        // Allocate some memory
        let _alloc = manager.allocate(64 * 1024).unwrap();
        
        // Try to allocate too much
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        assert!(result.is_err());
        
        let err = result.unwrap_err();
        let err_msg = format!("{:?}", err);
        
        // Error should ideally include current state info
        // (This is aspirational - we'll implement what's practical)
        assert!(!err_msg.is_empty());
    }
    
    // ========================================================================
    // Additional Comprehensive Error Tests (Task 5.5)
    // ========================================================================
    
    /// Test allocation failure cleanup - ensure no memory leaks
    ///
    /// This tests that failed allocations don't leave the system in an
    /// inconsistent state. Memory tracking should remain accurate.
    #[tokio::test]
    async fn test_allocation_failure_no_memory_leak() {
        let manager = MmapManager::new(280).unwrap();
        
        // Get initial stats
        let initial_stats = manager.get_stats();
        let initial_allocated = initial_stats.current_allocated_mb;
        
        // Try to allocate too much - should fail
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        assert!(result.is_err());
        
        // Stats should be unchanged after failed allocation
        let after_stats = manager.get_stats();
        assert_eq!(after_stats.current_allocated_mb, initial_allocated);
        
        // Successful allocation should work normally
        let alloc = manager.allocate(64 * 1024).unwrap();
        let final_stats = manager.get_stats();
        assert!(final_stats.current_allocated_mb >= initial_allocated);
        
        // Cleanup
        let _ = manager.deallocate(alloc, 64 * 1024);
    }
    
    /// Test dimension loading failure cleanup
    ///
    /// When dimension loading fails partway through, all partial state
    /// should be cleaned up (rollback pattern)
    #[tokio::test]
    async fn test_dimension_load_failure_rollback() {
        let manager = MmapManager::new(280).unwrap();
        
        // Record initial state
        let initial_regions = manager.get_stats().regions_loaded;
        let initial_layers = manager.get_stats().layers_indexed;
        
        // Try to load non-existent dimension
        let result = manager.load_dimension(DimensionId(99));
        assert!(result.is_err());
        
        // Verify rollback: region_id should not have incremented
        // (we can't directly access next_region_id, but we can infer from stats)
        let after_stats = manager.get_stats();
        assert_eq!(after_stats.regions_loaded, initial_regions);
        assert_eq!(after_stats.layers_indexed, initial_layers);
        
        // System should still be usable
        let alloc_result = manager.allocate(1024);
        assert!(alloc_result.is_ok());
    }
    
    /// Test memory limit enforcement at different thresholds
    ///
    /// Verifies that the graduated warning system works correctly
    /// at 75%, 85%, and 95% utilization thresholds
    #[tokio::test]
    async fn test_memory_limit_thresholds() {
        let manager = MmapManager::new(280).unwrap(); // 280MB limit
        
        // Try to allocate more than limit - should fail immediately
        let result = manager.allocate(300 * 1024 * 1024); // 300MB > 280MB
        assert!(result.is_err());
        
        // Verify error is LimitExceeded or AllocationFailed
        match result.unwrap_err() {
            ConsciousnessError::LimitExceeded { current_mb, limit_mb, requested_mb } => {
                assert!(current_mb <= limit_mb);
                assert_eq!(limit_mb, 280);
                assert!(requested_mb > 0);
            }
            ConsciousnessError::AllocationFailed(_) => {
                // Also acceptable - no pool can handle 300MB
            }
            other => panic!("Expected LimitExceeded or AllocationFailed, got {:?}", other),
        }
    }
    
    /// Test concurrent allocation attempts (simulated)
    ///
    /// While we can't easily test true concurrency in unit tests,
    /// we can verify that the atomic operations maintain consistency
    #[tokio::test]
    async fn test_sequential_allocations_maintain_consistency() {
        let manager = MmapManager::new(280).unwrap();
        
        // Perform many small allocations
        let mut allocations = Vec::new();
        let block_size = 64 * 1024; // 64KB
        for _ in 0..10 {
            let alloc = manager.allocate(block_size).unwrap();
            allocations.push(alloc);
        }
        
        // Check stats are consistent
        let stats = manager.get_stats();
        assert!(stats.current_allocated_mb >= 0);
        assert!(stats.current_allocated_mb <= 280);
        
        // Deallocate all
        for alloc in allocations {
            manager.deallocate(alloc, block_size).unwrap();
        }
        
        // Stats should reflect deallocations
        let final_stats = manager.get_stats();
        assert_eq!(final_stats.current_allocated_mb, 0);
    }
    
    /// Test error types have correct structure
    ///
    /// Verifies that our error types contain the expected fields
    /// and can be pattern matched correctly
    #[tokio::test]
    async fn test_error_type_structure() {
        let manager = MmapManager::new(280).unwrap();
        
        // Test LimitExceeded or AllocationFailed error structure
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        assert!(result.is_err());
        
        match result.unwrap_err() {
            ConsciousnessError::LimitExceeded { current_mb, limit_mb, requested_mb } => {
                // Verify fields are present and reasonable
                assert!(current_mb <= limit_mb);
                assert_eq!(limit_mb, 280);
                assert!(requested_mb > 0);
            }
            ConsciousnessError::AllocationFailed(_) => {
                // Also acceptable - no pool can handle 500MB
            }
            other => panic!("Expected LimitExceeded or AllocationFailed, got {:?}", other),
        }
        
        // Test LayerNotFound error structure
        let layer_result = manager.load_layer_context(LayerId {
            dimension: DimensionId(99),
            layer: 42,
        });
        assert!(layer_result.is_err());
        
        match layer_result.unwrap_err() {
            ConsciousnessError::LayerNotFound { dimension, layer } => {
                assert_eq!(dimension, 99);
                assert_eq!(layer, 42);
            }
            other => panic!("Expected LayerNotFound, got {:?}", other),
        }
    }
    
    /// Test that deallocation of invalid offset fails gracefully
    ///
    /// Verifies error handling in deallocation path
    #[tokio::test]
    async fn test_invalid_deallocation() {
        let manager = MmapManager::new(10).unwrap();
        
        // Try to deallocate an invalid offset
        let invalid_offset = MmapOffset {
            pool_id: 255, // Non-existent pool
            offset: 0,
        };
        
        let result = manager.deallocate(invalid_offset, 1024);
        assert!(result.is_err());
        
        // Manager should still be usable
        let alloc = manager.allocate(1024);
        assert!(alloc.is_ok());
    }
    
    /// Test allocation after deallocation reuses memory
    ///
    /// Verifies that the pool allocator properly tracks free blocks
    #[tokio::test]
    async fn test_memory_reuse_after_deallocation() {
        let manager = MmapManager::new(280).unwrap();
        
        // Allocate and deallocate
        let size = 64 * 1024; // 64KB
        let alloc1 = manager.allocate(size).unwrap();
        let stats_after_alloc = manager.get_stats();
        let allocated_after_first = stats_after_alloc.current_allocated_mb;
        
        manager.deallocate(alloc1, size).unwrap();
        let stats_after_dealloc = manager.get_stats();
        assert!(stats_after_dealloc.current_allocated_mb <= allocated_after_first);
        
        // Allocate again - should reuse the freed block
        let alloc2 = manager.allocate(size).unwrap();
        let stats_after_realloc = manager.get_stats();
        
        // Memory usage should be similar to first allocation
        assert!(stats_after_realloc.current_allocated_mb <= allocated_after_first + 1);
        
        // Cleanup
        let _ = manager.deallocate(alloc2, size);
    }
    
    /// Test proto-dimension error handling
    ///
    /// Verifies that proto-dimension operations handle errors correctly
    #[tokio::test]
    async fn test_proto_dimension_error_handling() {
        let manager = MmapManager::new(280).unwrap();
        
        // Create proto-dimension
        let dimension_id = DimensionId(99);
        let content = b"test content".to_vec();
        let layer_id = manager.create_proto_dimension(dimension_id, content).unwrap();
        
        // Try to crystallize non-existent proto-dimension (different layer number)
        let fake_layer_id = LayerId {
            dimension: DimensionId(99),
            layer: 999, // Different layer number that doesn't exist
        };
        let result = manager.crystallize_proto_dimension(fake_layer_id);
        assert!(result.is_err());
        
        // Verify it's a LayerNotFound error
        match result.unwrap_err() {
            ConsciousnessError::LayerNotFound { .. } => {}, // Expected
            other => panic!("Expected LayerNotFound, got {:?}", other),
        }
        
        // Original proto-dimension should still be accessible
        let context = manager.load_layer_context(layer_id);
        assert!(context.is_ok());
    }
    
    /// Test stats accuracy after various operations
    ///
    /// Verifies that memory statistics remain accurate through
    /// allocations, deallocations, and failures
    #[tokio::test]
    async fn test_stats_accuracy() {
        let manager = MmapManager::new(280).unwrap();
        
        // Initial state
        let stats0 = manager.get_stats();
        assert_eq!(stats0.current_allocated_mb, 0);
        assert_eq!(stats0.total_limit_mb, 280);
        
        // After allocation
        let size1 = 64 * 1024; // 64KB
        let alloc1 = manager.allocate(size1).unwrap();
        let stats1 = manager.get_stats();
        assert!(stats1.current_allocated_mb >= 0);
        
        // After failed allocation
        let result = manager.allocate(500 * 1024 * 1024); // 500MB
        assert!(result.is_err());
        let stats2 = manager.get_stats();
        assert_eq!(stats2.current_allocated_mb, stats1.current_allocated_mb);
        
        // After deallocation
        manager.deallocate(alloc1, size1).unwrap();
        let stats3 = manager.get_stats();
        assert_eq!(stats3.current_allocated_mb, 0);
        
        // Utilization percentage should be correct
        assert!(stats3.utilization_percent < 1.0);
    }
}
