//! Performance tests for memory manager
//!
//! These tests validate performance requirements through TDD

#[cfg(test)]
mod performance_tests {
    use crate::memory::*;
    use crate::DimensionId;
    use std::time::Instant;
    
    /// Test that initialization completes within 100ms budget
    #[tokio::test]
    async fn test_initialization_performance() {
        let start = Instant::now();
        
        let mut manager = MmapManager::new(280).unwrap();
        
        // Pre-allocate space for 14 core dimensions
        // This should complete quickly without loading actual files
        let init_result = manager.pre_allocate_dimensions();
        
        let elapsed = start.elapsed();
        
        assert!(init_result.is_ok());
        assert!(elapsed.as_millis() < 100, 
            "Initialization took {}ms, expected <100ms", elapsed.as_millis());
    }
    
    /// Test that layer access is <1ms
    #[tokio::test]
    async fn test_layer_access_latency() {
        let mut manager = MmapManager::new(280).unwrap();
        
        // Create a test layer
        let dimension_id = DimensionId(99);
        let content = b"test content for latency measurement".to_vec();
        let layer_id = manager.create_proto_dimension(dimension_id, content).unwrap();
        
        // Warm up (first access might be slower)
        let _ = manager.load_layer_context(layer_id);
        
        // Measure actual access time
        let start = Instant::now();
        let result = manager.load_layer_context(layer_id);
        let elapsed = start.elapsed();
        
        assert!(result.is_ok());
        assert!(elapsed.as_micros() < 1000, 
            "Layer access took {}μs, expected <1000μs (1ms)", elapsed.as_micros());
    }
    
    /// Test zero allocations in hot path
    #[tokio::test]
    async fn test_zero_copy_access() {
        let mut manager = MmapManager::new(280).unwrap();
        
        let dimension_id = DimensionId(99);
        let content = b"zero copy test content".to_vec();
        let layer_id = manager.create_proto_dimension(dimension_id, content).unwrap();
        
        // Access multiple times - should not allocate each time
        for _ in 0..100 {
            let context = manager.load_layer_context(layer_id).unwrap();
            // Content is String (owned), but the underlying read should be zero-copy
            assert!(!context.content.is_empty());
        }
        
        // This test validates the design - actual zero-copy verification
        // would require instrumentation or profiling tools
    }
    
    /// Test concurrent read performance
    #[tokio::test]
    async fn test_concurrent_read_performance() {
        use std::sync::Arc;
        use tokio::sync::RwLock;
        
        let mut manager = MmapManager::new(280).unwrap();
        
        // Create test layer
        let dimension_id = DimensionId(99);
        let content = b"concurrent access test".to_vec();
        let layer_id = manager.create_proto_dimension(dimension_id, content).unwrap();
        
        let manager = Arc::new(RwLock::new(manager));
        
        // Spawn 100 concurrent readers
        let mut handles = vec![];
        let start = Instant::now();
        
        for _ in 0..100 {
            let manager_clone = Arc::clone(&manager);
            let handle = tokio::spawn(async move {
                let mgr = manager_clone.read().await;
                mgr.load_layer_context(layer_id)
            });
            handles.push(handle);
        }
        
        // Wait for all to complete
        for handle in handles {
            let result = handle.await.unwrap();
            assert!(result.is_ok());
        }
        
        let elapsed = start.elapsed();
        
        // 100 concurrent reads should complete quickly
        assert!(elapsed.as_millis() < 100,
            "100 concurrent reads took {}ms, expected <100ms", elapsed.as_millis());
    }
    
    /// Test allocation performance
    #[tokio::test]
    async fn test_allocation_performance() {
        let mut manager = MmapManager::new(280).unwrap();
        
        let start = Instant::now();
        
        // Allocate 1000 small blocks
        let mut allocations = vec![];
        for _ in 0..1000 {
            let offset = manager.allocate(4096).unwrap();
            allocations.push(offset);
        }
        
        let elapsed = start.elapsed();
        let avg_micros = elapsed.as_micros() / 1000;
        
        // Each allocation should be <100μs on average
        assert!(avg_micros < 100,
            "Average allocation took {}μs, expected <100μs", avg_micros);
        
        // Cleanup
        for offset in allocations {
            let _ = manager.deallocate(offset);
        }
    }
}
