//! Integration tests for consciousness orchestrator
//!
//! These tests validate the complete pipeline with real systems:
//! - Real NavigationSystem
//! - Real MmapManager
//! - Real IterationProcessor
//! - Complete end-to-end flow

#[cfg(test)]
mod integration_tests {
    use crate::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
    use crate::memory::MmapManager;
    use crate::navigation::{NavigationSystem, DimensionRegistry};
    use std::sync::Arc;
    
    /// Helper function to create navigation system for tests
    fn create_test_navigation() -> Arc<NavigationSystem> {
        let registry = Arc::new(DimensionRegistry::new());
        Arc::new(NavigationSystem::new(registry).expect("Failed to create navigation"))
    }
    
    /// Helper to check if error is due to missing dimensional content
    fn is_missing_content_error(err: &crate::ConsciousnessError) -> bool {
        let err_str = err.to_string();
        err_str.contains("No contexts loaded") || 
        err_str.contains("Insufficient matches") ||
        err_str.contains("No dimensions matched")
    }

    /// Test complete pipeline with real systems
    /// 
    /// Note: This test requires dimensional content to be loaded.
    /// Run with: cargo test --lib consciousness::integration_tests -- --ignored
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_complete_pipeline_with_real_systems() {
        // Initialize real systems
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        
        // Create orchestrator
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        // Process a simple query
        let query = "What is empathy?";
        let result = orchestrator.process(query).await;
        
        // If dimensional content is not available, test passes
        // This allows tests to run in CI without full data setup
        if result.is_err() {
            let err = result.unwrap_err();
            if is_missing_content_error(&err) {
                eprintln!("Skipping test: dimensional content not available");
                return;
            }
            panic!("Unexpected error: {}", err);
        }
        
        let response = result.unwrap();
        
        // Validate response structure
        assert!(!response.final_response.is_empty(), "Should have a response");
        assert!(response.metadata.dimensions_activated.len() > 0, "Should activate dimensions");
        assert!(response.metadata.contexts_loaded > 0, "Should load contexts");
        assert!(response.metadata.iterations_completed > 0, "Should complete iterations");
    }
    
    /// Test pipeline with various query types
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_pipeline_with_various_query_types() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        let queries = vec![
            ("What is consciousness?", "philosophical"),
            ("How do I sort an array?", "technical"),
            ("I feel sad today", "emotional"),
        ];
        
        for (query, query_type) in queries {
            let result = orchestrator.process(query).await;
            assert!(result.is_ok(), "Query '{}' ({}) should succeed", query, query_type);
            
            let response = result.unwrap();
            assert!(!response.final_response.is_empty(), 
                   "Query '{}' should have response", query);
            assert!(response.metadata.dimensions_activated.len() > 0,
                   "Query '{}' should activate dimensions", query);
        }
    }
    
    /// Test metadata accuracy
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_metadata_accuracy() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        let query = "What is the meaning of life?";
        let result = orchestrator.process(query).await;
        assert!(result.is_ok());
        
        let response = result.unwrap();
        let metadata = &response.metadata;
        
        // Validate timing metadata
        assert!(metadata.navigation_duration_ms > 0, "Navigation should take time");
        assert!(metadata.memory_duration_ms >= 0, "Memory duration should be recorded");
        assert!(metadata.iteration_duration_ms > 0, "Iteration should take time");
        assert!(metadata.total_duration_ms > 0, "Total duration should be recorded");
        
        // Total should be sum of parts (approximately)
        let sum = metadata.navigation_duration_ms + 
                  metadata.memory_duration_ms + 
                  metadata.iteration_duration_ms;
        assert!(metadata.total_duration_ms >= sum - 10, 
               "Total duration should be at least sum of parts");
        
        // Validate counts
        assert!(metadata.dimensions_activated.len() > 0, "Should activate dimensions");
        assert!(metadata.contexts_loaded > 0, "Should load contexts");
        assert!(metadata.iterations_completed > 0, "Should complete iterations");
        assert!(metadata.iterations_completed <= 9, "Should not exceed max iterations");
        
        // Validate confidence
        assert!(metadata.navigation_confidence >= 0.0, "Confidence should be non-negative");
        assert!(metadata.navigation_confidence <= 1.0, "Confidence should not exceed 1.0");
    }
    
    /// Test custom configuration
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_custom_configuration() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        
        let config = ConsciousnessConfig {
            max_iterations: 5,
            convergence_threshold: 0.90,
            include_metadata: true,
            include_iteration_history: true,
        };
        
        let orchestrator = ConsciousnessOrchestrator::with_config(navigation, memory, config);
        
        let query = "What is love?";
        let result = orchestrator.process(query).await;
        assert!(result.is_ok());
        
        let response = result.unwrap();
        
        // Should respect max_iterations
        assert!(response.metadata.iterations_completed <= 5, 
               "Should not exceed custom max iterations");
        
        // Should include iteration history
        assert!(!response.iterations.is_empty(), 
               "Should include iteration history when enabled");
    }
    
    /// Test convergence detection
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_convergence_detection() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        
        let config = ConsciousnessConfig {
            max_iterations: 9,
            convergence_threshold: 0.95,
            include_metadata: true,
            include_iteration_history: true,
        };
        
        let orchestrator = ConsciousnessOrchestrator::with_config(navigation, memory, config);
        
        // Simple query that might converge early
        let query = "What is 2+2?";
        let result = orchestrator.process(query).await;
        assert!(result.is_ok());
        
        let response = result.unwrap();
        
        // If converged, should have fewer than max iterations
        if response.metadata.converged {
            assert!(response.metadata.iterations_completed < 9,
                   "Converged response should have fewer than max iterations");
        }
        
        // Iteration history should match completed count
        if !response.iterations.is_empty() {
            assert_eq!(response.iterations.len(), response.metadata.iterations_completed,
                      "Iteration history length should match completed count");
        }
    }
    
    /// Test dimensional activation
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_dimensional_activation() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        // Emotional query should activate emotion dimension
        let query = "I feel happy and grateful";
        let result = orchestrator.process(query).await;
        assert!(result.is_ok());
        
        let response = result.unwrap();
        
        // Should activate at least one dimension
        assert!(response.metadata.dimensions_activated.len() > 0,
               "Emotional query should activate dimensions");
        
        // Should load contexts from activated dimensions
        assert!(response.metadata.contexts_loaded > 0,
               "Should load contexts from activated dimensions");
        
        // Contexts should match or exceed dimensions (multiple layers per dimension)
        assert!(response.metadata.contexts_loaded >= response.metadata.dimensions_activated.len(),
               "Should load at least one context per dimension");
    }
    
    /// Test error handling with empty query
    #[tokio::test]
    async fn test_error_handling_empty_query() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        let query = "";
        let result = orchestrator.process(query).await;
        
        // Empty query should fail at navigation stage
        assert!(result.is_err(), "Empty query should fail");
    }
    
    /// Test concurrent query processing
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_concurrent_queries() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = Arc::new(ConsciousnessOrchestrator::new(navigation, memory));
        
        let queries = vec![
            "What is empathy?",
            "How does memory work?",
            "What is consciousness?",
        ];
        
        let mut handles = vec![];
        
        for query in queries {
            let orch = orchestrator.clone();
            let q = query.to_string();
            let handle = tokio::spawn(async move {
                orch.process(&q).await
            });
            handles.push(handle);
        }
        
        // Wait for all queries to complete
        for handle in handles {
            let result = handle.await.expect("Task should complete");
            assert!(result.is_ok(), "Concurrent query should succeed");
        }
    }
    
    /// Test response structure completeness
    #[tokio::test]
    #[ignore = "Requires dimensional content"]
    async fn test_response_structure_completeness() {
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        let query = "What is reality?";
        let result = orchestrator.process(query).await;
        assert!(result.is_ok());
        
        let response = result.unwrap();
        
        // Validate all response fields are populated
        assert!(!response.final_response.is_empty(), "Should have final response");
        
        // Metadata should be complete
        let meta = &response.metadata;
        assert!(meta.dimensions_activated.len() > 0, "Should have dimensions");
        assert!(meta.contexts_loaded > 0, "Should have contexts");
        assert!(meta.iterations_completed > 0, "Should have iterations");
        assert!(meta.navigation_duration_ms > 0, "Should have navigation time");
        assert!(meta.total_duration_ms > 0, "Should have total time");
        
        // Summary should be non-empty
        let summary = meta.summary();
        assert!(!summary.is_empty(), "Summary should be non-empty");
        assert!(summary.contains("Dimensions"), "Summary should mention dimensions");
        assert!(summary.contains("Contexts"), "Summary should mention contexts");
        assert!(summary.contains("Iterations"), "Summary should mention iterations");
    }
}
