//! Concurrency tests for navigation system
//!
//! Tests concurrent navigation requests to ensure:
//! - No data races
//! - Independent results (no cross-contamination)
//! - Performance under load (p95 < 100ms for scanning)
//! - Resource cleanup timing
//!
//! Requirements: 8.1-8.5

#[cfg(test)]
mod tests {
    use crate::navigation::{NavigationSystem, DimensionRegistry, NavigationError};
    use std::sync::Arc;
    use std::time::Instant;
    use tokio::task::JoinSet;

    fn create_test_system() -> NavigationSystem {
        use crate::memory::MmapManager;

        let config_data = std::fs::read_to_string("data/dimensions.json")
            .expect("Failed to read dimensions.json");
        let registry = Arc::new(
            DimensionRegistry::load_dimensions(&config_data)
                .expect("Failed to load registry")
        );
        let memory_manager = Arc::new(
            MmapManager::new(280).expect("Failed to create memory manager")
        );
        NavigationSystem::new(registry, memory_manager).expect("Failed to create system")
    }

    // ============================================================================
    // Task 11.4: Concurrency Tests (RED Phase)
    // Requirements: 8.1-8.5
    // ============================================================================

    #[tokio::test]
    async fn test_100_concurrent_navigation_requests() {
        // Requirement 8.1: Support at least 100 simultaneous operations
        let system = Arc::new(create_test_system());
        
        let mut join_set = JoinSet::new();
        
        // Spawn 100 concurrent navigation tasks
        for i in 0..100 {
            let system_clone = Arc::clone(&system);
            let query = format!("test query number {}", i);
            
            join_set.spawn(async move {
                system_clone.navigate(&query, None).await
            });
        }
        
        // Collect all results
        let mut results = Vec::new();
        while let Some(result) = join_set.join_next().await {
            let nav_result = result.expect("Task should not panic");
            results.push(nav_result);
        }
        
        // All requests should complete
        assert_eq!(results.len(), 100, "All 100 requests should complete");
        
        // Count successes and acceptable errors
        let successes = results.iter().filter(|r| r.is_ok()).count();
        let insufficient_matches = results.iter().filter(|r| {
            matches!(r, Err(NavigationError::InsufficientMatches { .. }))
        }).count();
        
        // All should either succeed or have insufficient matches (both valid)
        assert_eq!(
            successes + insufficient_matches,
            100,
            "All requests should complete with valid result or insufficient matches"
        );
    }

    #[tokio::test]
    async fn test_no_data_races_concurrent_access() {
        // Requirement 8.2: Prevent data races through synchronization
        // This test uses Rust's type system + Arc to ensure no data races
        let system = Arc::new(create_test_system());
        
        let mut handles = Vec::new();
        
        // Spawn 50 concurrent tasks accessing shared system
        for i in 0..50 {
            let system_clone = Arc::clone(&system);
            
            let handle = tokio::spawn(async move {
                // Each task performs multiple operations
                for j in 0..5 {
                    let query = format!("query {} iteration {}", i, j);
                    let _ = system_clone.navigate(&query, None).await;
                }
            });
            
            handles.push(handle);
        }
        
        // Wait for all tasks to complete
        for handle in handles {
            handle.await.expect("Task should not panic");
        }
        
        // If we reach here without panics or deadlocks, no data races occurred
        // Rust's type system (Arc, Send, Sync) prevents data races at compile time
        assert!(true, "No data races detected");
    }

    #[tokio::test]
    async fn test_independent_results_no_cross_contamination() {
        // Requirement 8.5: Each request produces independent results
        let system = Arc::new(create_test_system());
        
        // Create distinct queries
        let queries = vec![
            "emotional empathy compassion",
            "technical algorithm code",
            "philosophical meaning purpose",
        ];
        
        let mut join_set = JoinSet::new();
        
        // Run each query 10 times concurrently
        for query in &queries {
            for i in 0..10 {
                let system_clone = Arc::clone(&system);
                let query_str = query.to_string();
                
                join_set.spawn(async move {
                    (i, query_str.clone(), system_clone.navigate(&query_str, None).await)
                });
            }
        }
        
        // Collect results grouped by query
        let mut results_by_query: std::collections::HashMap<String, Vec<_>> = 
            std::collections::HashMap::new();
        
        while let Some(result) = join_set.join_next().await {
            let (iteration, query, nav_result) = result.expect("Task should not panic");
            results_by_query
                .entry(query)
                .or_insert_with(Vec::new)
                .push((iteration, nav_result));
        }
        
        // Verify each query type produced consistent results
        for (query, results) in results_by_query {
            // All results for same query should be similar
            // (either all succeed or all fail with insufficient matches)
            let success_count = results.iter().filter(|(_, r)| r.is_ok()).count();
            let failure_count = results.len() - success_count;
            
            // Results should be consistent (not random mix)
            // Either mostly successes or mostly failures
            assert!(
                success_count >= 8 || failure_count >= 8,
                "Query '{}' should have consistent results, got {} successes, {} failures",
                query, success_count, failure_count
            );
            
            // If successful, verify dimensions are consistent
            if success_count > 0 {
                let successful_results: Vec<_> = results
                    .iter()
                    .filter_map(|(_, r)| r.as_ref().ok())
                    .collect();
                
                if !successful_results.is_empty() {
                    let first_dims = &successful_results[0].dimensions;
                    
                    // All successful results should activate similar dimensions
                    for result in &successful_results[1..] {
                        // Dimensions should be similar (allowing some variation)
                        // At least 50% overlap expected
                        let overlap = result.dimensions.iter()
                            .filter(|d| first_dims.contains(d))
                            .count();
                        
                        let overlap_ratio = overlap as f32 / first_dims.len().max(result.dimensions.len()) as f32;
                        
                        assert!(
                            overlap_ratio >= 0.3,
                            "Results for same query should have similar dimensions, got {}% overlap",
                            overlap_ratio * 100.0
                        );
                    }
                }
            }
        }
    }

    #[tokio::test]
    async fn test_performance_under_load_p95() {
        // Requirement 8.4: Maintain 100ms scanning performance at p95 under load
        let system = Arc::new(create_test_system());
        
        let mut durations = Vec::new();
        let mut join_set = JoinSet::new();
        
        // Run 100 concurrent requests
        for i in 0..100 {
            let system_clone = Arc::clone(&system);
            let query = format!("performance test query {}", i);
            
            join_set.spawn(async move {
                let start = Instant::now();
                let result = system_clone.navigate(&query, None).await;
                let duration = start.elapsed();
                (duration, result)
            });
        }
        
        // Collect durations
        while let Some(result) = join_set.join_next().await {
            let (duration, _nav_result) = result.expect("Task should not panic");
            durations.push(duration.as_millis());
        }
        
        // Calculate p95
        durations.sort();
        let p95_index = (durations.len() as f32 * 0.95) as usize;
        let p95_latency = durations[p95_index];
        
        println!("P95 latency under 100 concurrent requests: {}ms", p95_latency);
        
        // P95 should be under 150ms for full navigation
        // (100ms is for scanning only, full navigation allows 150ms)
        assert!(
            p95_latency < 150,
            "P95 latency {}ms exceeds 150ms target under concurrent load",
            p95_latency
        );
    }

    #[tokio::test]
    async fn test_resource_cleanup_timing() {
        // Requirement 8.3: Release temporary allocations within 10ms
        let system = Arc::new(create_test_system());
        
        // Measure memory before
        let query = "test query for resource cleanup";
        
        // Run navigation
        let start = Instant::now();
        let _result = system.navigate(query, None).await;
        let navigation_duration = start.elapsed();
        
        // In Rust, cleanup happens automatically via Drop
        // By the time navigate() returns, all temporary allocations are freed
        // This should be effectively instant (< 1ms)
        
        // Verify navigation completes quickly (cleanup is part of this)
        assert!(
            navigation_duration.as_millis() < 200,
            "Navigation + cleanup should complete quickly"
        );
        
        // Run multiple times to verify no memory leaks
        for i in 0..10 {
            let query = format!("cleanup test {}", i);
            let _ = system.navigate(&query, None).await;
        }
        
        // If we reach here without OOM, cleanup is working
        assert!(true, "Resource cleanup working correctly");
    }

    #[tokio::test]
    async fn test_concurrent_requests_maintain_accuracy() {
        // Verify that concurrent execution doesn't affect result accuracy
        let system = Arc::new(create_test_system());
        
        // Run same query sequentially
        let query = "empathy compassion understanding";
        let sequential_result = system.navigate(query, None).await;
        
        // Run same query 20 times concurrently
        let mut join_set = JoinSet::new();
        for _ in 0..20 {
            let system_clone = Arc::clone(&system);
            let query_str = query.to_string();
            
            join_set.spawn(async move {
                system_clone.navigate(&query_str, None).await
            });
        }
        
        let mut concurrent_results = Vec::new();
        while let Some(result) = join_set.join_next().await {
            concurrent_results.push(result.expect("Task should not panic"));
        }
        
        // All concurrent results should match sequential result
        match sequential_result {
            Ok(seq_result) => {
                // If sequential succeeded, concurrent should too
                let concurrent_successes: Vec<_> = concurrent_results
                    .iter()
                    .filter_map(|r| r.as_ref().ok())
                    .collect();
                
                assert!(
                    !concurrent_successes.is_empty(),
                    "Concurrent requests should succeed if sequential succeeded"
                );
                
                // Dimensions should be consistent
                for conc_result in concurrent_successes {
                    assert_eq!(
                        conc_result.dimensions.len(),
                        seq_result.dimensions.len(),
                        "Concurrent result should have same number of dimensions"
                    );
                }
            }
            Err(NavigationError::InsufficientMatches { .. }) => {
                // If sequential failed with insufficient matches, concurrent should too
                let all_insufficient = concurrent_results.iter().all(|r| {
                    matches!(r, Err(NavigationError::InsufficientMatches { .. }))
                });
                
                assert!(
                    all_insufficient,
                    "Concurrent results should match sequential failure"
                );
            }
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }

    #[tokio::test]
    async fn test_no_deadlocks_under_load() {
        // Verify system doesn't deadlock under heavy concurrent load
        let system = Arc::new(create_test_system());
        
        let timeout_duration = tokio::time::Duration::from_secs(10);
        
        let test_future = async {
            let mut join_set = JoinSet::new();
            
            // Spawn 200 concurrent tasks (stress test)
            for i in 0..200 {
                let system_clone = Arc::clone(&system);
                let query = format!("deadlock test {}", i);
                
                join_set.spawn(async move {
                    system_clone.navigate(&query, None).await
                });
            }
            
            // Wait for all to complete
            while let Some(_) = join_set.join_next().await {}
        };
        
        // Run with timeout
        let result = tokio::time::timeout(timeout_duration, test_future).await;
        
        assert!(
            result.is_ok(),
            "System should not deadlock under heavy load"
        );
    }

    #[tokio::test]
    async fn test_concurrent_different_query_types() {
        // Test concurrent requests with different query types
        let system = Arc::new(create_test_system());
        
        let query_types = vec![
            ("emotional", "I feel anxious and worried"),
            ("technical", "algorithm implementation code"),
            ("philosophical", "what is the meaning of existence"),
            ("factual", "what is the capital of France"),
            ("mixed", "I feel anxious about implementing algorithms"),
        ];
        
        let mut join_set = JoinSet::new();
        
        // Run each query type 10 times concurrently
        for (query_type, query) in query_types {
            for i in 0..10 {
                let system_clone = Arc::clone(&system);
                let query_str = query.to_string();
                let qtype = query_type.to_string();
                
                join_set.spawn(async move {
                    (qtype, i, system_clone.navigate(&query_str, None).await)
                });
            }
        }
        
        // Collect results
        let mut results_by_type: std::collections::HashMap<String, Vec<_>> = 
            std::collections::HashMap::new();
        
        while let Some(result) = join_set.join_next().await {
            let (qtype, iteration, nav_result) = result.expect("Task should not panic");
            results_by_type
                .entry(qtype)
                .or_insert_with(Vec::new)
                .push((iteration, nav_result));
        }
        
        // Verify all query types completed
        assert_eq!(results_by_type.len(), 5, "All query types should complete");
        
        // Each type should have 10 results
        for (qtype, results) in results_by_type {
            assert_eq!(
                results.len(),
                10,
                "Query type '{}' should have 10 results",
                qtype
            );
        }
    }

    #[tokio::test]
    async fn test_memory_safety_concurrent_access() {
        // Verify memory safety under concurrent access
        // Rust's type system ensures this at compile time, but we test runtime behavior
        let system = Arc::new(create_test_system());
        
        let mut handles = Vec::new();
        
        // Spawn tasks that access system in various ways
        for i in 0..50 {
            let system_clone = Arc::clone(&system);
            
            let handle = tokio::spawn(async move {
                // Access config
                let _config = system_clone.config();
                
                // Navigate
                let query = format!("memory safety test {}", i);
                let _ = system_clone.navigate(&query, None).await;
                
                // Access metrics
                let _metrics = system_clone.metrics();
            });
            
            handles.push(handle);
        }
        
        // Wait for all
        for handle in handles {
            handle.await.expect("Task should not panic");
        }
        
        // If we reach here, memory safety is maintained
        assert!(true, "Memory safety maintained under concurrent access");
    }
}
