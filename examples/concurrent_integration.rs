//! Concurrent Integration Example
//!
//! Demonstrates thread-safe concurrent query processing:
//! - Multiple queries processed simultaneously
//! - Shared navigation and memory systems
//! - No data races or lock contention
//! - Performance monitoring
//!
//! Run with: cargo run --example concurrent_integration

use jessy::navigation::{NavigationSystem, DimensionRegistry};
use jessy::memory::MmapManager;
use jessy::Result;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::task;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();
    
    println!("🚀 Concurrent Integration Example");
    println!("=" .repeat(70));
    println!();
    
    // Initialize shared systems
    let (nav_system, memory_manager) = initialize_systems().await?;
    
    // Test queries
    let queries = vec![
        "I feel anxious about implementing algorithms",
        "What is the meaning of consciousness?",
        "How do I optimize database queries?",
        "I'm worried about climate change",
        "Explain quantum computing simply",
        "How can I be more creative?",
        "What are the ethics of AI?",
        "Help me understand recursion",
        "I feel overwhelmed with work",
        "What is the nature of time?",
    ];
    
    println!("📋 Processing {} queries concurrently...", queries.len());
    println!();
    
    // Process all queries concurrently
    let start = Instant::now();
    let results = process_queries_concurrent(
        &queries,
        &nav_system,
        &memory_manager,
    ).await;
    let total_duration = start.elapsed();
    
    // Show results
    println!("📊 Results:");
    println!("-" .repeat(70));
    
    let mut successful = 0;
    let mut failed = 0;
    let mut total_nav_time = Duration::ZERO;
    
    for (i, result) in results.iter().enumerate() {
        match result {
            Ok(metrics) => {
                successful += 1;
                total_nav_time += metrics.navigation_duration;
                println!("  ✅ Query {}: {} ms", i + 1, metrics.total_duration.as_millis());
            }
            Err(e) => {
                failed += 1;
                println!("  ❌ Query {}: {}", i + 1, e);
            }
        }
    }
    
    println!();
    println!("Summary:");
    println!("  - Total queries: {}", queries.len());
    println!("  - Successful: {}", successful);
    println!("  - Failed: {}", failed);
    println!("  - Total wall time: {} ms", total_duration.as_millis());
    println!("  - Average per query: {} ms", total_duration.as_millis() / queries.len() as u128);
    println!("  - Speedup: {:.1}x", 
        total_nav_time.as_millis() as f64 / total_duration.as_millis() as f64
    );
    println!();
    
    // Show memory statistics
    show_memory_stats(&memory_manager);
    
    Ok(())
}

/// Initialize shared systems
async fn initialize_systems() -> Result<(Arc<NavigationSystem>, Arc<MmapManager>)> {
    println!("🔧 Initializing shared systems...");
    
    // Load dimension registry
    let config_data = std::fs::read_to_string("data/dimensions.json")
        .unwrap_or_else(|_| r#"{"dimensions": []}"#.to_string());
    
    let registry = Arc::new(DimensionRegistry::load_dimensions(&config_data)?);
    let nav_system = Arc::new(NavigationSystem::new(registry)?);
    let memory_manager = Arc::new(MmapManager::new(280)?);
    
    println!("  ✓ Systems ready");
    println!();
    
    Ok((nav_system, memory_manager))
}

/// Process multiple queries concurrently
async fn process_queries_concurrent(
    queries: &[&str],
    nav_system: &Arc<NavigationSystem>,
    memory_manager: &Arc<MmapManager>,
) -> Vec<Result<QueryMetrics>> {
    // Spawn concurrent tasks
    let tasks: Vec<_> = queries
        .iter()
        .map(|&query| {
            let nav = Arc::clone(nav_system);
            let mem = Arc::clone(memory_manager);
            let query = query.to_string();
            
            task::spawn(async move {
                process_single_query(&query, &nav, &mem).await
            })
        })
        .collect();
    
    // Wait for all tasks to complete
    let results = futures::future::join_all(tasks).await;
    
    // Unwrap task results
    results
        .into_iter()
        .map(|r| r.unwrap_or_else(|e| Err(jessy::ConsciousnessError::MemoryError(e.to_string()))))
        .collect()
}

/// Process a single query and collect metrics
async fn process_single_query(
    query: &str,
    nav_system: &Arc<NavigationSystem>,
    memory_manager: &Arc<MmapManager>,
) -> Result<QueryMetrics> {
    let start = Instant::now();
    
    // Navigate
    let nav_start = Instant::now();
    let nav_result = nav_system.navigate(query).await?;
    let navigation_duration = nav_start.elapsed();
    
    // Load contexts
    let load_start = Instant::now();
    let contexts = match memory_manager.load_contexts(&nav_result.paths) {
        Ok(ctx) => ctx,
        Err(_) => {
            // Create simulated contexts for demo
            let mut collection = jessy::memory::ContextCollection::new();
            for path in &nav_result.paths {
                for layer_id in &path.layer_sequence {
                    let context = jessy::memory::LoadedContext {
                        layer_id: *layer_id,
                        content: format!("Simulated content for D{:02}", layer_id.dimension.0),
                        frequency: path.frequency,
                        keywords: path.keywords_matched.clone(),
                    };
                    collection.add_context(context);
                }
            }
            collection
        }
    };
    let loading_duration = load_start.elapsed();
    
    // Format for LLM
    let format_start = Instant::now();
    let _llm_input = contexts.format_with_metadata();
    let formatting_duration = format_start.elapsed();
    
    let total_duration = start.elapsed();
    
    Ok(QueryMetrics {
        navigation_duration,
        loading_duration,
        formatting_duration,
        total_duration,
        dimensions_activated: nav_result.paths.len(),
        contexts_loaded: contexts.len(),
        total_bytes: contexts.total_size,
    })
}

/// Metrics for a single query
#[derive(Debug)]
struct QueryMetrics {
    navigation_duration: Duration,
    loading_duration: Duration,
    formatting_duration: Duration,
    total_duration: Duration,
    dimensions_activated: usize,
    contexts_loaded: usize,
    total_bytes: usize,
}

/// Show memory statistics
fn show_memory_stats(memory_manager: &Arc<MmapManager>) {
    println!("💾 Memory Statistics:");
    println!("-" .repeat(70));
    
    let stats = memory_manager.get_stats();
    
    println!("  Usage:");
    println!("    - Allocated: {} MB / {} MB ({:.1}%)",
        stats.current_allocated_mb,
        stats.total_limit_mb,
        stats.utilization_percent
    );
    println!("    - Available: {} MB", stats.available_mb());
    println!();
    
    println!("  Loaded:");
    println!("    - Regions: {}", stats.regions_loaded);
    println!("    - Layers: {}", stats.layers_indexed);
    println!();
    
    println!("  Operations:");
    println!("    - Allocations: {}", stats.total_allocations);
    println!("    - Deallocations: {}", stats.total_deallocations);
    println!("    - Failures: {}", stats.allocation_failure_count);
    println!("    - Peak concurrent readers: {}", stats.peak_concurrent_readers);
    println!();
    
    if stats.is_near_capacity() {
        println!("  ⚠️  Warning: Near capacity!");
    } else {
        println!("  ✅ Healthy");
    }
}
