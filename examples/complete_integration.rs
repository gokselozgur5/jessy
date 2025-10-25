//! Complete Integration Example: Navigation → Memory → LLM
//!
//! This example demonstrates the full pipeline:
//! 1. User query → Navigation System
//! 2. Navigation result → Memory Manager
//! 3. Loaded contexts → LLM formatting
//! 4. Response generation (simulated)
//!
//! Run with: cargo run --example complete_integration

use jessy::navigation::{NavigationSystem, DimensionRegistry};
use jessy::memory::{MmapManager, ContextCollection};
use jessy::{DimensionId, Result};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();
    
    println!("🎯 Complete Integration Example");
    println!("=" .repeat(70));
    println!();
    
    // Initialize systems
    let (nav_system, memory_manager) = initialize_systems().await?;
    
    // Process example queries
    let queries = vec![
        "I feel anxious about implementing algorithms",
        "What is the meaning of consciousness?",
        "How do I optimize database queries?",
        "I'm worried about climate change and want to help",
    ];
    
    for (i, query) in queries.iter().enumerate() {
        println!("Query {}: {}", i + 1, query);
        println!("-" .repeat(70));
        
        match process_query(query, &nav_system, &memory_manager).await {
            Ok(response) => {
                println!("✅ Success!");
                println!("{}", response);
            }
            Err(e) => {
                println!("❌ Error: {}", e);
            }
        }
        
        println!();
        println!();
    }
    
    // Show final statistics
    show_statistics(&memory_manager);
    
    Ok(())
}

/// Initialize navigation and memory systems
async fn initialize_systems() -> Result<(NavigationSystem, Arc<MmapManager>)> {
    println!("🔧 Initializing Systems...");
    
    // Load dimension registry
    let config_data = std::fs::read_to_string("data/dimensions.json")
        .unwrap_or_else(|_| {
            // Fallback to minimal config for demo
            r#"{"dimensions": []}"#.to_string()
        });
    
    let registry = Arc::new(DimensionRegistry::load_dimensions(&config_data)?);
    println!("  ✓ Loaded {} dimensions", registry.dimension_count());
    
    // Create navigation system
    let nav_system = NavigationSystem::new(registry)?;
    println!("  ✓ Navigation system ready");
    
    // Create memory manager (280MB limit)
    let memory_manager = Arc::new(MmapManager::new(280)?);
    println!("  ✓ Memory manager ready (280 MB limit)");
    
    // Pre-load core dimensions (optional optimization)
    // memory_manager.initialize_core_dimensions().await?;
    
    println!();
    Ok((nav_system, memory_manager))
}

/// Process a single query through the complete pipeline
async fn process_query(
    query: &str,
    nav_system: &NavigationSystem,
    memory_manager: &Arc<MmapManager>,
) -> Result<String> {
    // Step 1: Navigate to find relevant dimensions
    println!("  🧭 Step 1: Navigation");
    let nav_result = nav_system.navigate(query).await?;
    
    println!("    - Analyzed query type: {:?}", nav_result.query_analysis.question_type);
    println!("    - Estimated frequency: {:.2} Hz", nav_result.query_analysis.frequency.hz());
    println!("    - Activated {} dimensions", nav_result.paths.len());
    println!("    - Total confidence: {:.2}", nav_result.total_confidence);
    
    if nav_result.return_to_source_triggered {
        println!("    ⚠️  Return-to-source triggered (complexity > 6)");
    }
    
    // Step 2: Load contexts from memory
    println!("  💾 Step 2: Memory Loading");
    
    let contexts = match memory_manager.load_contexts(&nav_result.paths) {
        Ok(ctx) => {
            println!("    - Loaded {} contexts", ctx.len());
            println!("    - Total content: {} bytes", ctx.total_size);
            ctx
        }
        Err(e) => {
            println!("    ⚠️  Could not load from files: {}", e);
            println!("    - Creating simulated contexts for demo");
            create_simulated_contexts(&nav_result.paths)
        }
    };
    
    // Step 3: Format for LLM
    println!("  🤖 Step 3: LLM Formatting");
    let llm_input = contexts.format_with_metadata();
    println!("    - Formatted {} bytes for LLM", llm_input.len());
    
    // Step 4: Generate response (simulated)
    println!("  ✨ Step 4: Response Generation");
    let response = simulate_llm_response(query, &nav_result, &contexts);
    println!("    - Generated {} byte response", response.len());
    
    Ok(response)
}

/// Create simulated contexts for demo when files don't exist
fn create_simulated_contexts(paths: &[jessy::navigation::NavigationPath]) -> ContextCollection {
    let mut collection = ContextCollection::new();
    
    for path in paths {
        for layer_id in &path.layer_sequence {
            let context = jessy::memory::LoadedContext {
                layer_id: *layer_id,
                content: format!(
                    "Simulated content for dimension {} layer {}\n\
                     This would contain actual consciousness layer data.",
                    layer_id.dimension.0,
                    layer_id.layer
                ),
                frequency: path.frequency,
                keywords: path.keywords_matched.clone(),
            };
            collection.add_context(context);
        }
    }
    
    collection
}

/// Simulate LLM response generation
fn simulate_llm_response(
    query: &str,
    nav_result: &jessy::navigation::NavigationResult,
    contexts: &ContextCollection,
) -> String {
    let mut response = String::new();
    
    response.push_str("=== SIMULATED LLM RESPONSE ===\n\n");
    response.push_str(&format!("Query: {}\n\n", query));
    
    response.push_str("Analysis:\n");
    response.push_str(&format!("- Type: {:?}\n", nav_result.query_analysis.question_type));
    response.push_str(&format!("- Urgency: {:?}\n", nav_result.query_analysis.urgency));
    response.push_str(&format!("- Frequency: {:.2} Hz\n", nav_result.query_analysis.frequency.hz()));
    response.push_str(&format!("- Complexity: {:.1}\n\n", nav_result.query_analysis.complexity));
    
    response.push_str("Activated Dimensions:\n");
    for path in &nav_result.paths {
        response.push_str(&format!(
            "- D{:02}: confidence {:.2}, {} layers\n",
            path.dimension_id.0,
            path.confidence,
            path.layer_sequence.len()
        ));
    }
    response.push_str("\n");
    
    response.push_str("Loaded Contexts:\n");
    response.push_str(&format!("- {} contexts loaded\n", contexts.len()));
    response.push_str(&format!("- {} total bytes\n\n", contexts.total_size));
    
    response.push_str("Response:\n");
    response.push_str("Based on the activated dimensional contexts, here's my response...\n");
    response.push_str("(In production, this would be generated by the actual LLM)\n\n");
    
    response.push_str("Performance:\n");
    response.push_str(&format!("- Navigation: {} ms\n", nav_result.total_duration_ms));
    response.push_str(&format!("- Memory loading: <50 ms (estimated)\n"));
    response.push_str(&format!("- Total: ~{} ms\n", nav_result.total_duration_ms + 50));
    
    response
}

/// Show final statistics
fn show_statistics(memory_manager: &Arc<MmapManager>) {
    println!("📊 Final Statistics");
    println!("=" .repeat(70));
    
    let stats = memory_manager.get_stats();
    
    println!("Memory Usage:");
    println!("  - Total limit: {} MB", stats.total_limit_mb);
    println!("  - Currently allocated: {} MB", stats.current_allocated_mb);
    println!("  - Available: {} MB", stats.available_mb());
    println!("  - Utilization: {:.1}%", stats.utilization_percent);
    println!();
    
    println!("Loaded Data:");
    println!("  - Regions loaded: {}", stats.regions_loaded);
    println!("  - Layers indexed: {}", stats.layers_indexed);
    println!();
    
    println!("Operations:");
    println!("  - Total allocations: {}", stats.total_allocations);
    println!("  - Total deallocations: {}", stats.total_deallocations);
    println!("  - Allocation failures: {}", stats.allocation_failure_count);
    println!();
    
    if stats.is_near_capacity() {
        println!("⚠️  Warning: Memory usage is near capacity!");
    } else {
        println!("✅ Memory usage is healthy");
    }
}
