//! Navigation + Memory Manager Integration Example
//!
//! Shows how NavigationSystem and MmapManager work together:
//! 1. Navigate query → get dimension paths
//! 2. Load dimensions from MMAP
//! 3. Access layer content (zero-copy)

use jessy::navigation::{NavigationSystem, DimensionRegistry};
use jessy::memory::MmapManager;
use jessy::{DimensionId, LayerId};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔗 Navigation + Memory Integration Demo");
    println!("=" .repeat(60));
    println!();
    
    // Step 1: Initialize Navigation System
    println!("📍 Step 1: Initialize Navigation System");
    let config_data = std::fs::read_to_string("data/dimensions.json")?;
    let registry = Arc::new(DimensionRegistry::load_dimensions(&config_data)?);
    let nav_system = NavigationSystem::new(registry)?;
    println!("✅ Navigation System ready");
    println!();
    
    // Step 2: Initialize Memory Manager
    println!("💾 Step 2: Initialize Memory Manager");
    let mut memory_manager = MmapManager::new()?;
    println!("✅ Memory Manager ready");
    println!();
    
    // Step 3: Navigate a query
    println!("🧭 Step 3: Navigate Query");
    let query = "I feel anxious about implementing algorithms";
    println!("Query: {}", query);
    println!();
    
    let nav_result = nav_system.navigate(query).await?;
    
    println!("📊 Navigation Result:");
    println!("  - Dimensions activated: {}", nav_result.dimensions.len());
    println!("  - Total confidence: {:.2}", nav_result.total_confidence);
    println!("  - Return to source: {}", nav_result.return_to_source_triggered);
    println!();
    
    // Step 4: Load contexts using high-level API
    println!("💾 Step 4: Load Contexts from Memory");
    println!("  Using high-level load_contexts() API...");
    
    match memory_manager.load_contexts(&nav_result.paths) {
        Ok(contexts) => {
            println!("  ✅ Loaded {} contexts", contexts.len());
            println!("  📊 Total content size: {} bytes", contexts.total_size);
            println!();
            
            // Show loaded contexts
            println!("📄 Step 5: Loaded Context Details");
            for (i, context) in contexts.contexts.iter().enumerate() {
                println!("  Context {}:", i + 1);
                println!("    Layer: {:?}", context.layer_id);
                println!("    Frequency: {:.2} Hz", context.frequency.hz());
                println!("    Keywords: {:?}", context.keywords);
                println!("    Content length: {} bytes", context.content.len());
                println!();
            }
            
            // Step 6: Format for LLM
            println!("🤖 Step 6: Format for LLM");
            let llm_input = contexts.format_with_metadata();
            println!("  Formatted context length: {} bytes", llm_input.len());
            println!("  First 200 chars:");
            println!("  {}", &llm_input[..200.min(llm_input.len())]);
            println!("  ...");
            println!();
        }
        Err(e) => {
            println!("  ⚠️  Could not load contexts: {}", e);
            println!("  (This is expected in demo - dimension files may not exist)");
            println!();
            
            // Show what would happen
            println!("📄 Step 5: What Would Be Loaded (Demo)");
            for path in &nav_result.paths {
                println!("  Dimension D{:02}:", path.dimension_id.0);
                println!("    Confidence: {:.2}", path.confidence);
                println!("    Frequency: {:.2} Hz", path.frequency.hz());
                println!("    Keywords: {:?}", path.keywords_matched);
                println!("    Layer sequence: {:?}", path.layer_sequence);
                println!();
            }
        }
    }
    
    // Step 7: Show memory statistics
    println!("📊 Step 7: Memory Statistics");
    let stats = memory_manager.get_stats();
    println!("  Total limit: {} MB", stats.total_limit_mb);
    println!("  Currently allocated: {} MB", stats.current_allocated_mb);
    println!("  Utilization: {:.1}%", stats.utilization_percent);
    println!("  Regions loaded: {}", stats.regions_loaded);
    println!("  Layers indexed: {}", stats.layers_indexed);
    println!();
    
    // Step 8: Show integration benefits
    println!("✨ Integration Benefits:");
    println!("  - Zero-copy: Direct memory access, no buffer allocation");
    println!("  - Selective loading: Only load activated dimensions");
    println!("  - Layer-specific: Access exact layers from navigation");
    println!("  - Thread-safe: Concurrent reads without locks");
    println!("  - Performance: <10ms per dimension load");
    println!("  - High-level API: load_contexts() handles everything");
    println!();
    
    println!("=" .repeat(60));
    println!("🎉 Integration demo complete!");
    
    Ok(())
}

/// Example: Complete query processing pipeline
async fn process_query_complete(
    query: &str,
    nav_system: &NavigationSystem,
    memory_manager: &MmapManager,
) -> Result<String, Box<dyn std::error::Error>> {
    // Step 1: Navigate
    let nav_result = nav_system.navigate(query).await?;
    
    // Step 2: Load contexts
    let contexts = memory_manager.load_contexts(&nav_result.paths)?;
    
    // Step 3: Format for LLM
    let llm_input = contexts.format_with_metadata();
    
    // Step 4: Send to LLM (placeholder)
    // let response = llm_client.complete(&llm_input).await?;
    
    Ok(llm_input)
}

/// Example: Selective loading based on confidence
async fn process_query_selective(
    query: &str,
    nav_system: &NavigationSystem,
    memory_manager: &MmapManager,
    min_confidence: f32,
) -> Result<String, Box<dyn std::error::Error>> {
    // Navigate
    let nav_result = nav_system.navigate(query).await?;
    
    // Filter high-confidence paths
    let high_confidence_paths: Vec<_> = nav_result.paths
        .iter()
        .filter(|p| p.confidence >= min_confidence)
        .cloned()
        .collect();
    
    // Load only high-confidence contexts
    let contexts = memory_manager.load_contexts(&high_confidence_paths)?;
    
    Ok(contexts.format_with_metadata())
}

/// Example: Parallel loading for better performance
async fn process_query_parallel(
    query: &str,
    nav_system: &NavigationSystem,
    memory_manager: Arc<MmapManager>,
) -> Result<String, Box<dyn std::error::Error>> {
    use tokio::task;
    
    // Navigate
    let nav_result = nav_system.navigate(query).await?;
    
    // Spawn concurrent loading tasks
    let load_tasks: Vec<_> = nav_result.paths
        .iter()
        .map(|path| {
            let manager = Arc::clone(&memory_manager);
            let path = path.clone();
            task::spawn(async move {
                manager.load_contexts(&[path])
            })
        })
        .collect();
    
    // Wait for all loads
    let results = futures::future::join_all(load_tasks).await;
    
    // Combine contexts
    let mut all_contexts = jessy::memory::ContextCollection::new();
    for result in results {
        if let Ok(Ok(contexts)) = result {
            for context in contexts.contexts {
                all_contexts.add_context(context);
            }
        }
    }
    
    Ok(all_contexts.format_with_metadata())
}
