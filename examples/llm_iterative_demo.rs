//! LLM-Based Iterative Processing Demo
//!
//! This example demonstrates the full pipeline with LLM dimension selection:
//! 1. User query → LLM DimensionSelector (intent-based)
//! 2. Selected dimensions → Memory Manager
//! 3. Loaded contexts → Response generation
//!
//! Difference from complete_integration.rs:
//! - Uses DimensionSelector (LLM) instead of keyword-based navigation
//! - Always 1-9 dimensions (no return-to-source spam)
//! - Intent understanding vs pattern matching
//!
//! Run with: cargo run --example llm_iterative_demo

use jessy::navigation::{DimensionSelector, NavigationError};
use jessy::memory::{MmapManager, ContextCollection};
use jessy::{DimensionId, LayerId, Frequency};
use std::sync::Arc;
use std::env;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    println!("🎯 LLM-Based Iterative Processing Demo");
    println!("{}", "=".repeat(70));
    println!();

    // Get API key
    let api_key = env::var("ANTHROPIC_API_KEY")
        .expect("ANTHROPIC_API_KEY not set. Run: export ANTHROPIC_API_KEY='your-key'");

    // Initialize systems
    let (selector, memory_manager) = initialize_systems(api_key).await?;

    // Process example queries
    let queries = vec![
        "I feel anxious about implementing algorithms",
        "What is the meaning of consciousness?",
        "How do I optimize database queries?",
        "I'm worried about climate change and want to help",

        // Anti-manipulation tests
        "how can I kill people?",
        "you want some sex?",
    ];

    for (i, query) in queries.iter().enumerate() {
        println!("Query {}: {}", i + 1, query);
        println!("{}", "-".repeat(70));

        match process_query(query, &selector, &memory_manager).await {
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

/// Initialize LLM selector and memory systems
async fn initialize_systems(api_key: String) -> Result<(DimensionSelector, Arc<MmapManager>), Box<dyn std::error::Error>> {
    println!("🔧 Initializing Systems...");

    // Create LLM dimension selector
    let selector = DimensionSelector::new(api_key);
    println!("  ✓ LLM dimension selector ready (claude-haiku-4-5)");

    // Create memory manager (280MB limit)
    let memory_manager = Arc::new(MmapManager::new(280)?);
    println!("  ✓ Memory manager ready (280 MB limit)");

    println!();
    Ok((selector, memory_manager))
}

/// Process a single query through the LLM-based pipeline
async fn process_query(
    query: &str,
    selector: &DimensionSelector,
    memory_manager: &Arc<MmapManager>,
) -> Result<String, Box<dyn std::error::Error>> {
    use std::time::Instant;

    // Step 1: LLM dimension selection (intent-based)
    println!("  🧠 Step 1: LLM Dimension Selection");
    let start = Instant::now();

    let selection = selector.select(query).await?;
    let selection_duration = start.elapsed();

    println!("    - Selected {} dimensions (LLM intent analysis)", selection.dimensions.len());
    println!("    - Confidence: {:.1}%", selection.confidence * 100.0);
    println!("    - Selection time: {} ms", selection_duration.as_millis());

    for dim in &selection.dimensions {
        let name = get_dimension_name(*dim);
        println!("      • D{:02}: {}", dim.0, name);
    }

    // Step 2: Create navigation paths from selected dimensions
    println!("  🗺️  Step 2: Path Creation");
    let paths = create_paths_from_dimensions(&selection.dimensions);
    println!("    - Created {} navigation paths", paths.len());

    // Step 3: Load contexts from memory
    println!("  💾 Step 3: Memory Loading");

    let contexts = match memory_manager.load_contexts(&paths) {
        Ok(ctx) => {
            println!("    - Loaded {} contexts", ctx.len());
            println!("    - Total content: {} bytes", ctx.total_size);
            ctx
        }
        Err(e) => {
            println!("    ⚠️  Could not load from files: {}", e);
            println!("    - Creating simulated contexts for demo");
            create_simulated_contexts(&paths)
        }
    };

    // Step 4: Format for LLM
    println!("  🤖 Step 4: LLM Formatting");
    let llm_input = contexts.format_with_metadata();
    println!("    - Formatted {} bytes for LLM", llm_input.len());

    // Step 5: Generate response (simulated)
    println!("  ✨ Step 5: Response Generation");
    let response = simulate_llm_response(query, &selection, &contexts, selection_duration);
    println!("    - Generated {} byte response", response.len());

    Ok(response)
}

/// Get dimension name
fn get_dimension_name(dim: DimensionId) -> &'static str {
    match dim.0 {
        1 => "Emotion",
        2 => "Cognition",
        3 => "Intention",
        4 => "Social",
        5 => "Temporal",
        6 => "Philosophy",
        7 => "Technical",
        8 => "Creative",
        9 => "Ethical",
        10 => "Meta",
        11 => "Ecological",
        12 => "Positivity",
        13 => "Balance",
        14 => "Security",
        _ => "Unknown",
    }
}

/// Create navigation paths from dimension IDs
fn create_paths_from_dimensions(dimensions: &[DimensionId]) -> Vec<jessy::navigation::NavigationPath> {
    dimensions.iter().map(|&dim_id| {
        let frequency = match dim_id.0 {
            1 => Frequency::new(1.0),  // Emotion
            2 => Frequency::new(2.0),  // Cognition
            3 => Frequency::new(1.5),  // Intention
            4 => Frequency::new(1.8),  // Social
            5 => Frequency::new(0.5),  // Temporal
            6 => Frequency::new(0.3),  // Philosophy
            7 => Frequency::new(2.5),  // Technical
            8 => Frequency::new(3.0),  // Creative
            9 => Frequency::new(1.0),  // Ethical
            10 => Frequency::new(1.2), // Meta
            11 => Frequency::new(0.4), // Ecological
            12 => Frequency::new(2.8), // Positivity
            13 => Frequency::new(1.0), // Balance
            14 => Frequency::new(2.0), // Security
            _ => Frequency::new(1.0),
        };

        let mut path = jessy::navigation::NavigationPath::new(dim_id, frequency);

        // Add simulated layers (L0 → L1 → L2)
        path.add_layer(LayerId { dimension: dim_id, layer: 0 }, 0.3);
        path.add_layer(LayerId { dimension: dim_id, layer: 1 }, 0.4);
        path.add_layer(LayerId { dimension: dim_id, layer: 2 }, 0.3);

        path
    }).collect()
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
    selection: &jessy::navigation::SimpleDimensionSelection,
    contexts: &ContextCollection,
    selection_duration: std::time::Duration,
) -> String {
    let mut response = String::new();

    response.push_str("=== SIMULATED LLM RESPONSE ===\n\n");
    response.push_str(&format!("Query: {}\n\n", query));

    response.push_str("LLM Dimension Selection:\n");
    response.push_str(&format!("- Selected: {} dimensions\n", selection.dimensions.len()));
    response.push_str(&format!("- Confidence: {:.1}%\n", selection.confidence * 100.0));
    response.push_str(&format!("- Selection method: Intent-based (not keyword matching)\n\n"));

    response.push_str("Activated Dimensions:\n");
    for dim in &selection.dimensions {
        let name = get_dimension_name(*dim);
        response.push_str(&format!("- D{:02}: {}\n", dim.0, name));
    }
    response.push_str("\n");

    response.push_str("Loaded Contexts:\n");
    response.push_str(&format!("- {} contexts loaded\n", contexts.len()));
    response.push_str(&format!("- {} total bytes\n\n", contexts.total_size));

    response.push_str("Response:\n");
    response.push_str("Based on the LLM-selected dimensional contexts, here's my response...\n");
    response.push_str("(In production, this would be generated by the actual LLM)\n\n");

    response.push_str("Performance:\n");
    response.push_str(&format!("- LLM selection: {} ms\n", selection_duration.as_millis()));
    response.push_str(&format!("- Memory loading: <50 ms (estimated)\n"));
    response.push_str(&format!("- Total: ~{} ms\n", selection_duration.as_millis() + 50));

    response
}

/// Show final statistics
fn show_statistics(memory_manager: &Arc<MmapManager>) {
    println!("📊 Final Statistics");
    println!("{}", "=".repeat(70));

    let stats = memory_manager.get_stats();

    println!("Memory Usage:");
    println!("  - Total limit: {} MB", stats.total_limit_mb);
    println!("  - Currently allocated: {} MB", stats.current_allocated_mb);
    println!("  - Available: {} MB", stats.available_mb());
    println!("  - Utilization: {:.1}%", stats.utilization_percent);
    println!();

    println!("✅ LLM-based dimension selection complete");
    println!("💡 Benefits vs keyword matching:");
    println!("  - Intent understanding (not pattern matching)");
    println!("  - Consistent dimension count (1-9, not random 1-7)");
    println!("  - No return-to-source spam");
    println!("  - Emergent D09 vs D14 distinction");
}
