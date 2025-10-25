//! Consciousness Orchestrator Demo
//!
//! This example demonstrates the complete consciousness pipeline:
//! 1. Navigation - Select dimensional paths
//! 2. Memory - Load contexts from dimensions
//! 3. Interference - Calculate frequency patterns
//! 4. Iteration - Deep thinking with convergence
//!
//! Run with: cargo run --example consciousness_demo

use jessy::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
use jessy::memory::MmapManager;
use jessy::navigation::NavigationSystem;
use std::sync::Arc;

#[tokio::main]
async fn main() -> jessy::Result<()> {
    println!("=== Consciousness Orchestrator Demo ===\n");
    
    // Initialize systems
    println!("Initializing systems...");
    let navigation = Arc::new(NavigationSystem::new()?);
    let memory = Arc::new(MmapManager::new(280)?);
    println!("✓ Navigation and Memory systems initialized\n");
    
    // Create orchestrator with custom config
    let config = ConsciousnessConfig {
        max_iterations: 9,
        convergence_threshold: 0.95,
        include_metadata: true,
        include_iteration_history: true, // Enable to see iteration steps
    };
    
    let orchestrator = ConsciousnessOrchestrator::with_config(
        navigation.clone(),
        memory.clone(),
        config,
    );
    println!("✓ Consciousness orchestrator created\n");
    
    // Example queries
    let queries = vec![
        "What is empathy?",
        "How does consciousness emerge?",
        "What is the nature of reality?",
    ];
    
    for (i, query) in queries.iter().enumerate() {
        println!("--- Query {} ---", i + 1);
        println!("Q: {}\n", query);
        
        // Process query through consciousness pipeline
        match orchestrator.process(query).await {
            Ok(response) => {
                println!("A: {}\n", response.final_response);
                
                // Display metadata
                println!("Metadata:");
                println!("  Dimensions activated: {:?}", response.metadata.dimensions_activated);
                println!("  Contexts loaded: {}", response.metadata.contexts_loaded);
                println!("  Navigation confidence: {:.2}", response.metadata.navigation_confidence);
                println!("  Iterations completed: {}", response.metadata.iterations_completed);
                println!("  Converged: {}", response.metadata.converged);
                println!("\nTiming:");
                println!("  Navigation: {}ms", response.metadata.navigation_duration_ms);
                println!("  Memory: {}ms", response.metadata.memory_duration_ms);
                println!("  Iteration: {}ms", response.metadata.iteration_duration_ms);
                println!("  Total: {}ms", response.metadata.total_duration_ms);
                
                // Performance check
                if response.metadata.meets_performance_targets() {
                    println!("  ✓ Performance targets met");
                } else {
                    println!("  ⚠ Performance targets not met");
                }
                
                // Show iteration history if enabled
                if !response.iterations.is_empty() {
                    println!("\nIteration History:");
                    for (idx, step) in response.iterations.iter().enumerate() {
                        println!("  Iteration {}: {:?} - {:.2} Hz (confidence: {:.2})",
                                idx + 1,
                                step.phase,
                                step.frequency.hz(),
                                step.confidence);
                        if let Some(sim) = step.similarity_to_previous {
                            println!("    Similarity to previous: {:.2}%", sim * 100.0);
                        }
                    }
                }
            }
            Err(e) => {
                println!("Error: {}\n", e);
            }
        }
        
        println!("\n{}\n", "=".repeat(60));
    }
    
    println!("Demo complete!");
    
    Ok(())
}
