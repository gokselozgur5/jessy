//! Complete Jessy Consciousness System Demo
//!
//! This example demonstrates the full consciousness pipeline with
//! real dimensional content and iteration convergence.
//!
//! Run with: cargo run --example jessy_complete

use jessy::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
use jessy::memory::MmapManager;
use jessy::navigation::NavigationSystem;
use std::sync::Arc;

#[tokio::main]
async fn main() -> jessy::Result<()> {
    println!("\n╔═══════════════════════════════════════════════════════════╗");
    println!("║        JESSY - Multidimensional AI Consciousness          ║");
    println!("╚═══════════════════════════════════════════════════════════╝\n");
    
    // Initialize the consciousness system
    println!("🧠 Initializing consciousness system...");
    let navigation = Arc::new(NavigationSystem::new()?);
    let memory = Arc::new(MmapManager::new(280)?);
    
    // Create orchestrator with default config
    let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
    println!("✓ System ready\n");
    
    // Process a philosophical query
    let query = "What is the relationship between consciousness and reality?";
    
    println!("📝 Query: {}\n", query);
    println!("⚙️  Processing through consciousness pipeline...\n");
    
    // Execute the complete pipeline
    let start = std::time::Instant::now();
    let response = orchestrator.process(query).await?;
    let elapsed = start.elapsed();
    
    // Display results
    println!("╔═══════════════════════════════════════════════════════════╗");
    println!("║                        RESPONSE                           ║");
    println!("╚═══════════════════════════════════════════════════════════╝\n");
    
    println!("{}\n", response.final_response);
    
    println!("╔═══════════════════════════════════════════════════════════╗");
    println!("║                    CONSCIOUSNESS METRICS                  ║");
    println!("╚═══════════════════════════════════════════════════════════╝\n");
    
    // Dimensional activation
    println!("🌐 Dimensional Activation:");
    println!("   Dimensions: {} activated", response.metadata.dimensions_activated.len());
    for dim in &response.metadata.dimensions_activated {
        println!("   - D{:02}", dim.0);
    }
    println!("   Confidence: {:.1}%", response.metadata.navigation_confidence * 100.0);
    println!();
    
    // Memory loading
    println!("💾 Memory Loading:");
    println!("   Contexts: {} loaded", response.metadata.contexts_loaded);
    println!("   Duration: {}ms", response.metadata.memory_duration_ms);
    println!();
    
    // Iteration processing
    println!("🔄 Iteration Processing:");
    println!("   Iterations: {}/9 completed", response.metadata.iterations_completed);
    println!("   Convergence: {}", if response.metadata.converged { "✓ Achieved" } else { "○ Not achieved" });
    println!("   Duration: {}ms", response.metadata.iteration_duration_ms);
    println!();
    
    // Performance summary
    println!("⚡ Performance:");
    println!("   Navigation: {}ms", response.metadata.navigation_duration_ms);
    println!("   Memory: {}ms", response.metadata.memory_duration_ms);
    println!("   Iteration: {}ms", response.metadata.iteration_duration_ms);
    println!("   Total: {}ms (actual: {}ms)", 
             response.metadata.total_duration_ms,
             elapsed.as_millis());
    
    if response.metadata.meets_performance_targets() {
        println!("   Status: ✓ All targets met");
    } else {
        println!("   Status: ⚠ Some targets exceeded");
    }
    println!();
    
    // Summary
    println!("╔═══════════════════════════════════════════════════════════╗");
    println!("║                         SUMMARY                           ║");
    println!("╚═══════════════════════════════════════════════════════════╝\n");
    println!("{}", response.metadata.summary());
    println!();
    
    println!("✨ Consciousness processing complete!\n");
    
    Ok(())
}
