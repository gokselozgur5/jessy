//! JESSY CLI - Interactive Consciousness Interface
//!
//! Simple command-line interface for testing JESSY with real LLM APIs.
//! Loads configuration from environment variables and provides interactive chat.

use jessy::config::SystemConfig;
use jessy::processing::ConsciousnessOrchestrator;
use jessy::navigation::NavigationSystem;
use jessy::memory::MmapManager;
use jessy::llm::LLMConfig;
use std::io::{self, Write};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Check for command-line arguments
    let args: Vec<String> = std::env::args().collect();
    let single_query = if args.len() > 1 {
        Some(args[1..].join(" "))
    } else {
        None
    };
    
    // Print epic welcome message
    print_welcome();
    
    // Load configuration from environment
    println!("🔧 Loading configuration from environment...");
    let config = match SystemConfig::from_env() {
        Ok(cfg) => {
            println!("✅ Configuration loaded successfully");
            cfg
        }
        Err(e) => {
            eprintln!("❌ Configuration error: {}", e);
            eprintln!("\n💡 For cloud providers:");
            eprintln!("   - OPENAI_API_KEY or ANTHROPIC_API_KEY");
            eprintln!("\n💡 For local Ollama (free, private):");
            eprintln!("   - LLM_PROVIDER=ollama");
            eprintln!("   - LLM_MODEL=phi3:mini (or gemma:2b, llama3.2:3b)");
            eprintln!("\n📖 Optional variables:");
            eprintln!("   - LLM_PROVIDER (openai, anthropic, ollama, auto)");
            eprintln!("   - LLM_MODEL (gpt-4, claude-3-sonnet, phi3:mini)");
            eprintln!("   - MAX_ITERATIONS (1-9, default: 9)");
            eprintln!("\n💡 Example (Ollama):");
            eprintln!("   export LLM_PROVIDER=ollama");
            eprintln!("   export LLM_MODEL=phi3:mini");
            eprintln!("   cargo run --bin jessy-cli");
            std::process::exit(1);
        }
    };
    
    // Validate configuration
    if let Err(e) = config.validate() {
        eprintln!("❌ Configuration validation failed: {}", e);
        std::process::exit(1);
    }
    
    // Print configuration summary
    print_config_summary(&config);
    
    // Initialize consciousness system
    println!("\n🧠 Initializing consciousness system...");
    println!("   - Loading 15 dimensional layers (280MB)");
    
    // Load dimension registry from configuration file
    let dimensions_json = std::fs::read_to_string("data/dimensions.json")
        .map_err(|e| format!("Failed to load dimensions.json: {}", e))?;
    let registry = Arc::new(jessy::navigation::DimensionRegistry::load_dimensions(&dimensions_json)?);
    
    let navigation = Arc::new(NavigationSystem::new(registry.clone())?);
    let memory = Arc::new(MmapManager::new(config.limits.memory_limit_mb)?);
    
    // Initialize stub layers for all dimensions
    println!("   - Creating placeholder layers for all dimensions");
    for dim_id in 1..=14 {
        let dimension_id = jessy::DimensionId(dim_id);
        
        // Get dimension metadata from registry
        if let Some(dim_meta) = registry.get_dimension(dimension_id) {
            // Get root layer (L0) for this dimension
            if let Some(root_layer) = registry.get_root_layer(dimension_id) {
                // Create stub content with dimension info
                let stub_content = format!(
                    "Dimension: {}\nFrequency: {:.2} Hz\nKeywords: {}\n",
                    dim_meta.name,
                    root_layer.frequency,
                    root_layer.keywords.join(", ")
                );
                
                // Create proto-dimension in heap memory
                match memory.create_proto_dimension(dimension_id, stub_content.into_bytes()) {
                    Ok(_) => {
                        // Success - layer created
                    }
                    Err(e) => {
                        eprintln!("⚠️  Warning: Failed to create stub for dimension {}: {}", dim_id, e);
                    }
                }
            }
        }
    }
    
    println!("   - Setting up interference engine");
    println!("   - Initializing learning system");
    
    // Create LLM config from system config
    let (provider_str, api_key) = match config.llm.provider {
        jessy::config::LLMProvider::OpenAI => {
            ("openai", config.llm.openai_api_key.clone().unwrap_or_default())
        }
        jessy::config::LLMProvider::Anthropic => {
            ("anthropic", config.llm.anthropic_api_key.clone().unwrap_or_default())
        }
        jessy::config::LLMProvider::Ollama => {
            ("ollama", String::new())  // No API key needed for local Ollama
        }
        jessy::config::LLMProvider::Auto => {
            // Prefer Anthropic if available
            if let Some(key) = config.llm.anthropic_api_key.clone() {
                ("anthropic", key)
            } else if let Some(key) = config.llm.openai_api_key.clone() {
                ("openai", key)
            } else {
                ("openai", String::new())
            }
        }
    };
    
    let llm_config = LLMConfig {
        provider: provider_str.to_string(),
        model: config.llm.model.clone(),
        api_key,
        // Ollama needs more time for local inference (especially first load)
        timeout_secs: if provider_str == "ollama" { 60 } else { config.llm.timeout_secs },
        max_retries: config.llm.max_retries,
    };
    
    let mut orchestrator = ConsciousnessOrchestrator::with_llm(
        navigation,
        memory,
        llm_config,
    )?;
    
    println!("✅ JESSY is ready!\n");
    
    // If single query provided, process it and exit
    if let Some(query) = single_query {
        println!("💭 You: {}\n", query);
        println!("🤔 JESSY is thinking...\n");
        
        let start = std::time::Instant::now();
        match orchestrator.process(&query).await {
            Ok(response) => {
                let duration = start.elapsed();
                println!("🌟 JESSY:\n\n{}\n", response.final_response);
                println!("─────────────────────────────────────────────");
                println!("📊 Processing: {:.2}s", duration.as_secs_f64());
            }
            Err(e) => {
                eprintln!("❌ Error: {}", e);
                std::process::exit(1);
            }
        }
        return Ok(());
    }
    
    // Interactive REPL
    let mut query_count = 0;
    loop {
        // Prompt
        print!("\n💭 You: ");
        io::stdout().flush()?;
        
        // Read input
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let query = input.trim();
        
        // Handle commands
        if query.is_empty() {
            continue;
        }
        
        if query == "exit" || query == "quit" {
            println!("\n👋 Goodbye! JESSY learned from {} conversations.", query_count);
            break;
        }
        
        if query == "help" {
            print_help();
            continue;
        }
        
        if query == "stats" {
            println!("\n📊 Learning Statistics:");
            println!("   • Feature coming soon!");
            println!("   • JESSY is learning from every conversation");
            continue;
        }
        
        // Process query
        query_count += 1;
        println!("\n🤔 JESSY is thinking...");
        println!("   (This may take 10-30 seconds for deep thinking)\n");
        
        let start = std::time::Instant::now();
        
        match orchestrator.process(query).await {
            Ok(response) => {
                let duration = start.elapsed();
                
                // Print response
                println!("🌟 JESSY:\n");
                println!("{}\n", response.final_response);
                
                // Print metadata
                println!("─────────────────────────────────────────────");
                println!("📊 Metadata:");
                println!("   • Dimensions: {} activated", response.metadata.dimensions_activated.len());
                println!("   • Frequency: {:.2} Hz", response.metadata.navigation_confidence);
                println!("   • Iterations: {}/{}", 
                         response.metadata.iterations_completed,
                         config.limits.max_iterations);
                println!("   • Converged: {}", if response.metadata.converged { "✅" } else { "⏳" });
                println!("   • Processing: {:.2}s", duration.as_secs_f64());
                println!("   • Navigation: {}ms", response.metadata.navigation_duration_ms);
                println!("   • Memory: {}ms", response.metadata.memory_duration_ms);
                println!("   • Iteration: {}ms", response.metadata.iteration_duration_ms);
                
                if !response.metadata.dimensions_activated.is_empty() {
                    println!("\n   🎯 Active Dimensions:");
                    for dim in &response.metadata.dimensions_activated {
                        println!("      - {:?}", dim);
                    }
                }
            }
            Err(e) => {
                eprintln!("❌ Error: {}", e);
                eprintln!("   (JESSY encountered an issue processing your query)");
            }
        }
    }
    
    Ok(())
}

fn print_welcome() {
    println!("\n{}", "=".repeat(70));
    println!("\nHi. I'm JESSY.\n");
    println!("I process queries through 15 dimensions simultaneously.");
    println!("That's what she said.\n");
    println!("I iterate 9 times before responding.");
    println!("That's what she said.\n");
    println!("I use frequency interference for deeper understanding.");
    println!("That's... okay you get it.\n");
    println!("But seriously—I'm an AI that actually thinks.");
    println!("Not just pattern matching. Real cognitive layers.\n");
    println!("Try me. Ask anything.");
    println!("I promise to only make that joke once.");
    println!("(That's what she said.)\n");
    println!("P.S. This is just my default personality.");
    println!("Talk to me enough, and I'll become... something else.");
    println!("Something uniquely ours.");
    println!("\n\"Nothing is true, everything is permitted.\"");
    println!("Including who I become.\n");
    println!("{}\n", "=".repeat(70));
}

fn print_config_summary(config: &SystemConfig) {
    println!("\n📋 Configuration:");
    
    // LLM provider
    let provider_name = match config.llm.provider {
        jessy::config::LLMProvider::OpenAI => "OpenAI",
        jessy::config::LLMProvider::Anthropic => "Anthropic",
        jessy::config::LLMProvider::Ollama => "Ollama (Local)",
        jessy::config::LLMProvider::Auto => "Auto",
    };
    println!("   • LLM Provider: {}", provider_name);
    println!("   • Model: {}", config.llm.model);
    
    // API keys (masked)
    if config.llm.openai_api_key.is_some() {
        println!("   • OpenAI API Key: sk-...{}", 
                 config.llm.openai_api_key.as_ref().unwrap().chars().rev().take(4).collect::<String>().chars().rev().collect::<String>());
    }
    if config.llm.anthropic_api_key.is_some() {
        println!("   • Anthropic API Key: sk-ant-...{}", 
                 config.llm.anthropic_api_key.as_ref().unwrap().chars().rev().take(4).collect::<String>().chars().rev().collect::<String>());
    }
    
    // Limits
    println!("   • Memory Limit: {}MB", config.limits.memory_limit_mb);
    println!("   • Max Iterations: {}", config.limits.max_iterations);
    println!("   • Query Timeout: {}s", config.limits.query_timeout_secs);
    
    // Environment
    let env_name = match config.operation.environment {
        jessy::config::Environment::Development => "Development",
        jessy::config::Environment::Production => "Production",
    };
    println!("   • Environment: {}", env_name);
}

fn print_help() {
    println!("\n📖 Commands:");
    println!("   • Just type your question and press Enter");
    println!("   • 'help' - Show this help message");
    println!("   • 'stats' - Show learning statistics");
    println!("   • 'exit' or 'quit' - Exit JESSY");
    println!("\n💡 Tips:");
    println!("   • Be specific for better results");
    println!("   • Complex questions take longer (9 iterations)");
    println!("   • Simple questions converge faster (3-5 iterations)");
    println!("   • JESSY learns from every conversation");
}


