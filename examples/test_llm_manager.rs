// Test LLM Manager with Ollama
use jessy::llm::{LLMConfig, LLMManager};
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Set environment
    env::set_var("LLM_PROVIDER", "ollama");
    env::set_var("LLM_MODEL", "phi3:mini");
    
    println!("Testing LLMManager with Ollama...");
    
    // Load config
    let config = LLMConfig::from_env()?;
    println!("✓ Config loaded: {} / {}", config.provider, config.model);
    
    // Create manager
    let manager = LLMManager::new(config)?;
    println!("✓ Manager created");
    println!("✓ Provider name: {}", manager.provider_name());
    println!("✓ Model name: {}", manager.model_name());
    
    println!("\n✅ All tests passed!");
    Ok(())
}
