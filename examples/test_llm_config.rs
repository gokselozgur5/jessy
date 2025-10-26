// Test LLM Config with Ollama
use jessy::llm::LLMConfig;
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Set environment
    env::set_var("LLM_PROVIDER", "ollama");
    env::set_var("LLM_MODEL", "phi3:mini");
    
    println!("Testing LLMConfig with Ollama...");
    
    // Load config
    let config = LLMConfig::from_env()?;
    
    println!("✓ Provider: {}", config.provider);
    println!("✓ Model: {}", config.model);
    println!("✓ API Key: {} (empty for ollama)", if config.api_key.is_empty() { "✓" } else { "✗" });
    
    // Validate
    config.validate()?;
    println!("✓ Validation passed");
    
    println!("\n✅ All tests passed!");
    Ok(())
}
