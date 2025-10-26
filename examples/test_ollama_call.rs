// Test actual Ollama API call
use jessy::llm::{LLMConfig, LLMManager};
use jessy::iteration::IterationContext;
use jessy::Frequency;
use std::env;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Set environment
    env::set_var("LLM_PROVIDER", "ollama");
    env::set_var("LLM_MODEL", "phi3:mini");
    
    println!("Testing Ollama API call...\n");
    
    // Create manager
    let config = LLMConfig::from_env()?;
    let manager = LLMManager::new(config)?;
    
    // Create context
    let context = IterationContext::new(
        "Merhaba".to_string(),
        Frequency::new(1.5)
    );
    
    // Make API call
    println!("Calling Ollama with phi3:mini...");
    let response = manager.generate("Say hello in Turkish", &context).await?;
    
    println!("\n✅ Response: {}", response);
    
    Ok(())
}
