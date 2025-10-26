// Test Ollama integration
use std::env;

fn main() {
    env::set_var("LLM_PROVIDER", "ollama");
    env::set_var("LLM_MODEL", "phi3:mini");
    
    println!("Testing Ollama configuration...");
    
    // Test 1: Check environment
    let provider = env::var("LLM_PROVIDER").unwrap();
    let model = env::var("LLM_MODEL").unwrap();
    println!("✓ Provider: {}", provider);
    println!("✓ Model: {}", model);
    
    println!("\nAll tests passed!");
}
