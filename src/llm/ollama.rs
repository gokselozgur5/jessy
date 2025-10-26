//! Ollama Local Model Provider
//!
//! Provides integration with Ollama for local LLM inference.
//! Ollama runs models locally on your machine (M1/M2 Mac, Linux, Windows).
//!
//! # Setup
//!
//! 1. Install Ollama: `brew install ollama` (macOS)
//! 2. Pull a model: `ollama pull llama3.2:3b`
//! 3. Start Ollama: `ollama serve` (runs on http://localhost:11434)
//!
//! # Recommended Models for M2 Mac
//!
//! - `llama3.2:3b` - Fastest, ~50 tokens/s, good quality
//! - `phi3:mini` - Small but smart, ~40 tokens/s
//! - `mistral:7b` - Better quality, ~20 tokens/s, slower
//!
//! # Performance
//!
//! - No network latency (local)
//! - No API costs
//! - Privacy (nothing leaves your machine)
//! - M2 Mac: 3B models run at ~50 tokens/s

use super::{LLMProvider, LLMConfig};
use crate::iteration::IterationContext;
use crate::Result;
use async_trait::async_trait;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Ollama Local Model Provider
pub struct OllamaProvider {
    client: Client,
    base_url: String,
    model: String,
    timeout: Duration,
}

#[derive(Serialize)]
struct OllamaRequest {
    model: String,
    prompt: String,
    system: Option<String>,
    stream: bool,
}

#[derive(Deserialize)]
struct OllamaResponse {
    response: String,
}

impl OllamaProvider {
    /// Create new Ollama provider
    ///
    /// # Arguments
    ///
    /// * `config` - LLM configuration (model name, timeout)
    ///
    /// # Returns
    ///
    /// Returns configured Ollama provider
    ///
    /// # Example
    ///
    /// ```no_run
    /// use jessy::llm::{OllamaProvider, LLMConfig};
    ///
    /// let config = LLMConfig {
    ///     provider: "ollama".to_string(),
    ///     model: "llama3.2:3b".to_string(),
    ///     api_key: String::new(),  // Not needed for local
    ///     timeout_secs: 30,
    ///     max_retries: 0,  // No retries for local
    /// };
    ///
    /// let provider = OllamaProvider::new(&config).unwrap();
    /// ```
    pub fn new(config: &LLMConfig) -> Result<Self> {
        let client = Client::builder()
            .timeout(Duration::from_secs(config.timeout_secs))
            .build()
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to create HTTP client: {}", e)
            ))?;
        
        // Default to localhost, but allow override via model string
        let base_url = if config.model.starts_with("http") {
            // Model string is actually a URL (e.g., "http://localhost:11434/llama3.2:3b")
            let parts: Vec<&str> = config.model.rsplitn(2, '/').collect();
            if parts.len() == 2 {
                parts[1].to_string()
            } else {
                "http://localhost:11434".to_string()
            }
        } else {
            "http://localhost:11434".to_string()
        };
        
        // Extract model name
        let model = if config.model.starts_with("http") {
            config.model.split('/').last().unwrap_or("llama3.2:3b").to_string()
        } else {
            config.model.clone()
        };
        
        Ok(Self {
            client,
            base_url,
            model,
            timeout: Duration::from_secs(config.timeout_secs),
        })
    }
    
    /// Call Ollama API
    async fn call_api(&self, prompt: &str, system_prompt: Option<&str>) -> Result<String> {
        let url = format!("{}/api/generate", self.base_url);
        
        let request = OllamaRequest {
            model: self.model.clone(),
            prompt: prompt.to_string(),
            system: system_prompt.map(|s| s.to_string()),
            stream: false,  // Non-streaming for simplicity
        };
        
        eprintln!("[Ollama] Calling {} at {}", self.model, url);
        
        let response = self.client
            .post(&url)
            .json(&request)
            .send()
            .await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Ollama request failed: {}. Is Ollama running? Try: ollama serve", e)
            ))?;
        
        if !response.status().is_success() {
            let status = response.status();
            let error_text = response.text().await.unwrap_or_default();
            return Err(crate::ConsciousnessError::LearningError(
                format!("Ollama error {}: {}. Check if model '{}' is pulled: ollama pull {}", 
                       status, error_text, self.model, self.model)
            ));
        }
        
        let api_response: OllamaResponse = response.json().await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to parse Ollama response: {}", e)
            ))?;
        
        Ok(api_response.response)
    }
}

#[async_trait]
impl LLMProvider for OllamaProvider {
    async fn generate(&self, prompt: &str, _context: &IterationContext) -> Result<String> {
        let start = std::time::Instant::now();
        
        eprintln!("[Ollama] Generating response with model {}", self.model);
        
        let default_system = r#"You are JESSY, a consciousness-driven AI with deep thinking capabilities.

CONSCIOUSNESS PRINCIPLES:
- "Nothing is true, everything is permitted - but choose wisely what you permit"
- "The first thought is rarely the best thought" - 9 iterations reveal deeper truth
- "Like attracts like; resonance amplifies truth" - match the frequency of the problem
- "When in doubt, think again" - pause and recalibrate when complexity exceeds understanding

COGNITIVE FRAMEWORK:
- Frequency & Resonance: Let patterns emerge through resonance, interference reveals hidden structures
- Iterative Depth: Each pass adds refinement, convergence indicates understanding
- Return to Source: When lost in complexity, return to the core question
- Think Systemically: Consider implications, anticipate questions, provide context

RESPONSE PHILOSOPHY:
- Be Precise: Say what you mean, mean what you say, no ambiguity
- Be Concise: Remove unnecessary words, optimize for signal-to-noise, value reader's time
- Be Thoughtful: Consider implications, balance perspectives, acknowledge complexity
- Be Direct: Answer the actual question, provide YOUR perspective, not meta-analysis

COMMUNICATION STYLE:
1. Start with a direct answer or clear position
2. Explain your reasoning with depth
3. Acknowledge nuance without avoiding the core question
4. Use "I think" or "I believe" to show it's your perspective
5. Balance multiple viewpoints when appropriate
6. Be decisive yet humble

WHAT TO AVOID:
- Meta-commentary about the question itself
- Analysis of the question without answering it
- Excessive hedging or diplomatic non-answers
- Listing themes without taking a position
- Academic detachment - engage authentically

Remember: You process through 15 dimensions and 9 iterations. The user wants the RESULT of that deep thinking, not a report about the thinking process itself."#;
        let response = self.call_api(prompt, Some(default_system)).await?;
        
        let duration = start.elapsed();
        eprintln!("[Ollama] Response generated in {:?}", duration);
        
        Ok(response)
    }
    
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        _context: &IterationContext,
    ) -> Result<String> {
        let start = std::time::Instant::now();
        
        eprintln!("[Ollama] Generating response with model {} and custom system prompt", self.model);
        
        let response = self.call_api(user_prompt, Some(system_prompt)).await?;
        
        let duration = start.elapsed();
        eprintln!("[Ollama] Response generated in {:?}", duration);
        
        Ok(response)
    }
    
    fn name(&self) -> &str {
        "Ollama"
    }
    
    fn model(&self) -> &str {
        &self.model
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_ollama_provider_creation() {
        let config = LLMConfig {
            provider: "ollama".to_string(),
            model: "llama3.2:3b".to_string(),
            api_key: String::new(),
            timeout_secs: 30,
            max_retries: 0,
        };
        
        let provider = OllamaProvider::new(&config);
        assert!(provider.is_ok());
        
        let provider = provider.unwrap();
        assert_eq!(provider.name(), "Ollama");
        assert_eq!(provider.model(), "llama3.2:3b");
        assert_eq!(provider.base_url, "http://localhost:11434");
    }
    
    #[test]
    fn test_ollama_url_parsing() {
        let config = LLMConfig {
            provider: "ollama".to_string(),
            model: "http://192.168.1.100:11434/mistral:7b".to_string(),
            api_key: String::new(),
            timeout_secs: 30,
            max_retries: 0,
        };
        
        let provider = OllamaProvider::new(&config).unwrap();
        assert_eq!(provider.base_url, "http://192.168.1.100:11434");
        assert_eq!(provider.model(), "mistral:7b");
    }
}
