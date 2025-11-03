//! LLM Provider Integration
//!
//! This module provides integration with external LLM providers (OpenAI, Anthropic, Ollama).
//! It handles API calls, retries, timeouts, and error handling.

pub mod openai;
pub mod anthropic;
pub mod ollama;
pub mod config;

use crate::iteration::IterationContext;
use crate::Result;
use async_trait::async_trait;

pub use config::LLMConfig;
pub use openai::OpenAIProvider;
pub use anthropic::{AnthropicProvider, Message};
pub use ollama::OllamaProvider;

/// Trait for LLM providers
///
/// All LLM providers must implement this trait to be used by the system.
#[async_trait]
pub trait LLMProvider: Send + Sync {
    /// Generate response from prompt
    async fn generate(&self, prompt: &str, context: &IterationContext) -> Result<String>;
    
    /// Generate response with custom system prompt
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String>;
    
    /// Get provider name
    fn name(&self) -> &str;
    
    /// Get model name
    fn model(&self) -> &str;
}

/// LLM Manager
pub struct LLMManager {
    provider: Box<dyn LLMProvider>,
    config: LLMConfig,
}

impl LLMManager {
    /// Generate response with custom system prompt
    pub async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String> {
        self.provider.generate_with_system_prompt(system_prompt, user_prompt, context).await
    }

    /// Create new LLM manager
    pub fn new(config: LLMConfig) -> Result<Self> {
        let provider: Box<dyn LLMProvider> = match config.provider.as_str() {
            "openai" => Box::new(OpenAIProvider::new(&config)?),
            "anthropic" => Box::new(AnthropicProvider::new(&config)?),
            "ollama" => Box::new(OllamaProvider::new(&config)?),
            _ => return Err(crate::ConsciousnessError::LearningError(
                format!("Invalid LLM provider: {}. Valid: openai, anthropic, ollama", config.provider)
            )),
        };
        
        eprintln!("[LLM] Initialized {} provider with model {}", 
                  provider.name(), provider.model());
        
        Ok(Self { provider, config })
    }
    
    /// Generate response
    pub async fn generate(&self, prompt: &str, context: &IterationContext) -> Result<String> {
        self.provider.generate(prompt, context).await
    }
    
    /// Get provider name
    pub fn provider_name(&self) -> &str {
        self.provider.name()
    }
    
    /// Get model name
    pub fn model_name(&self) -> &str {
        self.provider.model()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_invalid_provider() {
        let config = LLMConfig {
            provider: "invalid".to_string(),
            model: "test".to_string(),
            api_key: "test".to_string(),
            timeout_secs: 30,
            max_retries: 3,
        };
        
        let result = LLMManager::new(config);
        assert!(result.is_err());
    }
}
