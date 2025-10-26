//! LLM Provider Integration
//!
//! This module provides integration with external LLM providers (OpenAI, Anthropic).
//! It handles API calls, retries, timeouts, and error handling.

pub mod openai;
pub mod anthropic;
pub mod config;

use crate::iteration::IterationContext;
use crate::Result;
use async_trait::async_trait;

pub use config::LLMConfig;
pub use openai::OpenAIProvider;
pub use anthropic::AnthropicProvider;

/// Trait for LLM providers
///
/// All LLM providers must implement this trait to be used by the system.
#[async_trait]
pub trait LLMProvider: Send + Sync {
    /// Generate response from prompt
    ///
    /// # Arguments
    ///
    /// * `prompt` - The prompt to send to the LLM
    /// * `context` - Iteration context with query, dimensions, etc.
    ///
    /// # Returns
    ///
    /// Generated text response from the LLM
    async fn generate(&self, prompt: &str, context: &IterationContext) -> Result<String>;
    
    /// Get provider name
    fn name(&self) -> &str;
    
    /// Get model name
    fn model(&self) -> &str;
}

/// LLM Manager
///
/// Manages LLM provider selection and API calls.
pub struct LLMManager {
    provider: Box<dyn LLMProvider>,
    config: LLMConfig,
}

impl LLMManager {
    /// Create new LLM manager
    ///
    /// # Arguments
    ///
    /// * `config` - LLM configuration
    ///
    /// # Returns
    ///
    /// LLM manager with selected provider
    ///
    /// # Errors
    ///
    /// Returns error if provider is invalid or initialization fails
    pub fn new(config: LLMConfig) -> Result<Self> {
        let provider: Box<dyn LLMProvider> = match config.provider.as_str() {
            "openai" => Box::new(OpenAIProvider::new(&config)?),
            "anthropic" => Box::new(AnthropicProvider::new(&config)?),
            _ => return Err(crate::ConsciousnessError::LearningError(
                format!("Invalid LLM provider: {}", config.provider)
            )),
        };
        
        eprintln!("[LLM] Initialized {} provider with model {}", 
                  provider.name(), provider.model());
        
        Ok(Self { provider, config })
    }
    
    /// Generate response
    ///
    /// # Arguments
    ///
    /// * `prompt` - The prompt to send
    /// * `context` - Iteration context
    ///
    /// # Returns
    ///
    /// Generated response text
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
