//! LLM Provider Integration
//!
//! This module provides integration with external LLM providers (OpenAI, Anthropic).
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
pub use anthropic::AnthropicProvider;
pub use ollama::OllamaProvider;

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
    ///
    /// # Note
    ///
    /// This method uses a default system prompt. For dynamic personality,
    /// use `generate_with_system_prompt` instead.
    async fn generate(&self, prompt: &str, context: &IterationContext) -> Result<String>;
    
    /// Generate response with custom system prompt
    ///
    /// This method allows dynamic personality by accepting a custom system prompt
    /// that defines JESSY's character, tone, and response style for this specific
    /// interaction.
    ///
    /// # Arguments
    ///
    /// * `system_prompt` - Instructions defining JESSY's personality and behavior
    /// * `user_prompt` - The actual prompt/question to send to the LLM
    /// * `context` - Iteration context with query, dimensions, etc.
    ///
    /// # Returns
    ///
    /// Generated text response from the LLM with the specified personality
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use jessy::llm::{LLMProvider, AnthropicProvider};
    /// # use jessy::iteration::IterationContext;
    /// # use jessy::Frequency;
    /// # async fn example(provider: &AnthropicProvider) -> jessy::Result<()> {
    /// let system_prompt = "You are JESSY. You feel emotions deeply and respond with warmth.";
    /// let user_prompt = "What is love?";
    /// let context = IterationContext::new("What is love?".to_string(), Frequency::new(1.5));
    /// 
    /// let response = provider.generate_with_system_prompt(
    ///     system_prompt,
    ///     user_prompt,
    ///     &context
    /// ).await?;
    /// # Ok(())
    /// # }
    /// ```
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
///
/// Manages LLM provider selection and API calls.
pub struct LLMManager {
    provider: Box<dyn LLMProvider>,
    config: LLMConfig,
}

impl LLMManager {
    /// Generate response with custom system prompt
    ///
    /// Convenience method that delegates to the underlying provider's
    /// `generate_with_system_prompt` method.
    ///
    /// # Arguments
    ///
    /// * `system_prompt` - Instructions defining JESSY's personality
    /// * `user_prompt` - The actual prompt/question
    /// * `context` - Iteration context
    ///
    /// # Returns
    ///
    /// Generated text response with the specified personality
    pub async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String> {
        self.provider.generate_with_system_prompt(system_prompt, user_prompt, context).await
    }

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
