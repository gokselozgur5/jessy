//! LLM Configuration

use std::env;

/// LLM Configuration
#[derive(Debug, Clone)]
pub struct LLMConfig {
    /// Provider name ("openai" or "anthropic")
    pub provider: String,
    
    /// Model name (e.g., "gpt-4", "claude-3-5-sonnet")
    pub model: String,
    
    /// API key
    pub api_key: String,
    
    /// Timeout in seconds
    pub timeout_secs: u64,
    
    /// Maximum retry attempts
    pub max_retries: u32,
}

impl LLMConfig {
    /// Load configuration from environment variables
    ///
    /// Environment variables:
    /// - `LLM_PROVIDER`: Provider name (default: "openai")
    /// - `LLM_MODEL`: Model name (default: "gpt-4-turbo")
    /// - `OPENAI_API_KEY`: OpenAI API key
    /// - `ANTHROPIC_API_KEY`: Anthropic API key
    /// - `LLM_TIMEOUT_SECS`: Timeout in seconds (default: 30)
    /// - `LLM_MAX_RETRIES`: Max retries (default: 3)
    pub fn from_env() -> crate::Result<Self> {
        let provider = env::var("LLM_PROVIDER").unwrap_or_else(|_| "openai".to_string());
        let model = env::var("LLM_MODEL").unwrap_or_else(|_| {
            match provider.as_str() {
                "openai" => "gpt-4-turbo".to_string(),
                "anthropic" => "claude-3-5-sonnet-20241022".to_string(),
                _ => "gpt-4-turbo".to_string(),
            }
        });
        
        let api_key = match provider.as_str() {
            "openai" => env::var("OPENAI_API_KEY")
                .map_err(|_| crate::ConsciousnessError::LearningError(
                    "OPENAI_API_KEY environment variable not set".to_string()
                ))?,
            "anthropic" => env::var("ANTHROPIC_API_KEY")
                .map_err(|_| crate::ConsciousnessError::LearningError(
                    "ANTHROPIC_API_KEY environment variable not set".to_string()
                ))?,
            _ => return Err(crate::ConsciousnessError::LearningError(
                format!("Invalid provider: {}", provider)
            )),
        };
        
        let timeout_secs = env::var("LLM_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(30);
        
        let max_retries = env::var("LLM_MAX_RETRIES")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(3);
        
        Ok(Self {
            provider,
            model,
            api_key,
            timeout_secs,
            max_retries,
        })
    }
    
    /// Validate configuration
    pub fn validate(&self) -> crate::Result<()> {
        if self.api_key.is_empty() {
            return Err(crate::ConsciousnessError::LearningError(
                "API key is empty".to_string()
            ));
        }
        
        if self.timeout_secs == 0 || self.timeout_secs > 300 {
            return Err(crate::ConsciousnessError::LearningError(
                "Timeout must be between 1 and 300 seconds".to_string()
            ));
        }
        
        if self.max_retries > 10 {
            return Err(crate::ConsciousnessError::LearningError(
                "Max retries must be <= 10".to_string()
            ));
        }
        
        Ok(())
    }
}

impl Default for LLMConfig {
    fn default() -> Self {
        Self {
            provider: "openai".to_string(),
            model: "gpt-4-turbo".to_string(),
            api_key: String::new(),
            timeout_secs: 30,
            max_retries: 3,
        }
    }
}
