//! Configuration Management
//!
//! Loads system configuration from environment variables with validation.
//! Handles LLM API keys, system limits, and operational parameters.

use std::env;
use crate::Result;
use crate::ConsciousnessError;

/// System configuration loaded from environment variables
///
/// All configuration is loaded at startup from environment variables.
/// Missing required variables or invalid values result in initialization errors.
#[derive(Debug, Clone)]
pub struct SystemConfig {
    /// LLM configuration
    pub llm: LLMConfig,
    
    /// System limits
    pub limits: SystemLimits,
    
    /// Operational parameters
    pub operation: OperationConfig,
}

/// LLM provider configuration
#[derive(Debug, Clone)]
pub struct LLMConfig {
    /// OpenAI API key (optional)
    pub openai_api_key: Option<String>,
    
    /// Anthropic API key (optional)
    pub anthropic_api_key: Option<String>,
    
    /// Preferred LLM provider (openai, anthropic, or auto)
    pub provider: LLMProvider,
    
    /// Model name to use
    pub model: String,
    
    /// API request timeout in seconds
    pub timeout_secs: u64,
    
    /// Maximum retries for failed API calls
    pub max_retries: u32,
}

/// LLM provider selection
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LLMProvider {
    /// Use OpenAI (GPT-4, GPT-3.5)
    OpenAI,
    
    /// Use Anthropic (Claude)
    Anthropic,
    
    /// Auto-select based on available API keys
    Auto,
}

/// System resource limits
#[derive(Debug, Clone)]
pub struct SystemLimits {
    /// Memory limit in MB
    pub memory_limit_mb: usize,
    
    /// Maximum iterations per query
    pub max_iterations: u32,
    
    /// Query timeout in seconds
    pub query_timeout_secs: u64,
    
    /// Maximum concurrent queries
    pub max_concurrent_queries: usize,
}

/// Operational configuration
#[derive(Debug, Clone)]
pub struct OperationConfig {
    /// Environment (development, production)
    pub environment: Environment,
    
    /// Enable debug logging
    pub debug: bool,
    
    /// Log level (error, warn, info, debug, trace)
    pub log_level: String,
}

/// Runtime environment
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Environment {
    Development,
    Production,
}

impl SystemConfig {
    /// Load configuration from environment variables
    ///
    /// Reads all configuration from environment variables with sensible defaults.
    /// Required variables:
    /// - At least one of: `OPENAI_API_KEY` or `ANTHROPIC_API_KEY`
    ///
    /// Optional variables:
    /// - `LLM_PROVIDER`: openai, anthropic, or auto (default: auto)
    /// - `LLM_MODEL`: model name (default: gpt-4 or claude-3-sonnet)
    /// - `LLM_TIMEOUT_SECS`: API timeout (default: 30)
    /// - `LLM_MAX_RETRIES`: max retries (default: 3)
    /// - `MEMORY_LIMIT_MB`: memory limit (default: 500)
    /// - `MAX_ITERATIONS`: max iterations (default: 9)
    /// - `QUERY_TIMEOUT_SECS`: query timeout (default: 30)
    /// - `MAX_CONCURRENT_QUERIES`: concurrent limit (default: 10)
    /// - `RUST_ENV`: environment (default: development)
    /// - `DEBUG`: enable debug (default: false)
    /// - `RUST_LOG`: log level (default: info)
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemConfig)` if configuration is valid,
    /// or `Err` if required variables are missing or invalid.
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - No API keys provided
    /// - Invalid numeric values
    /// - Invalid enum values
    pub fn from_env() -> Result<Self> {
        // Load LLM configuration
        let openai_api_key = env::var("OPENAI_API_KEY").ok();
        let anthropic_api_key = env::var("ANTHROPIC_API_KEY").ok();
        
        // At least one API key required
        if openai_api_key.is_none() && anthropic_api_key.is_none() {
            return Err(ConsciousnessError::InvalidInput(
                "At least one API key required: OPENAI_API_KEY or ANTHROPIC_API_KEY".to_string()
            ));
        }
        
        // Determine provider
        let provider = match env::var("LLM_PROVIDER").as_deref() {
            Ok("openai") => LLMProvider::OpenAI,
            Ok("anthropic") => LLMProvider::Anthropic,
            Ok("auto") | Err(_) => LLMProvider::Auto,
            Ok(other) => {
                return Err(ConsciousnessError::InvalidInput(
                    format!("Invalid LLM_PROVIDER: {} (must be: openai, anthropic, auto)", other)
                ));
            }
        };
        
        // Validate provider has corresponding API key
        match provider {
            LLMProvider::OpenAI if openai_api_key.is_none() => {
                return Err(ConsciousnessError::InvalidInput(
                    "LLM_PROVIDER=openai but OPENAI_API_KEY not set".to_string()
                ));
            }
            LLMProvider::Anthropic if anthropic_api_key.is_none() => {
                return Err(ConsciousnessError::InvalidInput(
                    "LLM_PROVIDER=anthropic but ANTHROPIC_API_KEY not set".to_string()
                ));
            }
            _ => {}
        }
        
        // Determine default model based on provider
        let default_model = match provider {
            LLMProvider::OpenAI => "gpt-4",
            LLMProvider::Anthropic => "claude-3-sonnet-20240229",
            LLMProvider::Auto => {
                if anthropic_api_key.is_some() {
                    "claude-3-sonnet-20240229"
                } else {
                    "gpt-4"
                }
            }
        };
        
        let model = env::var("LLM_MODEL")
            .unwrap_or_else(|_| default_model.to_string());
        
        let timeout_secs = env::var("LLM_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(30);
        
        let max_retries = env::var("LLM_MAX_RETRIES")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(3);
        
        let llm = LLMConfig {
            openai_api_key,
            anthropic_api_key,
            provider,
            model,
            timeout_secs,
            max_retries,
        };
        
        // Load system limits
        let memory_limit_mb = env::var("MEMORY_LIMIT_MB")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(500);
        
        let max_iterations = env::var("MAX_ITERATIONS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(9);
        
        if max_iterations == 0 || max_iterations > 9 {
            return Err(ConsciousnessError::InvalidInput(
                format!("MAX_ITERATIONS must be 1-9, got {}", max_iterations)
            ));
        }
        
        let query_timeout_secs = env::var("QUERY_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(30);
        
        let max_concurrent_queries = env::var("MAX_CONCURRENT_QUERIES")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(10);
        
        let limits = SystemLimits {
            memory_limit_mb,
            max_iterations,
            query_timeout_secs,
            max_concurrent_queries,
        };
        
        // Load operational config
        let environment = match env::var("RUST_ENV").as_deref() {
            Ok("production") => Environment::Production,
            _ => Environment::Development,
        };
        
        let debug = env::var("DEBUG")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(false);
        
        let log_level = env::var("RUST_LOG")
            .unwrap_or_else(|_| "info".to_string());
        
        let operation = OperationConfig {
            environment,
            debug,
            log_level,
        };
        
        Ok(SystemConfig {
            llm,
            limits,
            operation,
        })
    }
    
    /// Validate configuration
    ///
    /// Checks that all configuration values are valid and consistent.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if valid, or `Err` with validation error.
    pub fn validate(&self) -> Result<()> {
        // Validate API keys format (basic check)
        if let Some(ref key) = self.llm.openai_api_key {
            if key.len() < 20 {
                return Err(ConsciousnessError::InvalidInput(
                    "OPENAI_API_KEY appears invalid (too short)".to_string()
                ));
            }
        }
        
        if let Some(ref key) = self.llm.anthropic_api_key {
            if key.len() < 20 {
                return Err(ConsciousnessError::InvalidInput(
                    "ANTHROPIC_API_KEY appears invalid (too short)".to_string()
                ));
            }
        }
        
        // Validate numeric ranges
        if self.limits.memory_limit_mb < 100 {
            return Err(ConsciousnessError::InvalidInput(
                format!("MEMORY_LIMIT_MB too low: {} (minimum 100)", self.limits.memory_limit_mb)
            ));
        }
        
        if self.limits.query_timeout_secs < 5 {
            return Err(ConsciousnessError::InvalidInput(
                format!("QUERY_TIMEOUT_SECS too low: {} (minimum 5)", self.limits.query_timeout_secs)
            ));
        }
        
        Ok(())
    }
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            llm: LLMConfig {
                openai_api_key: None,
                anthropic_api_key: None,
                provider: LLMProvider::Auto,
                model: "gpt-4".to_string(),
                timeout_secs: 30,
                max_retries: 3,
            },
            limits: SystemLimits {
                memory_limit_mb: 500,
                max_iterations: 9,
                query_timeout_secs: 30,
                max_concurrent_queries: 10,
            },
            operation: OperationConfig {
                environment: Environment::Development,
                debug: false,
                log_level: "info".to_string(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_default_config() {
        let config = SystemConfig::default();
        assert_eq!(config.limits.max_iterations, 9);
        assert_eq!(config.limits.memory_limit_mb, 500);
        assert_eq!(config.llm.provider, LLMProvider::Auto);
    }
    
    #[test]
    fn test_validate_memory_limit() {
        let mut config = SystemConfig::default();
        config.limits.memory_limit_mb = 50;
        assert!(config.validate().is_err());
        
        config.limits.memory_limit_mb = 500;
        assert!(config.validate().is_ok());
    }
    
    #[test]
    fn test_validate_timeout() {
        let mut config = SystemConfig::default();
        config.limits.query_timeout_secs = 2;
        assert!(config.validate().is_err());
        
        config.limits.query_timeout_secs = 30;
        assert!(config.validate().is_ok());
    }
}
