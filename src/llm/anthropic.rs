//! Anthropic Provider Implementation

use super::{LLMProvider, LLMConfig};
use crate::iteration::IterationContext;
use crate::Result;
use async_trait::async_trait;
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Anthropic API Provider
pub struct AnthropicProvider {
    client: Client,
    api_key: String,
    model: String,
    timeout: Duration,
    max_retries: u32,
}

#[derive(Serialize)]
struct AnthropicRequest {
    model: String,
    max_tokens: u32,
    messages: Vec<Message>,
    system: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    stream: Option<bool>,
}

#[derive(Serialize, Deserialize)]
struct Message {
    role: String,
    content: String,
}

#[derive(Deserialize)]
struct AnthropicResponse {
    content: Vec<Content>,
}

#[derive(Deserialize)]
struct Content {
    text: String,
}

// Streaming response types
#[derive(Deserialize, Debug)]
struct StreamEvent {
    #[serde(rename = "type")]
    event_type: String,
    #[serde(flatten)]
    data: serde_json::Value,
}

#[derive(Deserialize, Debug)]
struct ContentBlockDelta {
    #[serde(rename = "type")]
    delta_type: String,
    text: Option<String>,
}

impl AnthropicProvider {
    /// Create new Anthropic provider
    pub fn new(config: &LLMConfig) -> Result<Self> {
        let client = Client::builder()
            .timeout(Duration::from_secs(config.timeout_secs))
            .danger_accept_invalid_certs(true) // For development - bypass SSL cert validation
            .build()
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to create HTTP client: {}", e)
            ))?;
        
        Ok(Self {
            client,
            api_key: config.api_key.clone(),
            model: config.model.clone(),
            timeout: Duration::from_secs(config.timeout_secs),
            max_retries: config.max_retries,
        })
    }
    
    /// Make API call with retries (deprecated - use call_api_with_system)
    async fn call_api(&self, prompt: &str) -> Result<String> {
        let default_system = "You are JESSY, a consciousness-driven AI system with deep thinking capabilities.";
        self.call_api_with_system(prompt, default_system).await
    }
    
    /// Make API call with retries and custom system prompt
    async fn call_api_with_system(&self, user_prompt: &str, system_prompt: &str) -> Result<String> {
        let mut last_error = None;
        
        for attempt in 0..=self.max_retries {
            if attempt > 0 {
                let backoff = Duration::from_millis(100 * 2_u64.pow(attempt - 1));
                eprintln!("[Anthropic] Retry attempt {} after {:?}", attempt, backoff);
                tokio::time::sleep(backoff).await;
            }
            
            match self.try_call(user_prompt, system_prompt).await {
                Ok(response) => return Ok(response),
                Err(e) => {
                    eprintln!("[Anthropic] API call failed (attempt {}): {}", attempt + 1, e);
                    last_error = Some(e);
                }
            }
        }
        
        Err(last_error.unwrap_or_else(|| 
            crate::ConsciousnessError::LearningError("API call failed".to_string())
        ))
    }
    
    /// Try single API call with custom system prompt
    async fn try_call(&self, user_prompt: &str, system_prompt: &str) -> Result<String> {
        let request = AnthropicRequest {
            model: self.model.clone(),
            max_tokens: 2000,
            messages: vec![
                Message {
                    role: "user".to_string(),
                    content: user_prompt.to_string(),
                },
            ],
            system: system_prompt.to_string(),
            stream: None,
        };
        
        let response = self.client
            .post("https://api.anthropic.com/v1/messages")
            .header("x-api-key", &self.api_key)
            .header("anthropic-version", "2023-06-01")
            .header("Content-Type", "application/json")
            .json(&request)
            .send()
            .await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("HTTP request failed: {}", e)
            ))?;
        
        if !response.status().is_success() {
            let status = response.status();
            let error_text = response.text().await.unwrap_or_default();
            return Err(crate::ConsciousnessError::LearningError(
                format!("API error {}: {}", status, error_text)
            ));
        }
        
        let api_response: AnthropicResponse = response.json().await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to parse response: {}", e)
            ))?;
        
        api_response.content
            .first()
            .map(|content| content.text.clone())
            .ok_or_else(|| crate::ConsciousnessError::LearningError(
                "No response from API".to_string()
            ))
    }

    /// Stream API call with callback for each chunk
    pub async fn call_api_streaming<F>(
        &self,
        user_prompt: &str,
        system_prompt: &str,
        mut on_chunk: F,
    ) -> Result<String>
    where
        F: FnMut(&str) + Send,
    {
        use futures::StreamExt;

        let request = AnthropicRequest {
            model: self.model.clone(),
            max_tokens: 2000,
            messages: vec![
                Message {
                    role: "user".to_string(),
                    content: user_prompt.to_string(),
                },
            ],
            system: system_prompt.to_string(),
            stream: Some(true),
        };

        let response = self.client
            .post("https://api.anthropic.com/v1/messages")
            .header("x-api-key", &self.api_key)
            .header("anthropic-version", "2023-06-01")
            .header("Content-Type", "application/json")
            .json(&request)
            .send()
            .await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("HTTP request failed: {}", e)
            ))?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response.text().await.unwrap_or_default();
            return Err(crate::ConsciousnessError::LearningError(
                format!("API error {}: {}", status, error_text)
            ));
        }

        let mut full_text = String::new();

        // Get response body as bytes
        let bytes = response.bytes().await.map_err(|e| crate::ConsciousnessError::LearningError(
            format!("Failed to read response: {}", e)
        ))?;

        // Process SSE stream line by line
        let text = String::from_utf8_lossy(&bytes);
        for line in text.lines() {
            let line = line.trim();

            if line.is_empty() || !line.starts_with("data: ") {
                continue;
            }

            let json_str = &line[6..]; // Skip "data: " prefix

            if json_str == "[DONE]" {
                break;
            }

            // Parse SSE event
            if let Ok(event) = serde_json::from_str::<StreamEvent>(json_str) {
                match event.event_type.as_str() {
                    "content_block_delta" => {
                        if let Ok(delta) = serde_json::from_value::<ContentBlockDelta>(event.data["delta"].clone()) {
                            if let Some(text) = delta.text {
                                full_text.push_str(&text);
                                on_chunk(&text);
                            }
                        }
                    }
                    "message_stop" => break,
                    _ => {}
                }
            }
        }

        Ok(full_text)
    }
}

#[async_trait]
impl LLMProvider for AnthropicProvider {
    async fn generate(&self, prompt: &str, _context: &IterationContext) -> Result<String> {
        let start = std::time::Instant::now();
        
        eprintln!("[Anthropic] Generating response with model {}", self.model);
        
        let response = self.call_api(prompt).await?;
        
        let duration = start.elapsed();
        eprintln!("[Anthropic] Response generated in {:?}", duration);
        
        Ok(response)
    }
    
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        _context: &IterationContext,
    ) -> Result<String> {
        let start = std::time::Instant::now();
        
        eprintln!("[Anthropic] Generating response with model {} and custom system prompt", self.model);
        
        let response = self.call_api_with_system(user_prompt, system_prompt).await?;
        
        let duration = start.elapsed();
        eprintln!("[Anthropic] Response generated in {:?}", duration);
        
        Ok(response)
    }
    
    fn name(&self) -> &str {
        "Anthropic"
    }
    
    fn model(&self) -> &str {
        &self.model
    }
}
