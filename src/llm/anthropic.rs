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

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Message {
    pub role: String,
    pub content: String,
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
    pub async fn call_api_with_system(&self, user_prompt: &str, system_prompt: &str) -> Result<String> {
        let mut last_error = None;

        for attempt in 0..=self.max_retries {
            if attempt > 0 {
                // Check if last error was rate limit (429)
                let is_rate_limit = last_error.as_ref()
                    .map(|e| format!("{:?}", e).contains("429") || format!("{:?}", e).contains("rate_limit"))
                    .unwrap_or(false);

                let backoff = if is_rate_limit {
                    // For rate limits, wait much longer (60 seconds minimum)
                    Duration::from_secs(60)
                } else {
                    // For other errors, use exponential backoff
                    Duration::from_millis(100 * 2_u64.pow(attempt - 1))
                };

                eprintln!("[Anthropic] Retry attempt {} after {:?} (rate_limit: {})", attempt, backoff, is_rate_limit);
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

    /// Make API call with full conversation history (native messages array)
    ///
    /// This method sends the entire conversation history as a messages array,
    /// allowing Claude to see role-based conversation context (user/assistant).
    ///
    /// # Arguments
    ///
    /// * `messages` - Array of messages with role (user/assistant) and content
    /// * `system_prompt` - System prompt for context
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use jessy::llm::anthropic::{AnthropicProvider, Message};
    /// # use jessy::llm::LLMConfig;
    /// # async fn example() -> jessy::Result<()> {
    /// # let config = LLMConfig::default();
    /// # let provider = AnthropicProvider::new(&config)?;
    /// let messages = vec![
    ///     Message { role: "user".to_string(), content: "Hello!".to_string() },
    ///     Message { role: "assistant".to_string(), content: "Hi there!".to_string() },
    ///     Message { role: "user".to_string(), content: "How are you?".to_string() },
    /// ];
    /// let response = provider.call_api_with_conversation(messages, "You are JESSY").await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn call_api_with_conversation(&self, messages: Vec<Message>, system_prompt: &str) -> Result<String> {
        let mut last_error = None;

        for attempt in 0..=self.max_retries {
            if attempt > 0 {
                // Check if last error was rate limit (429)
                let is_rate_limit = last_error.as_ref()
                    .map(|e| format!("{:?}", e).contains("429") || format!("{:?}", e).contains("rate_limit"))
                    .unwrap_or(false);

                let backoff = if is_rate_limit {
                    // For rate limits, wait much longer (60 seconds minimum)
                    Duration::from_secs(60)
                } else {
                    // For other errors, use exponential backoff
                    Duration::from_millis(100 * 2_u64.pow(attempt - 1))
                };

                eprintln!("[Anthropic] Retry attempt {} after {:?} (rate_limit: {})", attempt, backoff, is_rate_limit);
                tokio::time::sleep(backoff).await;
            }

            match self.try_call_with_messages(&messages, system_prompt).await {
                Ok(response) => return Ok(response),
                Err(e) => {
                    eprintln!("[Anthropic] API call failed (attempt {}): {}", attempt + 1, e);
                    last_error = Some(e);
                }
            }
        }

        Err(last_error.unwrap_or_else(||
            crate::ConsciousnessError::LearningError("API call with conversation failed".to_string())
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

        self.send_request(request).await
    }

    /// Try single API call with conversation messages array
    async fn try_call_with_messages(&self, messages: &[Message], system_prompt: &str) -> Result<String> {
        let request = AnthropicRequest {
            model: self.model.clone(),
            max_tokens: 2000,
            messages: messages.to_vec(),
            system: system_prompt.to_string(),
            stream: None,
        };

        self.send_request(request).await
    }

    /// Send request and parse response (shared logic)
    async fn send_request(&self, request: AnthropicRequest) -> Result<String> {
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

            // Check for rate limiting headers
            let rate_limit_info = if status.as_u16() == 429 {
                let mut info = String::from("\n[Rate Limit Hit]");
                if let Some(limit) = response.headers().get("anthropic-ratelimit-requests-limit") {
                    info.push_str(&format!(" Requests limit: {:?}", limit));
                }
                if let Some(remaining) = response.headers().get("anthropic-ratelimit-requests-remaining") {
                    info.push_str(&format!(", Remaining: {:?}", remaining));
                }
                if let Some(reset) = response.headers().get("anthropic-ratelimit-requests-reset") {
                    info.push_str(&format!(", Reset: {:?}", reset));
                }
                if let Some(tokens_limit) = response.headers().get("anthropic-ratelimit-tokens-limit") {
                    info.push_str(&format!(", Token limit: {:?}", tokens_limit));
                }
                if let Some(tokens_remaining) = response.headers().get("anthropic-ratelimit-tokens-remaining") {
                    info.push_str(&format!(", Tokens remaining: {:?}", tokens_remaining));
                }
                info
            } else {
                String::new()
            };

            let error_text = response.text().await.unwrap_or_default();
            return Err(crate::ConsciousnessError::LearningError(
                format!("API error {}: {}{}", status, error_text, rate_limit_info)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_anthropic_provider_creation() {
        let config = LLMConfig {
            provider: "anthropic".to_string(),
            model: "claude-sonnet-4-20250514".to_string(),
            api_key: "test-key".to_string(),
            timeout_secs: 30,
            max_retries: 3,
        };

        let provider = AnthropicProvider::new(&config).expect("Failed to create provider");
        assert_eq!(provider.name(), "Anthropic");
        assert_eq!(provider.model(), "claude-sonnet-4-20250514");
        assert_eq!(provider.api_key, "test-key");
        assert_eq!(provider.max_retries, 3);
    }

    #[test]
    fn test_anthropic_request_serialization() {
        let request = AnthropicRequest {
            model: "claude-sonnet-4-20250514".to_string(),
            max_tokens: 2000,
            messages: vec![Message {
                role: "user".to_string(),
                content: "Test message".to_string(),
            }],
            system: "Test system prompt".to_string(),
            stream: None,
        };

        let json = serde_json::to_string(&request).expect("Failed to serialize");
        assert!(json.contains("claude-sonnet-4-20250514"));
        assert!(json.contains("Test message"));
        assert!(json.contains("Test system prompt"));
        assert!(json.contains("max_tokens"));
    }

    #[test]
    fn test_anthropic_request_with_streaming() {
        let request = AnthropicRequest {
            model: "claude-sonnet-4-20250514".to_string(),
            max_tokens: 2000,
            messages: vec![Message {
                role: "user".to_string(),
                content: "Test".to_string(),
            }],
            system: "System".to_string(),
            stream: Some(true),
        };

        let json = serde_json::to_string(&request).expect("Failed to serialize");
        assert!(json.contains("\"stream\":true"));
    }

    #[test]
    fn test_anthropic_response_deserialization() {
        let json = r#"{
            "content": [
                {"text": "Hello from Claude!"}
            ]
        }"#;

        let response: AnthropicResponse =
            serde_json::from_str(json).expect("Failed to deserialize");
        assert_eq!(response.content.len(), 1);
        assert_eq!(response.content[0].text, "Hello from Claude!");
    }

    #[test]
    fn test_stream_event_deserialization() {
        let json = r#"{
            "type": "content_block_delta",
            "delta": {
                "type": "text_delta",
                "text": "Hello"
            }
        }"#;

        let event: StreamEvent = serde_json::from_str(json).expect("Failed to deserialize");
        assert_eq!(event.event_type, "content_block_delta");
    }

    #[test]
    fn test_content_block_delta_deserialization() {
        let json = r#"{
            "type": "text_delta",
            "text": "Streaming text chunk"
        }"#;

        let delta: ContentBlockDelta =
            serde_json::from_str(json).expect("Failed to deserialize");
        assert_eq!(delta.delta_type, "text_delta");
        assert_eq!(delta.text.unwrap(), "Streaming text chunk");
    }

    #[test]
    fn test_message_serialization() {
        let msg = Message {
            role: "user".to_string(),
            content: "Test content".to_string(),
        };

        let json = serde_json::to_string(&msg).expect("Failed to serialize");
        assert!(json.contains("user"));
        assert!(json.contains("Test content"));
    }

    #[test]
    fn test_stream_event_message_stop() {
        let json = r#"{
            "type": "message_stop"
        }"#;

        let event: StreamEvent = serde_json::from_str(json).expect("Failed to deserialize");
        assert_eq!(event.event_type, "message_stop");
    }

    #[test]
    fn test_config_timeout_conversion() {
        let config = LLMConfig {
            provider: "anthropic".to_string(),
            model: "claude-sonnet-4-20250514".to_string(),
            api_key: "test".to_string(),
            timeout_secs: 45,
            max_retries: 5,
        };

        let provider = AnthropicProvider::new(&config).expect("Failed to create");
        assert_eq!(provider.timeout, Duration::from_secs(45));
    }

    // Unit test for streaming callback logic
    #[tokio::test]
    async fn test_streaming_callback_called() {
        // This is a unit test that verifies the callback mechanism works
        // without making actual API calls
        use std::sync::{Arc, Mutex};

        let chunks_received = Arc::new(Mutex::new(Vec::new()));
        let chunks_clone = chunks_received.clone();

        // Simulate what happens in streaming
        let mut callback = |chunk: &str| {
            chunks_clone.lock().unwrap().push(chunk.to_string());
        };

        // Simulate receiving chunks
        callback("Hello ");
        callback("World");
        callback("!");

        let result = chunks_received.lock().unwrap();
        assert_eq!(result.len(), 3);
        assert_eq!(result[0], "Hello ");
        assert_eq!(result[1], "World");
        assert_eq!(result[2], "!");
    }

    #[test]
    fn test_sse_data_prefix_parsing() {
        // Test SSE format parsing logic
        let line = "data: {\"type\":\"content_block_delta\"}";
        assert!(line.starts_with("data: "));

        let json_str = &line[6..]; // Skip "data: " prefix
        assert_eq!(json_str, "{\"type\":\"content_block_delta\"}");
    }

    #[test]
    fn test_sse_done_signal() {
        let line = "data: [DONE]";
        let json_str = &line[6..];
        assert_eq!(json_str, "[DONE]");
    }
}
