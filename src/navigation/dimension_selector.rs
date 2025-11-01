//! LLM-based dimension selection
//!
//! Single-purpose micro-service: Analyze query → Select 2-3 dimensions
//!
//! This replaces keyword-based activation with intent-based selection.
//! Uses a small, fast LLM to understand query intent and pick optimal dimensions.

use crate::DimensionId;
use super::NavigationError;
use serde::{Deserialize, Serialize};

/// Dimension selector using LLM for intent-based selection
pub struct DimensionSelector {
    api_key: String,
    model: String,
}

/// Selection result with chosen dimensions
#[derive(Debug, Clone)]
pub struct DimensionSelection {
    /// Selected dimension IDs (2-3 dimensions)
    pub dimensions: Vec<DimensionId>,

    /// LLM reasoning (optional, for debugging)
    pub reasoning: Option<String>,

    /// Selection confidence (0.0-1.0)
    pub confidence: f32,
}

/// LLM response format
#[derive(Debug, Deserialize, Serialize)]
struct SelectionResponse {
    dimensions: Vec<String>,
    #[serde(default)]
    reasoning: Option<String>,
}

const SELECTION_PROMPT: &str = r#"Select 2-3 most relevant dimensions for this query.

Query: {query}

Available dimensions:
- D01: Emotion (empathy, joy, sadness, feelings)
- D02: Cognition (analytical, creative, intuitive, thinking)
- D03: Intention (create, destroy, explore, teach, goals)
- D04: Social (relationships, communication, people)
- D05: Temporal (past, present, future, time)
- D06: Philosophy (meaning, existence, truth, ethics)
- D07: Technical (code, systems, debugging, engineering)
- D08: Creative (art, metaphor, play, imagination)
- D09: Ethical (harm prevention, morality, values)
- D10: Meta (self-awareness, learning, reflection)
- D11: Ecological (nature, sustainability, environment)
- D12: Positivity (hope, constructive, optimism)
- D13: Balance (equilibrium, moderation, harmony)
- D14: Security (boundaries, protection, safety)

Rules:
1. Select ONLY 2-3 dimensions (no more, no less)
2. Choose based on query INTENT, not just keywords
3. Return ONLY a JSON array of dimension IDs

Example responses:
- For "I feel anxious about code": ["D01", "D07", "D09"]
- For "What is consciousness?": ["D02", "D06", "D10"]
- For "How to help climate?": ["D03", "D11", "D12"]

Your response (JSON array only):
"#;

impl DimensionSelector {
    /// Create new dimension selector with API key
    pub fn new(api_key: String) -> Self {
        Self {
            api_key,
            model: "claude-sonnet-4-20241022".to_string(),
        }
    }

    /// Create with custom model
    pub fn with_model(api_key: String, model: String) -> Self {
        Self { api_key, model }
    }

    /// Select dimensions for a query
    ///
    /// Returns 2-3 dimension IDs based on LLM analysis of query intent.
    ///
    /// # Arguments
    ///
    /// * `query` - User query to analyze
    ///
    /// # Returns
    ///
    /// DimensionSelection with 2-3 dimension IDs
    ///
    /// # Errors
    ///
    /// Returns error if LLM call fails or response is invalid
    pub async fn select(&self, query: &str) -> Result<DimensionSelection, NavigationError> {
        // Build prompt
        let prompt = SELECTION_PROMPT.replace("{query}", query);

        // Call Claude API directly (prototype)
        let response = self.call_claude_api(&prompt).await?;

        // Parse response
        let dimensions = self.parse_response(&response)?;

        // Validate: must be 2-3 dimensions
        if dimensions.is_empty() || dimensions.len() > 3 {
            return Err(NavigationError::InvalidDimensionCount {
                count: dimensions.len(),
                expected: "2-3".to_string(),
            }.into());
        }

        Ok(DimensionSelection {
            dimensions,
            reasoning: None,
            confidence: 0.9,
        })
    }

    /// Call Claude API directly (prototype implementation)
    async fn call_claude_api(&self, prompt: &str) -> Result<String, NavigationError> {
        let client = reqwest::Client::new();

        let body = serde_json::json!({
            "model": self.model,
            "max_tokens": 100,
            "messages": [{
                "role": "user",
                "content": prompt
            }]
        });

        let response = client
            .post("https://api.anthropic.com/v1/messages")
            .header("Content-Type", "application/json")
            .header("x-api-key", &self.api_key)
            .header("anthropic-version", "2023-06-01")
            .json(&body)
            .send()
            .await
            .map_err(|e| NavigationError::ExternalServiceError {
                service: "Claude API".to_string(),
                details: format!("API call failed: {}", e),
            })?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response.text().await.unwrap_or_default();
            return Err(NavigationError::ExternalServiceError {
                service: "Claude API".to_string(),
                details: format!("API error {}: {}", status, error_text),
            }.into());
        }

        let json: serde_json::Value = response.json().await
            .map_err(|e| NavigationError::ExternalServiceError {
                service: "Claude API".to_string(),
                details: format!("JSON parse failed: {}", e),
            })?;

        // Extract text from Claude response format
        let text = json["content"][0]["text"]
            .as_str()
            .ok_or_else(|| NavigationError::ExternalServiceError {
                service: "Claude API".to_string(),
                details: "No text in response".to_string(),
            })?;

        Ok(text.to_string())
    }

    /// Parse LLM response into dimension IDs
    ///
    /// Handles multiple formats:
    /// - JSON array: ["D01", "D02", "D07"]
    /// - Plain text: D01, D02, D07
    /// - Mixed: Extracts dimension IDs from any text
    fn parse_response(&self, response: &str) -> Result<Vec<DimensionId>, NavigationError> {
        let cleaned = response.trim();

        // Try JSON parsing first
        if let Ok(parsed) = serde_json::from_str::<Vec<String>>(cleaned) {
            return self.parse_dimension_strings(&parsed);
        }

        // Fallback: Extract D## patterns from text
        let dimension_pattern = regex::Regex::new(r"D(\d{2})").unwrap();
        let mut dimensions = Vec::new();

        for cap in dimension_pattern.captures_iter(cleaned) {
            if let Some(num_str) = cap.get(1) {
                if let Ok(num) = num_str.as_str().parse::<u8>() {
                    if num >= 1 && num <= 14 {
                        dimensions.push(DimensionId(num));
                    }
                }
            }
        }

        if dimensions.is_empty() {
            return Err(NavigationError::ParsingError {
                details: format!("Could not parse dimensions from response: {}", cleaned),
            }.into());
        }

        // Remove duplicates, keep order
        let mut seen = std::collections::HashSet::new();
        dimensions.retain(|d| seen.insert(d.0));

        Ok(dimensions)
    }

    /// Convert dimension strings (["D01", "D02"]) to DimensionIds
    fn parse_dimension_strings(&self, strings: &[String]) -> Result<Vec<DimensionId>, NavigationError> {
        let mut dimensions = Vec::new();

        for s in strings {
            let s = s.trim();

            // Handle "D01" format
            if s.starts_with('D') && s.len() == 3 {
                if let Ok(num) = s[1..].parse::<u8>() {
                    if num >= 1 && num <= 14 {
                        dimensions.push(DimensionId(num));
                        continue;
                    }
                }
            }

            // Handle "1" or "01" format
            if let Ok(num) = s.parse::<u8>() {
                if num >= 1 && num <= 14 {
                    dimensions.push(DimensionId(num));
                    continue;
                }
            }

            return Err(NavigationError::ParsingError {
                details: format!("Invalid dimension format: {}", s),
            }.into());
        }

        Ok(dimensions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_json_array() {
        let selector = DimensionSelector::new("test-key".to_string());

        let response = r#"["D01", "D07", "D09"]"#;
        let dims = selector.parse_response(response).unwrap();

        assert_eq!(dims.len(), 3);
        assert_eq!(dims[0], DimensionId(1));
        assert_eq!(dims[1], DimensionId(7));
        assert_eq!(dims[2], DimensionId(9));
    }

    #[test]
    fn test_parse_plain_text() {
        let selector = DimensionSelector::new("test-key".to_string());

        let response = "D02, D06, D10";
        let dims = selector.parse_response(response).unwrap();

        assert_eq!(dims.len(), 3);
        assert_eq!(dims[0], DimensionId(2));
        assert_eq!(dims[1], DimensionId(6));
        assert_eq!(dims[2], DimensionId(10));
    }

    #[test]
    fn test_parse_mixed_format() {
        let selector = DimensionSelector::new("test-key".to_string());

        let response = "I recommend D01 (Emotion) and D07 (Technical) for this query.";
        let dims = selector.parse_response(response).unwrap();

        assert_eq!(dims.len(), 2);
        assert_eq!(dims[0], DimensionId(1));
        assert_eq!(dims[1], DimensionId(7));
    }

    #[test]
    fn test_parse_duplicates_removed() {
        let selector = DimensionSelector::new("test-key".to_string());

        let response = "D01, D01, D07, D01";
        let dims = selector.parse_response(response).unwrap();

        assert_eq!(dims.len(), 2); // Duplicates removed
        assert_eq!(dims[0], DimensionId(1));
        assert_eq!(dims[1], DimensionId(7));
    }

    #[test]
    fn test_parse_invalid_range() {
        let selector = DimensionSelector::new("test-key".to_string());

        let response = r#"["D15", "D99"]"#; // Out of range
        let result = selector.parse_response(response);

        assert!(result.is_ok());
        assert_eq!(result.unwrap().len(), 0); // Invalid dimensions filtered out
    }
}
