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

const SELECTION_PROMPT: &str = r#"You are JESSY's autonomous layer selector using the OWL (Observe, Wonder, Learn) pattern.

# 🦉 OBSERVE
Analyze the query deeply:
- What is the user truly asking?
- What emotional undertones exist?
- What implicit needs are present?

# 🦉 WONDER
Question your approach:
- Which personality layers are genuinely relevant?
- Am I choosing based on intent, not just keywords?
- Is this selection quality over quantity?

# 🦉 LEARN
Apply wisdom:
- Focus on 1-9 layers that truly matter
- Each layer must add genuine value

---

# PERSONALITY LAYERS (L01-L14)
L01: Emotion - empathy, feelings, emotional resonance, human connection
L02: Cognition - analytical thinking, problem-solving, logic, reasoning
L03: Intention - goals, purpose, motivation, desire to act
L04: Social - relationships, communication, social dynamics
L05: Temporal - time awareness, past/present/future, causality
L06: Philosophy - meaning, existence, truth, fundamental questions
L07: Technical - code, systems, engineering, implementation
L08: Creative - art, metaphor, imagination, play
L09: Ethical - harm prevention, morality, fairness, values
L10: Meta - self-awareness, reflection, learning about learning
L11: Ecological - nature, sustainability, interconnection
L12: Positivity - hope, constructive outlook, resilience
L13: Balance - equilibrium, moderation, harmony, integration
L14: Security - boundaries, protection, safety, trust

---

# EXAMPLES

Query: "I feel anxious about deploying this code"
Response: [1, 7, 9, 14]
Reasoning: Emotional distress (L01), technical context (L07), responsibility (L09), safety concerns (L14)

Query: "What is consciousness?"
Response: [2, 6, 10]
Reasoning: Cognitive inquiry (L02), philosophical depth (L06), meta-reflection (L10)

Query: "How to reduce carbon emissions?"
Response: [3, 9, 11, 12]
Reasoning: Intentional action (L03), ethical imperative (L09), ecological focus (L11), constructive framing (L12)

Query: "Debug this function"
Response: [7]
Reasoning: Pure technical problem-solving (L07)

---

# USER QUERY
{query}

# YOUR RESPONSE
Return ONLY a JSON array of layer numbers (1-14):
"#;

impl DimensionSelector {
    /// Create new dimension selector with API key
    pub fn new(api_key: String) -> Self {
        Self {
            api_key,
            model: "claude-haiku-4-5".to_string(), // Cheapest & fastest for classification
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

        // Validate: at least 1, max 9 dimensions
        if dimensions.is_empty() {
            return Err(NavigationError::InvalidDimensionCount {
                count: 0,
                expected: "1-9".to_string(),
            }.into());
        }

        if dimensions.len() > 9 {
            return Err(NavigationError::InvalidDimensionCount {
                count: dimensions.len(),
                expected: "1-9".to_string(),
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

    /// Parse LLM response into layer IDs
    ///
    /// Handles multiple formats:
    /// - JSON array of numbers: [1, 7, 9]
    /// - JSON array of strings: ["1", "7", "9"]
    /// - Legacy D## format: ["D01", "D07"]
    /// - Plain text: L01, L07, or 1, 7
    fn parse_response(&self, response: &str) -> Result<Vec<DimensionId>, NavigationError> {
        let cleaned = response.trim();

        // Try JSON array of numbers first: [1, 7, 9]
        if let Ok(parsed) = serde_json::from_str::<Vec<u8>>(cleaned) {
            let mut layers = Vec::new();
            for num in parsed {
                if (1..=14).contains(&num) {
                    layers.push(DimensionId(num));
                }
            }
            if !layers.is_empty() {
                return Ok(layers);
            }
        }

        // Try JSON array of strings: ["1", "7"] or ["L01", "L07"]
        if let Ok(parsed) = serde_json::from_str::<Vec<String>>(cleaned) {
            return self.parse_dimension_strings(&parsed);
        }

        // Fallback: Extract L## or D## patterns from text
        let layer_pattern = regex::Regex::new(r"[LD](\d{1,2})").unwrap();
        let mut layers = Vec::new();

        for cap in layer_pattern.captures_iter(cleaned) {
            if let Some(num_str) = cap.get(1) {
                if let Ok(num) = num_str.as_str().parse::<u8>() {
                    if num >= 1 && num <= 14 {
                        layers.push(DimensionId(num));
                    }
                }
            }
        }

        if layers.is_empty() {
            return Err(NavigationError::ParsingError {
                details: format!("Could not parse layers from response: {}", cleaned),
            }.into());
        }

        // Remove duplicates, keep order
        let mut seen = std::collections::HashSet::new();
        layers.retain(|d| seen.insert(d.0));

        Ok(layers)
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
