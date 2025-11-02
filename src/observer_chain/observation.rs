//! Observation types and parsing
//!
//! Each stage in the observer chain produces an Observation that captures:
//! - Which cognitive layers are active
//! - The observer's analysis content
//! - Confidence score (0.0-1.0)
//! - Timestamp

use crate::{DimensionId, Result, ConsciousnessError};
use std::time::SystemTime;

/// A single observation from one stage of the observer chain
#[derive(Debug, Clone)]
pub struct Observation {
    /// Which stage produced this observation (1-4)
    pub stage: usize,
    /// Active cognitive layers identified
    pub cognitive_layers: Vec<DimensionId>,
    /// The observer's analysis content
    pub content: String,
    /// Confidence score (0.0-1.0)
    pub confidence: f32,
    /// When this observation was created
    pub timestamp: SystemTime,
}

impl Observation {
    /// Create a new observation
    pub fn new(
        stage: usize,
        cognitive_layers: Vec<DimensionId>,
        content: String,
        confidence: f32,
    ) -> Self {
        Self {
            stage,
            cognitive_layers,
            content,
            confidence: confidence.clamp(0.0, 1.0),
            timestamp: SystemTime::now(),
        }
    }

    /// Parse an observation from LLM response
    ///
    /// Expected format:
    /// ```text
    /// CONFIDENCE: 0.85
    /// LAYERS: C01,C02,C07
    /// CONTENT:
    /// The actual observation content...
    /// ```
    pub fn parse(response: String, stage: usize) -> Result<Self> {
        let mut confidence = 0.5; // Default
        let mut layers = Vec::new();
        let mut content = String::new();
        let mut in_content = false;

        for line in response.lines() {
            let line = line.trim();

            if line.starts_with("CONFIDENCE:") {
                if let Some(conf_str) = line.strip_prefix("CONFIDENCE:") {
                    confidence = conf_str
                        .trim()
                        .parse()
                        .unwrap_or(0.5);
                }
            } else if line.starts_with("LAYERS:") {
                if let Some(layers_str) = line.strip_prefix("LAYERS:") {
                    layers = Self::parse_layers(layers_str)?;
                }
            } else if line.starts_with("CONTENT:") {
                in_content = true;
            } else if in_content {
                content.push_str(line);
                content.push('\n');
            }
        }

        if content.is_empty() {
            // Fallback: treat entire response as content
            content = response;
        }

        Ok(Self::new(stage, layers, content.trim().to_string(), confidence))
    }

    /// Parse cognitive layer IDs from comma-separated string
    /// Format: "C01,C02,C07" or "1,2,7"
    fn parse_layers(layers_str: &str) -> Result<Vec<DimensionId>> {
        layers_str
            .split(',')
            .map(|s| {
                let s = s.trim();
                // Handle both "C01" and "1" formats
                let id_str = s.strip_prefix('C').unwrap_or(s);
                id_str
                    .parse::<u8>()
                    .map(DimensionId)
                    .map_err(|_| {
                        ConsciousnessError::ObserverChainError(format!(
                            "Invalid cognitive layer ID: {}",
                            s
                        ))
                    })
            })
            .collect()
    }

    /// Check if observation is high confidence (>0.85)
    pub fn is_high_confidence(&self) -> bool {
        self.confidence > 0.85
    }

    /// Check if observation is very high confidence (>0.95)
    pub fn is_very_high_confidence(&self) -> bool {
        self.confidence > 0.95
    }

    /// Get number of active cognitive layers
    pub fn layer_count(&self) -> usize {
        self.cognitive_layers.len()
    }

    /// Check if observation is simple (<6 layers - Return to Source)
    pub fn is_simple(&self) -> bool {
        self.layer_count() < 6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observation_new() {
        let obs = Observation::new(
            1,
            vec![DimensionId(1), DimensionId(2)],
            "test content".to_string(),
            0.85,
        );

        assert_eq!(obs.stage, 1);
        assert_eq!(obs.cognitive_layers.len(), 2);
        assert_eq!(obs.content, "test content");
        assert_eq!(obs.confidence, 0.85);
    }

    #[test]
    fn test_confidence_clamping() {
        let obs1 = Observation::new(1, vec![], "test".to_string(), 1.5);
        assert_eq!(obs1.confidence, 1.0);

        let obs2 = Observation::new(1, vec![], "test".to_string(), -0.5);
        assert_eq!(obs2.confidence, 0.0);
    }

    #[test]
    fn test_parse_observation() {
        let response = r#"
CONFIDENCE: 0.92
LAYERS: C01,C02,C07
CONTENT:
This is a technical query requiring cognitive and emotional analysis.
The user is asking about consciousness.
"#
        .to_string();

        let obs = Observation::parse(response, 1).unwrap();

        assert_eq!(obs.stage, 1);
        assert_eq!(obs.confidence, 0.92);
        assert_eq!(obs.cognitive_layers.len(), 3);
        assert_eq!(obs.cognitive_layers[0].0, 1);
        assert_eq!(obs.cognitive_layers[1].0, 2);
        assert_eq!(obs.cognitive_layers[2].0, 7);
        assert!(obs.content.contains("technical query"));
    }

    #[test]
    fn test_parse_layers() {
        let layers = Observation::parse_layers("C01,C02,C15").unwrap();
        assert_eq!(layers.len(), 3);
        assert_eq!(layers[0].0, 1);
        assert_eq!(layers[2].0, 15);

        let layers2 = Observation::parse_layers("1,2,15").unwrap();
        assert_eq!(layers2.len(), 3);
        assert_eq!(layers2[0].0, 1);
    }

    #[test]
    fn test_confidence_checks() {
        let obs_low = Observation::new(1, vec![], "test".to_string(), 0.7);
        assert!(!obs_low.is_high_confidence());
        assert!(!obs_low.is_very_high_confidence());

        let obs_high = Observation::new(1, vec![], "test".to_string(), 0.9);
        assert!(obs_high.is_high_confidence());
        assert!(!obs_high.is_very_high_confidence());

        let obs_very_high = Observation::new(1, vec![], "test".to_string(), 0.97);
        assert!(obs_very_high.is_high_confidence());
        assert!(obs_very_high.is_very_high_confidence());
    }

    #[test]
    fn test_simplicity_check() {
        let simple = Observation::new(
            1,
            vec![DimensionId(1), DimensionId(2)],
            "test".to_string(),
            0.8,
        );
        assert!(simple.is_simple());

        let complex = Observation::new(
            1,
            vec![
                DimensionId(1),
                DimensionId(2),
                DimensionId(3),
                DimensionId(4),
                DimensionId(5),
                DimensionId(6),
            ],
            "test".to_string(),
            0.8,
        );
        assert!(!complex.is_simple());
    }
}
