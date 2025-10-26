//! LLM-based dimension selection
//!
//! Uses LLM to analyze queries and select relevant dimensions intelligently.

use crate::{DimensionId, Result, ConsciousnessError};
use crate::llm::LLMProvider;
use crate::navigation::OwlPattern;
use std::sync::Arc;
use std::collections::HashMap;

/// Result of LLM dimension selection
#[derive(Debug, Clone)]
pub struct DimensionSelection {
    /// Selected dimension IDs
    pub dimension_ids: Vec<DimensionId>,
    
    /// Confidence score per dimension
    pub confidences: HashMap<DimensionId, f32>,
    
    /// LLM reasoning
    pub reasoning: String,
    
    /// OWL binary pattern
    pub owl_pattern: String,
    
    /// Selection duration
    pub duration_ms: u64,
    
    /// Whether fallback was used
    pub is_fallback: bool,
}

/// LLM-based dimension selector
pub struct LLMDimensionSelector {
    llm: Arc<dyn LLMProvider>,
}

impl LLMDimensionSelector {
    /// Create new selector
    pub fn new(llm: Arc<dyn LLMProvider>) -> Self {
        Self { llm }
    }
    
    /// Select dimensions for query using LLM
    pub async fn select_dimensions(&self, query: &str) -> Result<DimensionSelection> {
        let start = std::time::Instant::now();
        
        // Build prompt
        let prompt = self.build_prompt(query);
        
        // Call LLM with minimal context
        use crate::iteration::IterationContext;
        use crate::Frequency;
        let context = IterationContext::new(query.to_string(), Frequency::new(1.5));
        
        let response = match self.llm.generate(&prompt, &context).await {
            Ok(resp) => resp,
            Err(e) => {
                eprintln!("[LLM Dimension Selector] LLM call failed: {}, using fallback", e);
                return Ok(self.fallback_selection());
            }
        };
        
        // Parse response
        let dimension_ids = self.parse_response(&response);
        
        if dimension_ids.is_empty() {
            eprintln!("[LLM Dimension Selector] No dimensions parsed, using fallback");
            return Ok(self.fallback_selection());
        }
        
        // Assign confidences (simple: decreasing by order)
        let mut confidences = HashMap::new();
        for (idx, dim_id) in dimension_ids.iter().enumerate() {
            let confidence = 0.9 - (idx as f32 * 0.1);
            confidences.insert(*dim_id, confidence.max(0.1));
        }
        
        // Generate OWL pattern
        let owl = OwlPattern::encode(&dimension_ids);
        
        let duration_ms = start.elapsed().as_millis() as u64;
        
        Ok(DimensionSelection {
            dimension_ids,
            confidences,
            reasoning: response.clone(),
            owl_pattern: owl.as_str().to_string(),
            duration_ms,
            is_fallback: false,
        })
    }
    
    fn build_prompt(&self, query: &str) -> String {
        format!(
            r#"You are JESSY's dimension selector. Analyze this query and select 3-7 relevant dimensions.

Available Dimensions:
1. Emotion - feelings, moods, emotional states
2. Cognition - thinking, reasoning, understanding
3. Intention - goals, purposes, desires
4. Social Context - relationships, interactions
5. Temporal State - time, timing, sequences
6. Philosophical Depth - existential, metaphysical
7. Technical Level - engineering, systems
8. Creative Mode - imagination, innovation
9. Ethical Framework - morals, values
10. Meta-Awareness - self-reference, consciousness
11. Ecological - nature, environment, systems
12. Positivity - optimism, hope, growth
13. Balance - harmony, equilibrium
14. Security - safety, protection, trust

Query: "{}"

Respond with ONLY dimension numbers separated by commas. Example: 2,4,10

Select 3-7 dimensions based on semantic meaning:"#,
            query
        )
    }
    
    fn parse_response(&self, response: &str) -> Vec<DimensionId> {
        // Extract numbers from response
        response
            .split(|c: char| !c.is_numeric())
            .filter_map(|s| s.parse::<u8>().ok())
            .filter(|&n| n >= 1 && n <= 14)
            .map(DimensionId)
            .collect()
    }
    
    fn fallback_selection(&self) -> DimensionSelection {
        // Default dimensions: Cognition, Social, Meta-Awareness
        let dimension_ids = vec![DimensionId(2), DimensionId(4), DimensionId(10)];
        let owl = OwlPattern::encode(&dimension_ids);
        
        let mut confidences = HashMap::new();
        confidences.insert(DimensionId(2), 0.5);
        confidences.insert(DimensionId(4), 0.5);
        confidences.insert(DimensionId(10), 0.5);
        
        DimensionSelection {
            dimension_ids,
            confidences,
            reasoning: "Fallback: using default dimensions".to_string(),
            owl_pattern: owl.as_str().to_string(),
            duration_ms: 0,
            is_fallback: true,
        }
    }
}
