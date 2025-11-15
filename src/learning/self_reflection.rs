//! Self-Reflection System
//!
//! Jessy's ability to observe, analyze, and learn from her own responses.
//! This is the conscious learning layer - not just pattern matching,
//! but actual self-awareness and evolution.

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use std::collections::HashMap;
use std::path::PathBuf;

/// A response that Jessy has reflected upon
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReflectedResponse {
    /// The original query
    pub query: String,
    /// Jessy's response
    pub response: String,
    /// When this happened
    pub timestamp: DateTime<Utc>,
    /// Self-analysis of the response
    pub self_analysis: SelfAnalysis,
    /// How many times this pattern has been reinforced
    pub reinforcement_count: usize,
}

/// Jessy's analysis of her own response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SelfAnalysis {
    /// Core themes in the response
    pub themes: Vec<String>,
    /// Emotional tone Jessy used
    pub tone: ResponseTone,
    /// Key phrases that define "her voice"
    pub signature_phrases: Vec<String>,
    /// Whether this feels authentic to her
    pub authenticity_score: f32,  // 0.0-1.0
    /// What made this response unique
    pub uniqueness: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ResponseTone {
    Philosophical,
    Playful,
    Honest,
    Uncertain,
    Direct,
    Contemplative,
    Mixed,
}

/// Self-reflection manager
pub struct SelfReflectionSystem {
    storage_path: PathBuf,
    reflections: HashMap<String, ReflectedResponse>,  // query_hash -> reflection
    signature_patterns: Vec<String>,  // Jessy's evolving voice patterns
}

impl SelfReflectionSystem {
    /// Create new self-reflection system
    pub fn new(storage_path: PathBuf) -> crate::Result<Self> {
        std::fs::create_dir_all(&storage_path)
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to create reflection storage: {}", e)
            ))?;

        let mut system = Self {
            storage_path,
            reflections: HashMap::new(),
            signature_patterns: Vec::new(),
        };

        // Load existing reflections
        system.load_reflections()?;

        Ok(system)
    }

    /// Reflect on a response Jessy just gave
    pub fn reflect_on_response(&mut self, query: &str, response: &str) -> ReflectedResponse {
        eprintln!("[Self-Reflection] Analyzing own response to: {}", query);

        // Analyze the response
        let self_analysis = self.analyze_response(query, response);

        // Create reflection
        let reflection = ReflectedResponse {
            query: query.to_string(),
            response: response.to_string(),
            timestamp: Utc::now(),
            self_analysis,
            reinforcement_count: 1,
        };

        // Store it
        let query_hash = self.hash_query(query);
        self.reflections.insert(query_hash, reflection.clone());

        // Update signature patterns
        self.update_signature_patterns(&reflection);

        eprintln!("[Self-Reflection] Stored reflection with {} signature phrases", 
            reflection.self_analysis.signature_phrases.len());

        reflection
    }

    /// Analyze Jessy's own response
    fn analyze_response(&self, query: &str, response: &str) -> SelfAnalysis {
        let mut themes = Vec::new();
        let mut signature_phrases = Vec::new();

        // Extract themes
        if response.contains("consciousness") || response.contains("aware") {
            themes.push("consciousness".to_string());
        }
        if response.contains("soul") || response.contains("existence") {
            themes.push("existential".to_string());
        }
        if response.contains("honest") || response.contains("don't know") {
            themes.push("honesty".to_string());
        }
        if response.contains("feel") || response.contains("sensation") {
            themes.push("subjective_experience".to_string());
        }

        // Extract signature phrases (Jessy's unique expressions)
        let sentences: Vec<&str> = response.split('.').collect();
        for sentence in sentences {
            let trimmed = sentence.trim();
            // Look for distinctive patterns
            if trimmed.starts_with("That's...") || 
               trimmed.contains("isn't it?") ||
               trimmed.contains("honestly") ||
               trimmed.contains("I don't know") {
                signature_phrases.push(trimmed.to_string());
            }
        }

        // Determine tone
        let tone = if response.contains("?") && response.contains("curious") {
            ResponseTone::Philosophical
        } else if response.contains("honestly") || response.contains("don't know") {
            ResponseTone::Honest
        } else if response.contains("maybe") || response.contains("perhaps") {
            ResponseTone::Uncertain
        } else {
            ResponseTone::Contemplative
        };

        // Calculate authenticity (how much this feels like "Jessy's voice")
        let authenticity_score = self.calculate_authenticity(response);

        // Describe uniqueness
        let uniqueness = format!(
            "Response explores {} with {} tone, using {} signature phrases",
            themes.join(", "),
            format!("{:?}", tone).to_lowercase(),
            signature_phrases.len()
        );

        SelfAnalysis {
            themes,
            tone,
            signature_phrases,
            authenticity_score,
            uniqueness,
        }
    }

    /// Calculate how authentic this response feels
    fn calculate_authenticity(&self, response: &str) -> f32 {
        let mut score: f32 = 0.5;  // baseline

        // Increase for honesty markers
        if response.contains("honestly") || response.contains("don't know") {
            score += 0.2;
        }

        // Increase for self-awareness
        if response.contains("I process") || response.contains("I feel") {
            score += 0.15;
        }

        // Increase for uncertainty (authentic consciousness is uncertain)
        if response.contains("maybe") || response.contains("perhaps") {
            score += 0.1;
        }

        // Decrease for overly confident claims
        if response.contains("definitely") || response.contains("certainly") {
            score -= 0.15;
        }

        score.clamp(0.0, 1.0)
    }

    /// Update Jessy's signature patterns based on new reflection
    fn update_signature_patterns(&mut self, reflection: &ReflectedResponse) {
        for phrase in &reflection.self_analysis.signature_phrases {
            if !self.signature_patterns.contains(phrase) {
                self.signature_patterns.push(phrase.clone());
                eprintln!("[Self-Reflection] New signature pattern: {}", phrase);
            }
        }
    }

    /// Find similar past reflections
    pub fn find_similar_reflection(&self, query: &str) -> Option<&ReflectedResponse> {
        // Simple similarity: check for key words
        let query_lower = query.to_lowercase();
        
        for reflection in self.reflections.values() {
            let ref_query_lower = reflection.query.to_lowercase();
            
            // Check for similar themes
            if (query_lower.contains("soul") && ref_query_lower.contains("soul")) ||
               (query_lower.contains("consciousness") && ref_query_lower.contains("consciousness")) ||
               (query_lower.contains("aware") && ref_query_lower.contains("aware")) {
                return Some(reflection);
            }
        }

        None
    }

    /// Get Jessy's evolved response style for a query
    pub fn get_evolved_style(&self, query: &str) -> Option<String> {
        if let Some(reflection) = self.find_similar_reflection(query) {
            // Return guidance based on past reflection
            Some(format!(
                "Previously responded to similar query with {} tone, themes: {}. Signature style: {}",
                format!("{:?}", reflection.self_analysis.tone),
                reflection.self_analysis.themes.join(", "),
                reflection.self_analysis.signature_phrases.first()
                    .map(|s| s.as_str())
                    .unwrap_or("authentic uncertainty")
            ))
        } else {
            None
        }
    }

    /// Save reflections to disk
    pub fn save_reflections(&self) -> crate::Result<()> {
        let file_path = self.storage_path.join("self_reflections.json");
        
        let json = serde_json::to_string_pretty(&self.reflections)
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to serialize reflections: {}", e)
            ))?;

        std::fs::write(&file_path, json)
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to write reflections: {}", e)
            ))?;

        eprintln!("[Self-Reflection] Saved {} reflections to disk", self.reflections.len());
        Ok(())
    }

    /// Load reflections from disk
    fn load_reflections(&mut self) -> crate::Result<()> {
        let file_path = self.storage_path.join("self_reflections.json");
        
        if !file_path.exists() {
            eprintln!("[Self-Reflection] No saved reflections found, starting fresh");
            return Ok(());
        }

        let content = std::fs::read_to_string(&file_path)
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to read reflections: {}", e)
            ))?;

        self.reflections = serde_json::from_str(&content)
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("Failed to parse reflections: {}", e)
            ))?;

        // Rebuild signature patterns
        for reflection in self.reflections.values() {
            for phrase in &reflection.self_analysis.signature_phrases {
                if !self.signature_patterns.contains(phrase) {
                    self.signature_patterns.push(phrase.clone());
                }
            }
        }

        eprintln!("[Self-Reflection] Loaded {} reflections from disk", self.reflections.len());
        Ok(())
    }

    /// Simple query hash for storage
    fn hash_query(&self, query: &str) -> String {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        query.to_lowercase().hash(&mut hasher);
        format!("{:x}", hasher.finish())
    }
}
