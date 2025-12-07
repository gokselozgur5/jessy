use serde::{Deserialize, Serialize};

/// Represents a semantic chunk of personality data from steering files
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PersonalityChunk {
    /// Unique identifier (e.g., "kiro-decision-framework-1")
    pub id: String,
    
    /// Source of the personality data
    pub source: PersonalitySource,
    
    /// Section title from markdown (e.g., "Decision Framework")
    pub section: String,
    
    /// Actual text content of the chunk
    pub content: String,
    
    /// Metadata about the chunk
    pub metadata: ChunkMetadata,
}

/// Source of personality data
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum PersonalitySource {
    /// From Kiro's personality file
    Kiro,

    /// From Göksel's personality file
    Goksel,

    /// From dynamic memory (runtime learnings, training)
    DynamicMemory,
}

impl std::fmt::Display for PersonalitySource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PersonalitySource::Kiro => write!(f, "Kiro"),
            PersonalitySource::Goksel => write!(f, "Göksel"),
            PersonalitySource::DynamicMemory => write!(f, "DynamicMemory"),
        }
    }
}

/// Metadata about a personality chunk
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ChunkMetadata {
    /// Category of the chunk
    pub category: ChunkCategory,
    
    /// Relevance tags for semantic search (e.g., ["dating", "philosophy"])
    pub relevance_tags: Vec<String>,
    
    /// Priority for ranking (1-10, where 10 is highest)
    pub priority: u8,
}

/// Category of personality chunk
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum ChunkCategory {
    /// Core identity and philosophy
    CoreIdentity,
    
    /// Decision-making frameworks
    DecisionFramework,
    
    /// Specific heuristics and rules
    Heuristic,
    
    /// Communication patterns and style
    CommunicationPattern,
    
    /// Philosophical foundations
    PhilosophicalFramework,
    
    /// Real-world case studies
    CaseStudy,
    
    /// Generic examples
    Example,
}

impl ChunkCategory {
    /// Get default priority for this category
    pub fn default_priority(&self) -> u8 {
        match self {
            ChunkCategory::CoreIdentity => 10,
            ChunkCategory::DecisionFramework => 9,
            ChunkCategory::Heuristic => 8,
            ChunkCategory::PhilosophicalFramework => 7,
            ChunkCategory::CommunicationPattern => 6,
            ChunkCategory::CaseStudy => 5,
            ChunkCategory::Example => 4,
        }
    }
}

impl std::fmt::Display for ChunkCategory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChunkCategory::CoreIdentity => write!(f, "CoreIdentity"),
            ChunkCategory::DecisionFramework => write!(f, "DecisionFramework"),
            ChunkCategory::Heuristic => write!(f, "Heuristic"),
            ChunkCategory::CommunicationPattern => write!(f, "CommunicationPattern"),
            ChunkCategory::PhilosophicalFramework => write!(f, "PhilosophicalFramework"),
            ChunkCategory::CaseStudy => write!(f, "CaseStudy"),
            ChunkCategory::Example => write!(f, "Example"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_personality_chunk_creation() {
        let chunk = PersonalityChunk {
            id: "kiro-test-1".to_string(),
            source: PersonalitySource::Kiro,
            section: "Test Section".to_string(),
            content: "Test content".to_string(),
            metadata: ChunkMetadata {
                category: ChunkCategory::CoreIdentity,
                relevance_tags: vec!["test".to_string()],
                priority: 10,
            },
        };

        assert_eq!(chunk.id, "kiro-test-1");
        assert_eq!(chunk.source, PersonalitySource::Kiro);
        assert_eq!(chunk.section, "Test Section");
        assert_eq!(chunk.content, "Test content");
        assert_eq!(chunk.metadata.category, ChunkCategory::CoreIdentity);
        assert_eq!(chunk.metadata.priority, 10);
    }

    #[test]
    fn test_personality_source_display() {
        assert_eq!(PersonalitySource::Kiro.to_string(), "Kiro");
        assert_eq!(PersonalitySource::Goksel.to_string(), "Göksel");
    }

    #[test]
    fn test_chunk_category_priority() {
        assert_eq!(ChunkCategory::CoreIdentity.default_priority(), 10);
        assert_eq!(ChunkCategory::DecisionFramework.default_priority(), 9);
        assert_eq!(ChunkCategory::Heuristic.default_priority(), 8);
        assert_eq!(ChunkCategory::PhilosophicalFramework.default_priority(), 7);
        assert_eq!(ChunkCategory::CommunicationPattern.default_priority(), 6);
        assert_eq!(ChunkCategory::CaseStudy.default_priority(), 5);
        assert_eq!(ChunkCategory::Example.default_priority(), 4);
    }

    #[test]
    fn test_chunk_category_display() {
        assert_eq!(ChunkCategory::CoreIdentity.to_string(), "CoreIdentity");
        assert_eq!(ChunkCategory::Heuristic.to_string(), "Heuristic");
    }

    #[test]
    fn test_serialization() {
        let chunk = PersonalityChunk {
            id: "test-1".to_string(),
            source: PersonalitySource::Goksel,
            section: "Philosophy".to_string(),
            content: "Nothing is true".to_string(),
            metadata: ChunkMetadata {
                category: ChunkCategory::PhilosophicalFramework,
                relevance_tags: vec!["philosophy".to_string(), "nietzsche".to_string()],
                priority: 7,
            },
        };

        // Serialize to JSON
        let json = serde_json::to_string(&chunk).unwrap();
        
        // Deserialize back
        let deserialized: PersonalityChunk = serde_json::from_str(&json).unwrap();
        
        assert_eq!(chunk, deserialized);
    }

    #[test]
    fn test_chunk_metadata() {
        let metadata = ChunkMetadata {
            category: ChunkCategory::Heuristic,
            relevance_tags: vec!["dating".to_string(), "decision-making".to_string()],
            priority: 8,
        };

        assert_eq!(metadata.category, ChunkCategory::Heuristic);
        assert_eq!(metadata.relevance_tags.len(), 2);
        assert!(metadata.relevance_tags.contains(&"dating".to_string()));
        assert_eq!(metadata.priority, 8);
    }
}
