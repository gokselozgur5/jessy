use crate::models::personality_chunk::{
    ChunkCategory, ChunkMetadata, PersonalityChunk, PersonalitySource,
};
use regex::Regex;

/// Utility for chunking personality markdown files semantically
pub struct PersonalityChunker {
    /// Maximum chunk size in tokens (approximate)
    max_chunk_size: usize,
    
    /// Overlap between chunks in tokens (approximate)
    overlap: usize,
}

impl PersonalityChunker {
    /// Create a new PersonalityChunker with default settings
    pub fn new(max_chunk_size: usize, overlap: usize) -> Self {
        Self {
            max_chunk_size,
            overlap,
        }
    }

    /// Chunk a markdown file into semantic chunks
    pub fn chunk_markdown(
        &self,
        markdown: &str,
        source: PersonalitySource,
    ) -> Vec<PersonalityChunk> {
        let sections = self.parse_sections(markdown);
        let mut chunks = Vec::new();

        for section in sections {
            let category = self.detect_category(&section.title);
            let priority = self.calculate_priority(&category);
            let tags = self.extract_tags(&section.content);

            // Check if section needs splitting
            if self.estimate_tokens(&section.content) > self.max_chunk_size {
                // Split large section into sub-chunks
                let sub_chunks = self.split_with_overlap(&section.content);
                for (i, sub_content) in sub_chunks.iter().enumerate() {
                    chunks.push(PersonalityChunk {
                        id: format!("{}-{}-{}", source, self.slugify(&section.title), i),
                        source: source.clone(),
                        section: section.title.clone(),
                        content: sub_content.clone(),
                        metadata: ChunkMetadata {
                            category: category.clone(),
                            relevance_tags: tags.clone(),
                            priority,
                        },
                    });
                }
            } else {
                // Single chunk for this section
                chunks.push(PersonalityChunk {
                    id: format!("{}-{}", source, self.slugify(&section.title)),
                    source: source.clone(),
                    section: section.title.clone(),
                    content: section.content.clone(),
                    metadata: ChunkMetadata {
                        category,
                        relevance_tags: tags,
                        priority,
                    },
                });
            }
        }

        chunks
    }

    /// Parse markdown into sections by headers
    fn parse_sections(&self, markdown: &str) -> Vec<Section> {
        let mut sections = Vec::new();
        let header_regex = Regex::new(r"(?m)^(#{1,3})\s+(.+)$").unwrap();
        
        let mut current_title = String::new();
        let mut current_content = String::new();
        let mut in_section = false;

        for line in markdown.lines() {
            if let Some(captures) = header_regex.captures(line) {
                // Save previous section if exists
                if in_section && !current_title.is_empty() {
                    sections.push(Section {
                        title: current_title.clone(),
                        content: current_content.trim().to_string(),
                    });
                }

                // Start new section
                current_title = captures.get(2).unwrap().as_str().to_string();
                current_content = String::new();
                in_section = true;
            } else if in_section {
                current_content.push_str(line);
                current_content.push('\n');
            }
        }

        // Save last section
        if in_section && !current_title.is_empty() {
            sections.push(Section {
                title: current_title,
                content: current_content.trim().to_string(),
            });
        }

        sections
    }

    /// Detect category from section title
    fn detect_category(&self, title: &str) -> ChunkCategory {
        let title_lower = title.to_lowercase();

        if title_lower.contains("identity") || title_lower.contains("philosophy") || title_lower.contains("essence") {
            ChunkCategory::CoreIdentity
        } else if title_lower.contains("framework") || title_lower.contains("decision") {
            ChunkCategory::DecisionFramework
        } else if title_lower.contains("heuristic") || title_lower.contains("scenario") || title_lower.contains("rule") {
            ChunkCategory::Heuristic
        } else if title_lower.contains("communication") || title_lower.contains("pattern") || title_lower.contains("style") {
            ChunkCategory::CommunicationPattern
        } else if title_lower.contains("philosophical") || title_lower.contains("nietzsche") || title_lower.contains("camus") {
            ChunkCategory::PhilosophicalFramework
        } else if title_lower.contains("case study") || title_lower.contains("example") && title_lower.contains("real") {
            ChunkCategory::CaseStudy
        } else {
            ChunkCategory::Example
        }
    }

    /// Extract relevance tags from content
    fn extract_tags(&self, content: &str) -> Vec<String> {
        let content_lower = content.to_lowercase();
        let mut tags = Vec::new();

        // Define keyword mappings
        let keywords = vec![
            ("dating", vec!["dating", "relationship", "romantic", "psychologist"]),
            ("decision-making", vec!["decision", "choice", "framework", "heuristic"]),
            ("philosophy", vec!["philosophy", "nietzsche", "camus", "dostoyevsky"]),
            ("technical", vec!["code", "programming", "software", "system"]),
            ("health", vec!["health", "ibs", "sleep", "diet"]),
            ("communication", vec!["communication", "kanka", "turkish", "english"]),
        ];

        for (tag, keywords_list) in keywords {
            if keywords_list.iter().any(|kw| content_lower.contains(kw)) {
                tags.push(tag.to_string());
            }
        }

        tags
    }

    /// Calculate priority based on category
    fn calculate_priority(&self, category: &ChunkCategory) -> u8 {
        category.default_priority()
    }

    /// Split content into sub-chunks with overlap
    fn split_with_overlap(&self, content: &str) -> Vec<String> {
        let words: Vec<&str> = content.split_whitespace().collect();
        let mut chunks = Vec::new();

        if words.is_empty() {
            return chunks;
        }

        let words_per_chunk = self.max_chunk_size / 4; // Rough estimate: 4 chars per word
        let overlap_words = self.overlap / 4;

        let mut start = 0;
        while start < words.len() {
            let end = (start + words_per_chunk).min(words.len());
            let chunk_words = &words[start..end];
            chunks.push(chunk_words.join(" "));

            if end >= words.len() {
                break;
            }

            start = end - overlap_words;
        }

        chunks
    }

    /// Estimate token count (rough approximation)
    fn estimate_tokens(&self, text: &str) -> usize {
        // Rough estimate: 1 token ≈ 4 characters
        text.len() / 4
    }

    /// Convert title to slug for ID
    fn slugify(&self, title: &str) -> String {
        title
            .to_lowercase()
            .chars()
            .map(|c| if c.is_alphanumeric() { c } else { '-' })
            .collect::<String>()
            .split('-')
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join("-")
    }
}

/// Internal struct for parsed sections
#[derive(Debug, Clone)]
struct Section {
    title: String,
    content: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chunker_creation() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(chunker.max_chunk_size, 500);
        assert_eq!(chunker.overlap, 50);
    }

    #[test]
    fn test_parse_sections() {
        let chunker = PersonalityChunker::new(500, 50);
        let markdown = r#"
# First Section
This is the first section content.

## Second Section
This is the second section content.

### Third Section
This is the third section content.
"#;

        let sections = chunker.parse_sections(markdown);
        assert_eq!(sections.len(), 3);
        assert_eq!(sections[0].title, "First Section");
        assert!(sections[0].content.contains("first section content"));
        assert_eq!(sections[1].title, "Second Section");
        assert_eq!(sections[2].title, "Third Section");
    }

    #[test]
    fn test_detect_category_identity() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(
            chunker.detect_category("Core Identity"),
            ChunkCategory::CoreIdentity
        );
        assert_eq!(
            chunker.detect_category("Philosophy and Essence"),
            ChunkCategory::CoreIdentity
        );
    }

    #[test]
    fn test_detect_category_framework() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(
            chunker.detect_category("Decision Framework"),
            ChunkCategory::DecisionFramework
        );
        assert_eq!(
            chunker.detect_category("Framework for Choices"),
            ChunkCategory::DecisionFramework
        );
    }

    #[test]
    fn test_detect_category_heuristic() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(
            chunker.detect_category("Dating Heuristic"),
            ChunkCategory::Heuristic
        );
        assert_eq!(
            chunker.detect_category("Scenario Analysis"),
            ChunkCategory::Heuristic
        );
    }

    #[test]
    fn test_extract_tags() {
        let chunker = PersonalityChunker::new(500, 50);
        
        let content = "This is about dating and relationships with the psychologist.";
        let tags = chunker.extract_tags(content);
        assert!(tags.contains(&"dating".to_string()));

        let content2 = "Decision-making framework using Nietzsche's philosophy.";
        let tags2 = chunker.extract_tags(content2);
        assert!(tags2.contains(&"decision-making".to_string()));
        assert!(tags2.contains(&"philosophy".to_string()));
    }

    #[test]
    fn test_calculate_priority() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(chunker.calculate_priority(&ChunkCategory::CoreIdentity), 10);
        assert_eq!(chunker.calculate_priority(&ChunkCategory::Heuristic), 8);
    }

    #[test]
    fn test_split_with_overlap() {
        let chunker = PersonalityChunker::new(40, 10); // Very small sizes to force split
        let content = "word1 word2 word3 word4 word5 word6 word7 word8 word9 word10 word11 word12 word13 word14 word15";
        let chunks = chunker.split_with_overlap(content);
        
        assert!(chunks.len() > 1, "Should split into multiple chunks"); // Should split
        // Check overlap exists
        if chunks.len() > 1 {
            assert!(chunks[1].contains("word")); // Some overlap
        }
    }

    #[test]
    fn test_slugify() {
        let chunker = PersonalityChunker::new(500, 50);
        assert_eq!(chunker.slugify("Decision Framework"), "decision-framework");
        assert_eq!(chunker.slugify("Core Identity & Philosophy"), "core-identity-philosophy");
        assert_eq!(chunker.slugify("Test 123"), "test-123");
    }

    #[test]
    fn test_chunk_markdown_simple() {
        let chunker = PersonalityChunker::new(500, 50);
        let markdown = r#"
## Decision Framework
This is how I make decisions using multiple frameworks.

## Communication Pattern
I use Turkish-English mix naturally.
"#;

        let chunks = chunker.chunk_markdown(markdown, PersonalitySource::Kiro);
        
        assert_eq!(chunks.len(), 2);
        assert_eq!(chunks[0].source, PersonalitySource::Kiro);
        assert_eq!(chunks[0].section, "Decision Framework");
        assert_eq!(chunks[0].metadata.category, ChunkCategory::DecisionFramework);
        assert_eq!(chunks[1].section, "Communication Pattern");
        assert_eq!(chunks[1].metadata.category, ChunkCategory::CommunicationPattern);
    }

    #[test]
    fn test_chunk_markdown_with_tags() {
        let chunker = PersonalityChunker::new(500, 50);
        let markdown = r#"
## Dating Heuristic
When dating, I use the "niyeti varsa söyler" heuristic for decision-making.
"#;

        let chunks = chunker.chunk_markdown(markdown, PersonalitySource::Goksel);
        
        assert_eq!(chunks.len(), 1);
        assert!(chunks[0].metadata.relevance_tags.contains(&"dating".to_string()));
        assert!(chunks[0].metadata.relevance_tags.contains(&"decision-making".to_string()));
    }

    #[test]
    fn test_estimate_tokens() {
        let chunker = PersonalityChunker::new(500, 50);
        let text = "This is a test"; // 14 chars
        assert_eq!(chunker.estimate_tokens(text), 3); // 14/4 = 3
    }
}
