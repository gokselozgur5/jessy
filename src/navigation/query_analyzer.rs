//! Query analysis for extracting keywords and metadata from user queries
//!
//! This module implements the query analysis phase of navigation, which:
//! - Extracts keywords from queries
//! - Identifies emotional and technical indicators
//! - Classifies question types
//! - Assigns urgency levels
//! - Estimates query frequency and complexity
//!
//! The analyzer uses vocabulary files to identify indicators and follows
//! strict TDD methodology with comprehensive test coverage.

use crate::navigation::{NavigationError, QuestionType, UrgencyLevel, QueryAnalysis};
use std::collections::HashSet;
use std::fs;
use std::path::Path;

/// Query analyzer for extracting keywords and metadata from queries
///
/// The QueryAnalyzer is initialized with vocabulary files and provides
/// methods to analyze queries according to Requirement 1 (Query Analysis).
///
/// # Thread Safety
/// QueryAnalyzer is immutable after initialization and can be safely shared
/// across threads using Arc.
///
/// # Performance
/// - Keyword extraction: <5ms (Requirement 1.1)
/// - Full analysis: <5ms total
/// - Vocabulary lookups: O(1) using HashSet
#[derive(Debug, Clone)]
pub struct QueryAnalyzer {
    /// Emotional indicator words (100+ words, Requirement 14.4)
    emotional_vocabulary: HashSet<String>,
    
    /// Technical indicator words (100+ words, Requirement 14.5)
    technical_vocabulary: HashSet<String>,
    
    /// Common stopwords to filter out (Requirement 14.6)
    stopwords: HashSet<String>,
    
    /// Philosophical keywords for question type classification
    philosophical_keywords: HashSet<String>,
    
    /// Interrogative words for factual question detection
    interrogative_words: HashSet<String>,
    
    /// High urgency keywords
    high_urgency_keywords: HashSet<String>,
    
    /// High-intensity emotional keywords for frequency adjustment
    high_intensity_emotional: HashSet<String>,
}

impl QueryAnalyzer {
    /// Create a new QueryAnalyzer by loading vocabulary files
    ///
    /// # Arguments
    /// * `emotional_path` - Path to emotional vocabulary file
    /// * `technical_path` - Path to technical vocabulary file
    /// * `stopwords_path` - Path to stopwords file
    ///
    /// # Returns
    /// Returns `Ok(QueryAnalyzer)` on success, or `NavigationError` if:
    /// - Any vocabulary file cannot be loaded
    /// - Vocabulary validation fails (< 100 words for emotional/technical)
    ///
    /// # Requirements
    /// - 14.4: Load emotional vocabulary with 100+ words
    /// - 14.5: Load technical vocabulary with 100+ words
    /// - 14.6: Load stopword list
    /// - 14.11-14.12: Validate vocabularies and handle errors
    pub fn new<P: AsRef<Path>>(
        emotional_path: P,
        technical_path: P,
        stopwords_path: P,
    ) -> Result<Self, NavigationError> {
        // Load vocabularies
        let emotional_vocabulary = Self::load_vocabulary(
            emotional_path.as_ref(),
            "emotional",
            Some(100),
        )?;
        
        let technical_vocabulary = Self::load_vocabulary(
            technical_path.as_ref(),
            "technical",
            Some(100),
        )?;
        
        let stopwords = Self::load_vocabulary(
            stopwords_path.as_ref(),
            "stopwords",
            None,
        )?;
        
        // Initialize philosophical keywords
        let philosophical_keywords = [
            "meaning", "purpose", "existence", "consciousness", "reality",
            "truth", "knowledge", "wisdom", "ethics", "morality",
            "being", "essence", "nature", "philosophy", "philosophical",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        
        // Initialize interrogative words
        let interrogative_words = [
            "what", "when", "where", "who", "why", "how",
            "which", "whose", "whom",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        
        // Initialize high urgency keywords (Requirement 1.13)
        let high_urgency_keywords = [
            "urgent", "emergency", "critical", "immediately",
            "asap", "crisis", "danger", "serious",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        
        // Initialize high-intensity emotional keywords (Requirement 15.6)
        let high_intensity_emotional = [
            "terrified", "devastated", "ecstatic", "furious",
            "enraged", "panicked", "desperate", "hopeless",
            "suicidal", "traumatized", "overwhelmed",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        
        Ok(Self {
            emotional_vocabulary,
            technical_vocabulary,
            stopwords,
            philosophical_keywords,
            interrogative_words,
            high_urgency_keywords,
            high_intensity_emotional,
        })
    }
    
    /// Load vocabulary from a file
    ///
    /// # Arguments
    /// * `path` - Path to vocabulary file
    /// * `name` - Name of vocabulary (for error messages)
    /// * `min_words` - Minimum required word count (None for no validation)
    ///
    /// # Returns
    /// Returns `Ok(HashSet<String>)` with loaded words, or `NavigationError` if:
    /// - File cannot be read
    /// - File is empty
    /// - Word count is below minimum (if specified)
    ///
    /// # File Format
    /// - One word per line
    /// - Lines starting with '#' are comments
    /// - Empty lines are ignored
    /// - Words are converted to lowercase
    ///
    /// # Requirements
    /// - 14.4-14.6: Load vocabulary files
    /// - 14.11: Handle missing files
    /// - 14.12: Handle empty files and validation
    fn load_vocabulary<P: AsRef<Path>>(
        path: P,
        name: &str,
        min_words: Option<usize>,
    ) -> Result<HashSet<String>, NavigationError> {
        // Read file contents
        let contents = fs::read_to_string(path.as_ref()).map_err(|e| {
            NavigationError::VocabularyLoadFailed {
                vocabulary_name: name.to_string(),
                reason: format!("Failed to read file: {}", e),
            }
        })?;
        
        // Parse words from file
        let mut words = HashSet::new();
        for line in contents.lines() {
            let line = line.trim();
            
            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            
            // Convert to lowercase and add to set
            words.insert(line.to_lowercase());
        }
        
        // Check if file was empty (no words after filtering)
        if words.is_empty() {
            return Err(NavigationError::VocabularyLoadFailed {
                vocabulary_name: name.to_string(),
                reason: "File is empty or contains no valid words".to_string(),
            });
        }
        
        // Validate minimum word count if specified
        if let Some(min) = min_words {
            if words.len() < min {
                return Err(NavigationError::VocabularyValidationFailed {
                    vocabulary_name: name.to_string(),
                    reason: format!(
                        "Vocabulary has {} words, but requires at least {}",
                        words.len(),
                        min
                    ),
                });
            }
        }
        
        Ok(words)
    }
    
    /// Extract keywords from a query
    ///
    /// # Arguments
    /// * `query` - The query string to extract keywords from
    ///
    /// # Returns
    /// Returns a vector of keywords (up to 50) in order of appearance
    ///
    /// # Processing Steps
    /// 1. Tokenize on whitespace
    /// 2. Strip punctuation
    /// 3. Convert to lowercase
    /// 4. Filter stopwords
    /// 5. Limit to 50 keywords
    ///
    /// # Requirements
    /// - 1.1: Tokenize query into keywords
    /// - 1.2: Strip punctuation
    /// - 1.3: Normalize to lowercase
    /// - 1.4: Filter stopwords
    /// - 1.5: Limit to 50 keywords
    ///
    /// # Performance
    /// - Target: <5ms (Requirement 1.1)
    pub fn extract_keywords(&self, query: &str) -> Vec<String> {
        query
            .split_whitespace()
            .map(|word| {
                // Strip punctuation from both ends
                word.trim_matches(|c: char| !c.is_alphanumeric())
                    .to_lowercase()
            })
            .filter(|word| {
                // Filter out empty strings and stopwords
                !word.is_empty() && !self.stopwords.contains(word)
            })
            .take(50) // Limit to 50 keywords (Requirement 1.5)
            .collect()
    }
    
    /// Classify keywords into emotional and technical indicators
    ///
    /// # Arguments
    /// * `keywords` - List of keywords to classify
    ///
    /// # Returns
    /// Returns a tuple of (emotional_indicators, technical_indicators)
    ///
    /// # Requirements
    /// - 1.6: Identify emotional indicators
    /// - 1.7: Identify technical indicators
    ///
    /// # Performance
    /// - O(n) where n is number of keywords
    /// - Uses HashSet lookups for O(1) classification per keyword
    pub fn classify_indicators(&self, keywords: &[String]) -> (Vec<String>, Vec<String>) {
        let mut emotional_indicators = Vec::new();
        let mut technical_indicators = Vec::new();
        
        for keyword in keywords {
            // Check if keyword is an emotional indicator
            if self.emotional_vocabulary.contains(keyword) {
                emotional_indicators.push(keyword.clone());
            }
            
            // Check if keyword is a technical indicator
            if self.technical_vocabulary.contains(keyword) {
                technical_indicators.push(keyword.clone());
            }
        }
        
        (emotional_indicators, technical_indicators)
    }
    
    /// Classify question type based on indicators and keywords
    ///
    /// # Arguments
    /// * `keywords` - All keywords from query
    /// * `emotional_indicators` - Emotional indicator keywords
    /// * `technical_indicators` - Technical indicator keywords
    ///
    /// # Returns
    /// Returns the classified QuestionType
    ///
    /// # Classification Logic
    /// 1. If >60% indicators are emotional → Emotional
    /// 2. If >60% indicators are technical → Technical
    /// 3. If contains philosophical keywords → Philosophical
    /// 4. If contains interrogative words → Factual
    /// 5. If ~50/50 emotional/technical → Mixed
    /// 6. Default → Factual
    ///
    /// # Requirements
    /// - 1.8: Classify as Emotional if >60% emotional
    /// - 1.9: Classify as Technical if >60% technical
    /// - 1.10: Classify as Philosophical if contains philosophical keywords
    /// - 1.11: Classify as Factual if contains interrogative words
    /// - 1.12: Classify as Mixed if balanced indicators
    pub fn classify_question_type(
        &self,
        keywords: &[String],
        emotional_indicators: &[String],
        technical_indicators: &[String],
    ) -> QuestionType {
        let total_indicators = emotional_indicators.len() + technical_indicators.len();
        
        // Check for philosophical keywords first
        if keywords.iter().any(|k| self.philosophical_keywords.contains(k)) {
            return QuestionType::Philosophical;
        }
        
        // If we have indicators, classify based on percentage
        if total_indicators > 0 {
            let emotional_pct = emotional_indicators.len() as f32 / total_indicators as f32;
            let technical_pct = technical_indicators.len() as f32 / total_indicators as f32;
            
            // >60% emotional
            if emotional_pct > 0.6 {
                return QuestionType::Emotional;
            }
            
            // >60% technical
            if technical_pct > 0.6 {
                return QuestionType::Technical;
            }
            
            // Balanced (both around 50%)
            if (emotional_pct - 0.5).abs() < 0.2 && (technical_pct - 0.5).abs() < 0.2 {
                return QuestionType::Mixed;
            }
        }
        
        // Check for interrogative words (factual questions)
        if keywords.iter().any(|k| self.interrogative_words.contains(k)) {
            return QuestionType::Factual;
        }
        
        // Default to Factual
        QuestionType::Factual
    }
    
    /// Assign urgency level based on keywords
    ///
    /// # Arguments
    /// * `keywords` - Keywords from query
    ///
    /// # Returns
    /// Returns the assigned UrgencyLevel
    ///
    /// # Classification Logic
    /// - High: Contains high urgency keywords (urgent, emergency, critical, etc.)
    /// - Medium: Contains medium urgency keywords (need, help)
    /// - Low: Default for general queries
    ///
    /// # Requirements
    /// - 1.13: Assign High urgency for urgent keywords
    /// - 1.14: Assign Medium urgency for moderate keywords
    /// - 1.15: Assign Low urgency by default
    pub fn assign_urgency(&self, keywords: &[String]) -> UrgencyLevel {
        // Check for high urgency keywords
        if keywords.iter().any(|k| self.high_urgency_keywords.contains(k)) {
            return UrgencyLevel::High;
        }
        
        // Check for medium urgency keywords
        let medium_urgency_keywords = ["need", "help"];
        if keywords.iter().any(|k| medium_urgency_keywords.contains(&k.as_str())) {
            return UrgencyLevel::Medium;
        }
        
        // Default to low urgency
        UrgencyLevel::Low
    }
    
    /// Estimate query frequency based on urgency, question type, and emotional intensity
    ///
    /// # Arguments
    /// * `urgency` - Urgency level
    /// * `question_type` - Question type
    /// * `emotional_indicators` - Emotional indicator keywords
    ///
    /// # Returns
    /// Returns estimated frequency in Hz (0.1-4.5 range)
    ///
    /// # Frequency Calculation
    /// 1. Start with base frequency from urgency level
    /// 2. Apply question type adjustments
    /// 3. Apply high-intensity emotional adjustments
    /// 4. Clamp to 0.1-4.5 Hz range
    ///
    /// # Requirements
    /// - 15.1: Base frequency from urgency (Low=0.5, Medium=2.0, High=3.5)
    /// - 15.2: Philosophical adjustment (-0.5 Hz)
    /// - 15.3: Technical adjustment (+0.5 Hz)
    /// - 15.6: High-intensity emotional adjustment (+1.0 Hz)
    /// - 15.8: Clamp to 0.1-4.5 Hz range
    pub fn estimate_frequency(
        &self,
        urgency: UrgencyLevel,
        question_type: QuestionType,
        emotional_indicators: &[String],
    ) -> f32 {
        // Start with base frequency from urgency level
        let mut frequency = urgency.base_frequency();
        
        // Apply question type adjustments
        match question_type {
            QuestionType::Philosophical => frequency -= 0.5,
            QuestionType::Technical => frequency += 0.5,
            _ => {}
        }
        
        // Check for high-intensity emotional keywords
        let has_high_intensity = emotional_indicators
            .iter()
            .any(|k| self.high_intensity_emotional.contains(k));
        
        if has_high_intensity {
            frequency += 1.0;
        }
        
        // Clamp to valid range (0.1-4.5 Hz)
        frequency.clamp(0.1, 4.5)
    }
    
    /// Estimate query complexity based on keyword count and indicator diversity
    ///
    /// # Arguments
    /// * `keywords` - All keywords from query
    /// * `emotional_indicators` - Emotional indicator keywords
    /// * `technical_indicators` - Technical indicator keywords
    ///
    /// # Returns
    /// Returns complexity score (0.0-5.0 range)
    ///
    /// # Complexity Calculation
    /// - Keyword count contributes to complexity
    /// - Indicator diversity (both emotional and technical) increases complexity
    /// - More indicators = higher complexity
    ///
    /// # Requirements
    /// - 1.17: Estimate complexity score
    pub fn estimate_complexity(
        &self,
        keywords: &[String],
        emotional_indicators: &[String],
        technical_indicators: &[String],
    ) -> f32 {
        // Base complexity from keyword count (normalized to 0-3 range)
        let keyword_complexity = (keywords.len() as f32 / 10.0).min(3.0);
        
        // Indicator diversity adds complexity
        let has_emotional = !emotional_indicators.is_empty();
        let has_technical = !technical_indicators.is_empty();
        
        let diversity_complexity = match (has_emotional, has_technical) {
            (true, true) => 2.0,  // Mixed indicators = high complexity
            (true, false) | (false, true) => 1.0,  // Single type = medium complexity
            (false, false) => 0.0,  // No indicators = low complexity
        };
        
        // Combine and clamp to 0.0-5.0 range
        (keyword_complexity + diversity_complexity).clamp(0.0, 5.0)
    }
    
    /// Analyze a query and extract all metadata
    ///
    /// # Arguments
    /// * `query` - The query string to analyze
    ///
    /// # Returns
    /// Returns `Ok(QueryAnalysis)` with complete analysis, or `NavigationError` if:
    /// - Query is empty
    /// - Query exceeds maximum length
    ///
    /// # Requirements
    /// - 1.1-1.17: Complete query analysis
    /// - 15.1-15.8: Frequency estimation
    ///
    /// # Performance
    /// - Target: <5ms total (Requirement 1.1)
    pub fn analyze(&self, query: &str) -> Result<QueryAnalysis, NavigationError> {
        // Extract keywords
        let keywords = self.extract_keywords(query);
        
        // Classify indicators
        let (emotional_indicators, technical_indicators) = self.classify_indicators(&keywords);
        
        // Classify question type
        let question_type = self.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        
        // Assign urgency level
        let urgency_level = self.assign_urgency(&keywords);
        
        // Estimate frequency
        let estimated_frequency = self.estimate_frequency(urgency_level, question_type, &emotional_indicators);
        
        // Estimate complexity
        let estimated_complexity = self.estimate_complexity(&keywords, &emotional_indicators, &technical_indicators);
        
        Ok(QueryAnalysis {
            raw_query: query.to_string(),
            keywords,
            estimated_complexity,
            emotional_indicators,
            technical_indicators,
            question_type,
            urgency_level,
            estimated_frequency,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;
    
    // Helper function to create test vocabulary files
    fn create_test_vocab_files() -> (String, String, String) {
        // Return paths to actual vocabulary files
        (
            "data/emotional.txt".to_string(),
            "data/technical.txt".to_string(),
            "data/stopwords.txt".to_string(),
        )
    }
    
    // Helper to create a temporary vocabulary file
    fn create_temp_vocab_file(words: &[&str]) -> NamedTempFile {
        let mut file = NamedTempFile::new().unwrap();
        for word in words {
            writeln!(file, "{}", word).unwrap();
        }
        file.flush().unwrap();
        file
    }
    
    // Task 3.2: Tests for vocabulary loading (RED phase)
    // These tests define the expected behavior before implementation
    
    #[test]
    fn test_load_vocabulary_reads_file_and_returns_hashset() {
        // Test that load_vocabulary reads a file and returns a HashSet
        let words = vec!["happy", "sad", "angry", "joyful", "anxious"];
        let file = create_temp_vocab_file(&words);
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "test",
            None,
        );
        
        assert!(result.is_ok(), "Should successfully load vocabulary");
        let vocab = result.unwrap();
        assert_eq!(vocab.len(), 5, "Should contain 5 words");
        assert!(vocab.contains("happy"), "Should contain 'happy'");
        assert!(vocab.contains("sad"), "Should contain 'sad'");
        assert!(vocab.contains("angry"), "Should contain 'angry'");
    }
    
    #[test]
    fn test_load_vocabulary_validates_minimum_word_count() {
        // Test that validation fails if vocabulary has fewer than required words
        let words = vec!["happy", "sad"]; // Only 2 words
        let file = create_temp_vocab_file(&words);
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "emotional",
            Some(100), // Require 100 words
        );
        
        assert!(result.is_err(), "Should fail validation with too few words");
        match result.unwrap_err() {
            NavigationError::VocabularyValidationFailed { vocabulary_name, reason } => {
                assert_eq!(vocabulary_name, "emotional");
                assert!(reason.contains("100"), "Error should mention required count");
            }
            _ => panic!("Wrong error type"),
        }
    }
    
    #[test]
    fn test_load_vocabulary_handles_missing_file() {
        // Test that missing file returns appropriate error
        let result = QueryAnalyzer::load_vocabulary(
            "/nonexistent/path/to/file.txt",
            "test",
            None,
        );
        
        assert!(result.is_err(), "Should fail for missing file");
        match result.unwrap_err() {
            NavigationError::VocabularyLoadFailed { vocabulary_name, .. } => {
                assert_eq!(vocabulary_name, "test");
            }
            _ => panic!("Wrong error type"),
        }
    }
    
    #[test]
    fn test_load_vocabulary_handles_empty_file() {
        // Test that empty file returns appropriate error
        let file = create_temp_vocab_file(&[]);
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "empty",
            None,
        );
        
        assert!(result.is_err(), "Should fail for empty file");
        match result.unwrap_err() {
            NavigationError::VocabularyLoadFailed { vocabulary_name, reason } => {
                assert_eq!(vocabulary_name, "empty");
                assert!(reason.contains("empty"), "Error should mention empty file");
            }
            _ => panic!("Wrong error type"),
        }
    }
    
    #[test]
    fn test_load_vocabulary_ignores_comments() {
        // Test that lines starting with # are ignored
        let mut file = NamedTempFile::new().unwrap();
        writeln!(file, "# This is a comment").unwrap();
        writeln!(file, "happy").unwrap();
        writeln!(file, "# Another comment").unwrap();
        writeln!(file, "sad").unwrap();
        file.flush().unwrap();
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "test",
            None,
        );
        
        assert!(result.is_ok());
        let vocab = result.unwrap();
        assert_eq!(vocab.len(), 2, "Should only contain non-comment lines");
        assert!(vocab.contains("happy"));
        assert!(vocab.contains("sad"));
    }
    
    #[test]
    fn test_load_vocabulary_ignores_empty_lines() {
        // Test that empty lines are ignored
        let mut file = NamedTempFile::new().unwrap();
        writeln!(file, "happy").unwrap();
        writeln!(file, "").unwrap();
        writeln!(file, "sad").unwrap();
        writeln!(file, "   ").unwrap(); // Whitespace only
        writeln!(file, "angry").unwrap();
        file.flush().unwrap();
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "test",
            None,
        );
        
        assert!(result.is_ok());
        let vocab = result.unwrap();
        assert_eq!(vocab.len(), 3, "Should ignore empty lines");
    }
    
    #[test]
    fn test_load_vocabulary_converts_to_lowercase() {
        // Test that words are converted to lowercase
        let words = vec!["HAPPY", "Sad", "AnGrY"];
        let file = create_temp_vocab_file(&words);
        
        let result = QueryAnalyzer::load_vocabulary(
            file.path(),
            "test",
            None,
        );
        
        assert!(result.is_ok());
        let vocab = result.unwrap();
        assert!(vocab.contains("happy"), "Should contain lowercase 'happy'");
        assert!(vocab.contains("sad"), "Should contain lowercase 'sad'");
        assert!(vocab.contains("angry"), "Should contain lowercase 'angry'");
        assert!(!vocab.contains("HAPPY"), "Should not contain uppercase");
    }
    
    #[test]
    fn test_query_analyzer_new_loads_all_vocabularies() {
        // Test that QueryAnalyzer::new successfully loads all vocabularies
        let (emotional, technical, stopwords) = create_test_vocab_files();
        
        let result = QueryAnalyzer::new(&emotional, &technical, &stopwords);
        
        assert!(result.is_ok(), "Should successfully create QueryAnalyzer");
        let analyzer = result.unwrap();
        
        // Verify vocabularies are loaded (will check counts once implemented)
        assert!(
            analyzer.emotional_vocabulary.len() >= 100,
            "Emotional vocabulary should have at least 100 words"
        );
        assert!(
            analyzer.technical_vocabulary.len() >= 100,
            "Technical vocabulary should have at least 100 words"
        );
        assert!(
            analyzer.stopwords.len() > 0,
            "Stopwords should be loaded"
        );
    }
    
    #[test]
    fn test_query_analyzer_new_validates_emotional_vocabulary() {
        // Test that QueryAnalyzer::new validates emotional vocabulary has 100+ words
        let words = vec!["happy", "sad"]; // Only 2 words
        let file = create_temp_vocab_file(&words);
        let (_, technical, stopwords) = create_test_vocab_files();
        
        let result = QueryAnalyzer::new(file.path(), Path::new(&technical), Path::new(&stopwords));
        
        assert!(result.is_err(), "Should fail with insufficient emotional words");
        match result.unwrap_err() {
            NavigationError::VocabularyValidationFailed { vocabulary_name, .. } => {
                assert_eq!(vocabulary_name, "emotional");
            }
            _ => panic!("Wrong error type"),
        }
    }
    
    #[test]
    fn test_query_analyzer_new_validates_technical_vocabulary() {
        // Test that QueryAnalyzer::new validates technical vocabulary has 100+ words
        let words = vec!["algorithm", "code"]; // Only 2 words
        let file = create_temp_vocab_file(&words);
        let (emotional, _, stopwords) = create_test_vocab_files();
        
        let result = QueryAnalyzer::new(Path::new(&emotional), file.path(), Path::new(&stopwords));
        
        assert!(result.is_err(), "Should fail with insufficient technical words");
        match result.unwrap_err() {
            NavigationError::VocabularyValidationFailed { vocabulary_name, .. } => {
                assert_eq!(vocabulary_name, "technical");
            }
            _ => panic!("Wrong error type"),
        }
    }
    
    // Task 3.4: Tests for keyword extraction (RED phase)
    
    #[test]
    fn test_extract_keywords_basic_tokenization() {
        // Test basic whitespace tokenization
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("hello world");
        assert_eq!(keywords, vec!["hello", "world"]);
    }
    
    #[test]
    fn test_extract_keywords_handles_punctuation() {
        // Test punctuation is stripped
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("hello, world!");
        assert_eq!(keywords, vec!["hello", "world"]);
        
        let keywords = analyzer.extract_keywords("debugging, testing, coding!");
        assert_eq!(keywords.len(), 3);
        assert!(keywords.contains(&"debugging".to_string()));
        assert!(keywords.contains(&"testing".to_string()));
        assert!(keywords.contains(&"coding".to_string()));
    }
    
    #[test]
    fn test_extract_keywords_lowercase_normalization() {
        // Test all keywords are converted to lowercase
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("Hello WORLD");
        assert_eq!(keywords, vec!["hello", "world"]);
    }
    
    #[test]
    fn test_extract_keywords_filters_stopwords() {
        // Test stopwords are filtered out
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("the cat is on the mat");
        // "the", "is", "on" should be filtered as stopwords
        assert!(keywords.contains(&"cat".to_string()));
        assert!(keywords.contains(&"mat".to_string()));
        assert!(!keywords.contains(&"the".to_string()));
        assert!(!keywords.contains(&"is".to_string()));
        assert!(!keywords.contains(&"on".to_string()));
    }
    
    #[test]
    fn test_extract_keywords_max_50_limit() {
        // Test max 50 keywords limit (Requirement 1.5)
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        // Create a query with 100 unique words
        let words: Vec<String> = (0..100).map(|i| format!("word{}", i)).collect();
        let query = words.join(" ");
        
        let keywords = analyzer.extract_keywords(&query);
        assert!(
            keywords.len() <= 50,
            "Should limit to 50 keywords, got {}",
            keywords.len()
        );
    }
    
    #[test]
    fn test_extract_keywords_empty_query() {
        // Test empty query returns empty list
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("");
        assert_eq!(keywords.len(), 0);
    }
    
    #[test]
    fn test_extract_keywords_performance() {
        // Test extraction completes in <5ms (Requirement 1.1)
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "I am feeling anxious about debugging this complex algorithm implementation";
        
        let start = std::time::Instant::now();
        let _keywords = analyzer.extract_keywords(query);
        let duration = start.elapsed();
        
        assert!(
            duration.as_millis() < 5,
            "Keyword extraction took {}ms, should be <5ms",
            duration.as_millis()
        );
    }
    
    #[test]
    fn test_extract_keywords_preserves_order() {
        // Test keywords maintain order of appearance
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("zebra apple banana");
        assert_eq!(keywords, vec!["zebra", "apple", "banana"]);
    }
    
    #[test]
    fn test_extract_keywords_handles_multiple_spaces() {
        // Test multiple spaces are handled correctly
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("hello    world");
        assert_eq!(keywords, vec!["hello", "world"]);
    }
    
    #[test]
    fn test_extract_keywords_handles_newlines() {
        // Test newlines are treated as whitespace
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = analyzer.extract_keywords("hello\nworld");
        assert_eq!(keywords, vec!["hello", "world"]);
    }
    
    // Task 3.6: Tests for indicator classification (RED phase)
    
    #[test]
    fn test_classify_indicators_emotional() {
        // Test emotional indicator detection
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["anxious".to_string(), "happy".to_string()];
        let (emotional_indicators, technical_indicators) = analyzer.classify_indicators(&keywords);
        
        assert_eq!(emotional_indicators.len(), 2);
        assert!(emotional_indicators.contains(&"anxious".to_string()));
        assert!(emotional_indicators.contains(&"happy".to_string()));
        assert_eq!(technical_indicators.len(), 0);
    }
    
    #[test]
    fn test_classify_indicators_technical() {
        // Test technical indicator detection
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["algorithm".to_string(), "database".to_string()];
        let (emotional_indicators, technical_indicators) = analyzer.classify_indicators(&keywords);
        
        assert_eq!(emotional_indicators.len(), 0);
        assert_eq!(technical_indicators.len(), 2);
        assert!(technical_indicators.contains(&"algorithm".to_string()));
        assert!(technical_indicators.contains(&"database".to_string()));
    }
    
    #[test]
    fn test_classify_indicators_mixed() {
        // Test mixed indicators
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["anxious".to_string(), "algorithm".to_string()];
        let (emotional_indicators, technical_indicators) = analyzer.classify_indicators(&keywords);
        
        assert_eq!(emotional_indicators.len(), 1);
        assert_eq!(technical_indicators.len(), 1);
        assert!(emotional_indicators.contains(&"anxious".to_string()));
        assert!(technical_indicators.contains(&"algorithm".to_string()));
    }
    
    #[test]
    fn test_classify_indicators_none() {
        // Test no indicators
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["random".to_string(), "words".to_string()];
        let (emotional_indicators, technical_indicators) = analyzer.classify_indicators(&keywords);
        
        assert_eq!(emotional_indicators.len(), 0);
        assert_eq!(technical_indicators.len(), 0);
    }
    
    #[test]
    fn test_classify_indicators_case_insensitive() {
        // Test case insensitivity
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        // Keywords should already be lowercase from extract_keywords
        let keywords = vec!["anxious".to_string(), "algorithm".to_string()];
        let (emotional_indicators, technical_indicators) = analyzer.classify_indicators(&keywords);
        
        assert_eq!(emotional_indicators.len(), 1);
        assert_eq!(technical_indicators.len(), 1);
    }
    
    // Task 3.8: Tests for question type classification (RED phase)
    
    #[test]
    fn test_classify_question_type_emotional() {
        // Test Emotional: 60% emotional indicators
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec!["anxious".to_string(), "worried".to_string(), "stressed".to_string()];
        let technical_indicators = vec!["code".to_string()];
        let keywords = vec!["anxious".to_string(), "worried".to_string(), "stressed".to_string(), "code".to_string(), "help".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        assert_eq!(question_type, QuestionType::Emotional);
    }
    
    #[test]
    fn test_classify_question_type_technical() {
        // Test Technical: 60% technical indicators
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec!["happy".to_string()];
        let technical_indicators = vec!["algorithm".to_string(), "database".to_string(), "code".to_string()];
        let keywords = vec!["algorithm".to_string(), "database".to_string(), "code".to_string(), "happy".to_string(), "help".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        assert_eq!(question_type, QuestionType::Technical);
    }
    
    #[test]
    fn test_classify_question_type_philosophical() {
        // Test Philosophical: contains philosophical keywords
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec![];
        let technical_indicators = vec![];
        let keywords = vec!["meaning".to_string(), "purpose".to_string(), "existence".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        assert_eq!(question_type, QuestionType::Philosophical);
    }
    
    #[test]
    fn test_classify_question_type_factual() {
        // Test Factual: contains interrogative words
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec![];
        let technical_indicators = vec![];
        let keywords = vec!["what".to_string(), "definition".to_string(), "explain".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        assert_eq!(question_type, QuestionType::Factual);
    }
    
    #[test]
    fn test_classify_question_type_mixed() {
        // Test Mixed: 50% emotional, 50% technical
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec!["anxious".to_string(), "worried".to_string()];
        let technical_indicators = vec!["algorithm".to_string(), "code".to_string()];
        let keywords = vec!["anxious".to_string(), "worried".to_string(), "algorithm".to_string(), "code".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        assert_eq!(question_type, QuestionType::Mixed);
    }
    
    #[test]
    fn test_classify_question_type_edge_cases() {
        // Test edge cases: no indicators, all stopwords
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec![];
        let technical_indicators = vec![];
        let keywords = vec!["random".to_string(), "words".to_string()];
        
        let question_type = analyzer.classify_question_type(&keywords, &emotional_indicators, &technical_indicators);
        // Should default to Factual when no clear type
        assert_eq!(question_type, QuestionType::Factual);
    }
    
    // Task 3.10: Tests for urgency detection (RED phase)
    
    #[test]
    fn test_assign_urgency_high() {
        // Test High urgency
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["urgent".to_string(), "help".to_string(), "needed".to_string()];
        let urgency = analyzer.assign_urgency(&keywords);
        assert_eq!(urgency, UrgencyLevel::High);
    }
    
    #[test]
    fn test_assign_urgency_medium() {
        // Test Medium urgency
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["need".to_string(), "help".to_string()];
        let urgency = analyzer.assign_urgency(&keywords);
        assert_eq!(urgency, UrgencyLevel::Medium);
    }
    
    #[test]
    fn test_assign_urgency_low() {
        // Test Low urgency
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["curious".to_string(), "wondering".to_string()];
        let urgency = analyzer.assign_urgency(&keywords);
        assert_eq!(urgency, UrgencyLevel::Low);
    }
    
    #[test]
    fn test_assign_urgency_multiple_keywords() {
        // Test multiple urgency keywords
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["emergency".to_string(), "critical".to_string(), "immediately".to_string()];
        let urgency = analyzer.assign_urgency(&keywords);
        assert_eq!(urgency, UrgencyLevel::High);
    }
    
    #[test]
    fn test_assign_urgency_case_insensitive() {
        // Test case insensitivity (keywords should already be lowercase)
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["urgent".to_string()];
        let urgency = analyzer.assign_urgency(&keywords);
        assert_eq!(urgency, UrgencyLevel::High);
    }
    
    // Task 3.12: Tests for frequency estimation (RED phase)
    
    #[test]
    fn test_estimate_frequency_base_low() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Low,
            QuestionType::Factual,
            &[],
        );
        assert_eq!(freq, 0.5); // Base frequency for Low
    }
    
    #[test]
    fn test_estimate_frequency_base_medium() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Medium,
            QuestionType::Factual,
            &[],
        );
        assert_eq!(freq, 2.0); // Base frequency for Medium
    }
    
    #[test]
    fn test_estimate_frequency_base_high() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::High,
            QuestionType::Factual,
            &[],
        );
        assert_eq!(freq, 3.5); // Base frequency for High
    }
    
    #[test]
    fn test_estimate_frequency_philosophical_adjustment() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Medium,
            QuestionType::Philosophical,
            &[],
        );
        assert_eq!(freq, 1.5); // 2.0 - 0.5 for philosophical
    }
    
    #[test]
    fn test_estimate_frequency_technical_adjustment() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Medium,
            QuestionType::Technical,
            &[],
        );
        assert_eq!(freq, 2.5); // 2.0 + 0.5 for technical
    }
    
    #[test]
    fn test_estimate_frequency_high_intensity_emotional() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let emotional_indicators = vec!["terrified".to_string(), "desperate".to_string()];
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Medium,
            QuestionType::Emotional,
            &emotional_indicators,
        );
        assert_eq!(freq, 3.0); // 2.0 + 1.0 for high-intensity emotional
    }
    
    #[test]
    fn test_estimate_frequency_clamping_min() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        // Low urgency (0.5) + Philosophical (-0.5) = 0.0, should clamp to 0.1
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Low,
            QuestionType::Philosophical,
            &[],
        );
        assert!(freq >= 0.1, "Frequency should be clamped to minimum 0.1, got {}", freq);
    }
    
    #[test]
    fn test_estimate_frequency_clamping_max() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        // High urgency (3.5) + Technical (+0.5) + High-intensity emotional (+1.0) = 5.0, should clamp to 4.5
        let emotional_indicators = vec!["terrified".to_string()];
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::High,
            QuestionType::Technical,
            &emotional_indicators,
        );
        assert!(freq <= 4.5, "Frequency should be clamped to maximum 4.5, got {}", freq);
    }
    
    #[test]
    fn test_estimate_frequency_combined_adjustments() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        // Medium (2.0) + Technical (+0.5) = 2.5
        let freq = analyzer.estimate_frequency(
            UrgencyLevel::Medium,
            QuestionType::Technical,
            &[],
        );
        assert_eq!(freq, 2.5);
    }
    
    // Task 3.14: Tests for complexity scoring (RED phase)
    
    #[test]
    fn test_estimate_complexity_low() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["hello".to_string(), "world".to_string()];
        let emotional_indicators = vec![];
        let technical_indicators = vec![];
        
        let complexity = analyzer.estimate_complexity(&keywords, &emotional_indicators, &technical_indicators);
        assert!(complexity >= 0.0 && complexity <= 2.0, "Low complexity should be ~1.0, got {}", complexity);
    }
    
    #[test]
    fn test_estimate_complexity_medium() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["anxious".to_string(), "algorithm".to_string(), "debugging".to_string(), "worried".to_string(), "code".to_string()];
        let emotional_indicators = vec!["anxious".to_string(), "worried".to_string()];
        let technical_indicators = vec!["algorithm".to_string(), "debugging".to_string(), "code".to_string()];
        
        let complexity = analyzer.estimate_complexity(&keywords, &emotional_indicators, &technical_indicators);
        assert!(complexity >= 2.0 && complexity <= 4.0, "Medium complexity should be ~3.0, got {}", complexity);
    }
    
    #[test]
    fn test_estimate_complexity_high() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords: Vec<String> = (0..20).map(|i| format!("word{}", i)).collect();
        let emotional_indicators: Vec<String> = (0..5).map(|i| format!("emotional{}", i)).collect();
        let technical_indicators: Vec<String> = (0..5).map(|i| format!("technical{}", i)).collect();
        
        let complexity = analyzer.estimate_complexity(&keywords, &emotional_indicators, &technical_indicators);
        assert!(complexity >= 3.0 && complexity <= 5.0, "High complexity should be ~5.0, got {}", complexity);
    }
    
    #[test]
    fn test_estimate_complexity_range() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let keywords = vec!["test".to_string()];
        let emotional_indicators = vec![];
        let technical_indicators = vec![];
        
        let complexity = analyzer.estimate_complexity(&keywords, &emotional_indicators, &technical_indicators);
        assert!(complexity >= 0.0 && complexity <= 5.0, "Complexity should be in range 0.0-5.0, got {}", complexity);
    }
    
    // Task 3.16: Tests for full query analysis (RED phase)
    
    #[test]
    fn test_analyze_complete_flow() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "I am feeling anxious about debugging this complex algorithm";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.raw_query, query);
        assert!(result.keywords.len() > 0, "Should extract keywords");
        assert!(result.emotional_indicators.len() > 0, "Should identify emotional indicators");
        assert!(result.technical_indicators.len() > 0, "Should identify technical indicators");
        assert!(result.estimated_frequency >= 0.1 && result.estimated_frequency <= 4.5);
        assert!(result.estimated_complexity >= 0.0 && result.estimated_complexity <= 5.0);
    }
    
    #[test]
    fn test_analyze_emotional_query() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "I feel anxious worried stressed overwhelmed";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.question_type, QuestionType::Emotional);
        assert!(result.emotional_indicators.len() >= 3);
    }
    
    #[test]
    fn test_analyze_technical_query() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "debug algorithm database code implementation";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.question_type, QuestionType::Technical);
        assert!(result.technical_indicators.len() >= 3);
    }
    
    #[test]
    fn test_analyze_philosophical_query() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "what is the meaning of consciousness and existence";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.question_type, QuestionType::Philosophical);
    }
    
    #[test]
    fn test_analyze_factual_query() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "what is definition explain concept";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.question_type, QuestionType::Factual);
    }
    
    #[test]
    fn test_analyze_mixed_query() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "anxious worried algorithm database";
        let result = analyzer.analyze(query).unwrap();
        
        assert_eq!(result.question_type, QuestionType::Mixed);
    }
    
    #[test]
    fn test_analyze_performance() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let query = "I am feeling anxious about debugging this complex algorithm implementation";
        
        let start = std::time::Instant::now();
        let _result = analyzer.analyze(query).unwrap();
        let duration = start.elapsed();
        
        assert!(
            duration.as_millis() < 5,
            "Full analysis took {}ms, should be <5ms",
            duration.as_millis()
        );
    }
    
    #[test]
    fn test_analyze_various_query_types() {
        let (emotional, technical, stopwords) = create_test_vocab_files();
        let analyzer = QueryAnalyzer::new(&emotional, &technical, &stopwords).unwrap();
        
        let queries = vec![
            "I feel happy",
            "debug code algorithm",
            "meaning of existence",
            "what is this",
            "anxious about algorithm",
        ];
        
        for query in queries {
            let result = analyzer.analyze(query);
            assert!(result.is_ok(), "Should successfully analyze: {}", query);
        }
    }
}
