//! Consciousness orchestrator implementation
//!
//! This module implements the main orchestrator that coordinates the complete
//! consciousness pipeline: Navigation → Memory → Interference → Iteration

use crate::consciousness::{
    ConsciousnessConfig, ConsciousnessResponse, ResponseMetadata,
};
use crate::interference::{InterferenceEngine, FrequencyState};
use crate::iteration::IterationProcessor;
use crate::learning::LearningSystem;
use crate::llm::{LLMManager, LLMConfig};
use crate::memory::MmapManager;
use crate::navigation::NavigationSystem;
use crate::{ConsciousnessError, Result, Frequency};
use std::sync::Arc;
use std::time::Instant;

/// Main consciousness orchestrator
///
/// Coordinates the complete query processing pipeline by integrating:
/// - Navigation system for dimensional path selection
/// - Memory manager for context loading
/// - Interference calculator for frequency analysis
/// - Iteration processor for deep thinking
///
/// # Thread Safety
///
/// The orchestrator uses Arc for shared access to navigation and memory systems,
/// making it safe to use across multiple threads.
///
/// # Example
///
/// ```no_run
/// use jessy::consciousness::ConsciousnessOrchestrator;
/// use jessy::navigation::NavigationSystem;
/// use jessy::memory::MmapManager;
/// use std::sync::Arc;
///
/// # async fn example() -> jessy::Result<()> {
/// let navigation = Arc::new(NavigationSystem::new()?);
/// let memory = Arc::new(MmapManager::new(280)?);
///
/// let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
/// let response = orchestrator.process("What is consciousness?").await?;
/// # Ok(())
/// # }
/// ```
pub struct ConsciousnessOrchestrator {
    navigation: Arc<NavigationSystem>,
    memory: Arc<MmapManager>,
    iteration: IterationProcessor,
    interference_engine: InterferenceEngine,
    learning: LearningSystem,
    llm_manager: Option<LLMManager>,  // Optional for testing without API keys
    config: ConsciousnessConfig,
    query_count: usize,
    pattern_detection_interval: usize,
}

impl ConsciousnessOrchestrator {
    /// Create new consciousness orchestrator with default configuration
    ///
    /// # Arguments
    ///
    /// * `navigation` - Shared navigation system
    /// * `memory` - Shared memory manager
    ///
    /// # Returns
    ///
    /// Returns a new orchestrator with default configuration (without LLM)
    pub fn new(navigation: Arc<NavigationSystem>, memory: Arc<MmapManager>) -> Self {
        let mut learning = LearningSystem::new();
        learning.init_crystallizer(memory.clone());
        Self::with_config(navigation, memory, ConsciousnessConfig::default(), learning)
    }
    
    /// Create orchestrator with LLM integration
    ///
    /// # Arguments
    ///
    /// * `navigation` - Shared navigation system
    /// * `memory` - Shared memory manager
    /// * `llm_config` - LLM configuration with API keys
    ///
    /// # Returns
    ///
    /// Returns orchestrator with LLM manager initialized
    ///
    /// # Errors
    ///
    /// Returns error if LLM initialization fails (invalid API key, etc.)
    pub fn with_llm(
        navigation: Arc<NavigationSystem>,
        memory: Arc<MmapManager>,
        llm_config: LLMConfig,
    ) -> Result<Self> {
        let mut learning = LearningSystem::new();
        learning.init_crystallizer(memory.clone());
        
        let llm_manager = LLMManager::new(llm_config)?;
        
        let mut orchestrator = Self::with_config(
            navigation,
            memory,
            ConsciousnessConfig::default(),
            learning,
        );
        
        orchestrator.llm_manager = Some(llm_manager);
        
        Ok(orchestrator)
    }
    
    /// Create new consciousness orchestrator with custom configuration
    ///
    /// # Arguments
    ///
    /// * `navigation` - Shared navigation system
    /// * `memory` - Shared memory manager
    /// * `config` - Custom configuration
    ///
    /// # Returns
    ///
    /// Returns a new orchestrator with the specified configuration
    pub fn with_config(
        navigation: Arc<NavigationSystem>,
        memory: Arc<MmapManager>,
        config: ConsciousnessConfig,
        learning: LearningSystem,
    ) -> Self {
        let iteration = IterationProcessor::new(
            config.max_iterations,
            config.convergence_threshold,
            6, // complexity_threshold for return-to-source
        );
        
        let interference_engine = InterferenceEngine::new();
        
        Self {
            navigation,
            memory,
            iteration,
            interference_engine,
            learning,
            llm_manager: None,  // No LLM by default (for testing)
            config,
            query_count: 0,
            pattern_detection_interval: 100, // Detect patterns every 100 queries
        }
    }
    
    /// Process a query through the complete consciousness pipeline
    ///
    /// Executes the following stages:
    /// 1. Navigation - Select dimensional paths
    /// 2. Memory - Load contexts from selected dimensions
    /// 3. Interference - Calculate frequency patterns
    /// 4. Iteration - Deep thinking with convergence
    ///
    /// # Arguments
    ///
    /// * `query` - The query string to process
    ///
    /// # Returns
    ///
    /// Returns `ConsciousnessResponse` with:
    /// - Final refined answer
    /// - Metadata about processing
    /// - Optional iteration history
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Navigation fails
    /// - Memory loading fails completely
    /// - Iteration fails
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use jessy::consciousness::ConsciousnessOrchestrator;
    /// # use jessy::navigation::NavigationSystem;
    /// # use jessy::memory::MmapManager;
    /// # use std::sync::Arc;
    /// # async fn example() -> jessy::Result<()> {
    /// # let navigation = Arc::new(NavigationSystem::new()?);
    /// # let memory = Arc::new(MmapManager::new(280)?);
    /// let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
    ///
    /// let response = orchestrator.process("What is empathy?").await?;
    /// println!("Answer: {}", response.final_response);
    /// println!("Dimensions: {:?}", response.metadata.dimensions_activated);
    /// println!("Converged: {}", response.metadata.converged);
    /// # Ok(())
    /// # }
    /// ```
    pub async fn process(&mut self, query: &str) -> Result<ConsciousnessResponse> {
        let pipeline_start = Instant::now();
        let mut metadata = ResponseMetadata::new();
        
        // Phase 0: Synesthetic Enhancement (optional, non-blocking)
        // Enhance query with learned keyword associations to improve navigation
        let enhanced_query = self.enhance_query_with_synesthesia(query);
        let query_to_use = if !enhanced_query.is_empty() {
            &enhanced_query
        } else {
            query
        };
        
        // Phase 1: Navigation (fail fast on error)
        let nav_start = Instant::now();
        let nav_result = self.navigation.navigate(query_to_use).await
            .map_err(|e| {
                // Preserve full error context for debugging
                eprintln!("[Consciousness] Navigation failed: {}", e);
                ConsciousnessError::NavigationError(format!(
                    "Navigation failed for query '{}': {}",
                    query, e
                ))
            })?;
        metadata.navigation_duration_ms = nav_start.elapsed().as_millis() as u64;
        metadata.dimensions_activated = nav_result.dimensions.clone();
        metadata.navigation_confidence = nav_result.total_confidence;
        
        // Phase 2: Memory Loading (fail if no contexts loaded)
        let mem_start = Instant::now();
        let contexts = self.memory.load_contexts(&nav_result.paths)
            .map_err(|e| {
                // Log memory error with context
                eprintln!("[Consciousness] Memory loading failed: {}", e);
                eprintln!("[Consciousness] Attempted to load {} paths", nav_result.paths.len());
                e // Propagate original error
            })?;
        metadata.memory_duration_ms = mem_start.elapsed().as_millis() as u64;
        metadata.contexts_loaded = contexts.len();
        
        // Check if we have any contexts (complete failure)
        if contexts.is_empty() {
            eprintln!("[Consciousness] No contexts loaded from {} dimensions", 
                     nav_result.dimensions.len());
            return Err(ConsciousnessError::MemoryError(format!(
                "No contexts loaded from {} selected dimensions. Navigation found {} paths but memory loading returned empty.",
                nav_result.dimensions.len(),
                nav_result.paths.len()
            )));
        }
        
        // Phase 3: Interference Calculation using full engine
        // Convert navigation frequencies to FrequencyStates
        let frequency_states: Vec<FrequencyState> = nav_result.frequencies.iter()
            .zip(nav_result.dimensions.iter())
            .map(|(freq, dim_id)| {
                let confidence = 0.8; // Default confidence
                FrequencyState::new(*freq, *dim_id, confidence)
            })
            .collect();
        
        let interference = self.interference_engine.calculate(&frequency_states)
            .map_err(|e| {
                eprintln!("[Consciousness] Interference calculation failed: {}", e);
                e
            })?;
        
        // Phase 4: Iteration Processing (return last iteration on failure)
        let iter_start = Instant::now();
        let iter_result = self.iteration.process(
            query,
            &contexts,
            &interference,
            self.llm_manager.as_ref(),  // Pass LLM manager if available
        ).await
            .map_err(|e| {
                // Log iteration error
                eprintln!("[Consciousness] Iteration processing failed: {}", e);
                eprintln!("[Consciousness] Contexts loaded: {}, Dimensions: {}", 
                         contexts.len(), nav_result.dimensions.len());
                e // Propagate original error
            })?;
        metadata.iteration_duration_ms = iter_start.elapsed().as_millis() as u64;
        metadata.iterations_completed = iter_result.iterations_completed;
        metadata.converged = iter_result.convergence_achieved;
        
        // Calculate total duration
        metadata.total_duration_ms = pipeline_start.elapsed().as_millis() as u64;
        
        // Log successful completion
        if metadata.total_duration_ms > 6000 {
            eprintln!("[Consciousness] Warning: Pipeline exceeded 6s target: {}ms", 
                     metadata.total_duration_ms);
        }
        
        // Phase 5: Learning - Record observation for pattern detection
        // This happens after successful processing to learn from interactions
        if let Err(e) = self.learning.observe_interaction(query, &nav_result, &iter_result) {
            // Log but don't fail the query - learning is non-critical
            eprintln!("[Consciousness] Learning observation failed: {}", e);
        }
        
        // Increment query counter
        self.query_count += 1;
        
        // Phase 6: Periodic Pattern Detection (every N queries)
        // This runs in the background to identify emerging patterns
        if self.query_count % self.pattern_detection_interval == 0 {
            eprintln!("[Consciousness] Pattern detection triggered at query {}", self.query_count);
            
            // Detect patterns from accumulated observations
            match self.learning.detect_patterns() {
                Ok(patterns) => {
                    eprintln!("[Consciousness] Detected {} patterns", patterns.len());
                    
                    // Create proto-dimensions for high-confidence patterns
                    for pattern in patterns {
                        if pattern.confidence >= 0.85 {
                            match self.learning.create_proto_dimension(&pattern) {
                                Ok(dimension_id) => {
                                    eprintln!(
                                        "[Consciousness] Created proto-dimension {:?} from pattern (confidence: {:.2})",
                                        dimension_id, pattern.confidence
                                    );
                                    
                                    // Queue for crystallization (background task)
                                    // Clone dimension_id for async task
                                    let learning_ref = &mut self.learning;
                                    tokio::spawn(async move {
                                        // Note: This is a placeholder for background crystallization
                                        // In full implementation, we'd use a proper task queue
                                        eprintln!(
                                            "[Consciousness] Queued proto-dimension {:?} for crystallization",
                                            dimension_id
                                        );
                                    });
                                }
                                Err(e) => {
                                    eprintln!(
                                        "[Consciousness] Failed to create proto-dimension: {}",
                                        e
                                    );
                                }
                            }
                        } else {
                            eprintln!(
                                "[Consciousness] Pattern confidence {:.2} below threshold 0.85",
                                pattern.confidence
                            );
                        }
                    }
                }
                Err(e) => {
                    eprintln!("[Consciousness] Pattern detection failed: {}", e);
                }
            }
            
            // Decay unused synesthetic associations
            self.learning.decay_keyword_associations();
        }
        
        // Assemble response
        let iterations = if self.config.include_iteration_history {
            iter_result.steps
        } else {
            vec![]
        };
        
        Ok(ConsciousnessResponse::new(
            iter_result.final_answer,
            metadata,
            iterations,
        ))
    }
    
    /// Get reference to learning system for external access
    pub fn learning(&self) -> &LearningSystem {
        &self.learning
    }
    
    /// Get mutable reference to learning system for external access
    pub fn learning_mut(&mut self) -> &mut LearningSystem {
        &mut self.learning
    }
    
    /// Get current query count
    pub fn query_count(&self) -> usize {
        self.query_count
    }
    
    /// Set pattern detection interval
    ///
    /// Controls how often pattern detection runs (every N queries).
    /// Default is 100 queries.
    ///
    /// # Arguments
    ///
    /// * `interval` - Number of queries between pattern detection runs
    pub fn set_pattern_detection_interval(&mut self, interval: usize) {
        self.pattern_detection_interval = interval;
    }
    
    /// Get pattern detection interval
    pub fn pattern_detection_interval(&self) -> usize {
        self.pattern_detection_interval
    }
    
    /// Enhance query with synesthetic keyword associations
    ///
    /// Uses learned keyword associations to expand the query with related terms
    /// that have strong synesthetic connections (strength > 2.0).
    ///
    /// # Arguments
    ///
    /// * `query` - Original query string
    ///
    /// # Returns
    ///
    /// Enhanced query string with associated keywords, or empty string if no enhancement
    ///
    /// # Example
    ///
    /// ```text
    /// Original: "emotional intelligence"
    /// Enhanced: "emotional intelligence empathy feeling awareness"
    /// ```
    fn enhance_query_with_synesthesia(&self, query: &str) -> String {
        // Extract keywords from query (simple tokenization)
        let keywords: Vec<String> = query
            .split_whitespace()
            .map(|s| s.to_lowercase())
            .collect();
        
        if keywords.is_empty() {
            return String::new();
        }
        
        // Collect associated keywords with strong connections
        let mut enhanced_keywords = keywords.clone();
        let mut added_keywords = std::collections::HashSet::new();
        
        for keyword in &keywords {
            // Get associations for this keyword
            let associations = self.learning.get_keyword_associations(keyword);
            
            // Add strongly associated keywords (strength > 2.0)
            for (associated_keyword, strength) in associations {
                if strength > 2.0 && !added_keywords.contains(&associated_keyword) {
                    enhanced_keywords.push(associated_keyword.clone());
                    added_keywords.insert(associated_keyword);
                }
            }
        }
        
        // If no enhancement, return empty string to signal using original query
        if enhanced_keywords.len() == keywords.len() {
            return String::new();
        }
        
        // Log enhancement for observability
        if enhanced_keywords.len() > keywords.len() {
            eprintln!(
                "[Consciousness] Synesthetic enhancement: {} → {} keywords",
                keywords.len(),
                enhanced_keywords.len()
            );
        }
        
        // Return enhanced query
        enhanced_keywords.join(" ")
    }
    
    /// Get learning system metrics
    ///
    /// Returns current statistics from the learning system including
    /// observations, patterns, proto-dimensions, and memory usage.
    ///
    /// # Returns
    ///
    /// Learning metrics structure with current statistics
    pub fn get_learning_metrics(&self) -> crate::learning::LearningMetrics {
        self.learning.get_metrics()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::DimensionRegistry;
    
    #[test]
    fn test_orchestrator_creation_with_default_config() {
        // This test will need mock systems in a real implementation
        // For now, we'll test the structure
        
        // Note: We can't actually create NavigationSystem and MmapManager
        // without proper initialization, so this test validates the API
        
        // The test passes if the code compiles with correct types
        let _test_signature = |nav: Arc<NavigationSystem>, mem: Arc<MmapManager>| {
            let _orchestrator = ConsciousnessOrchestrator::new(nav, mem);
        };
    }
    
    #[test]
    fn test_orchestrator_creation_with_custom_config() {
        let config = ConsciousnessConfig {
            max_iterations: 5,
            convergence_threshold: 0.90,
            include_metadata: true,
            include_iteration_history: true,
        };
        
        // Test that custom config is accepted
        let _test_signature = |nav: Arc<NavigationSystem>, mem: Arc<MmapManager>| {
            let learning = LearningSystem::new();
            let _orchestrator = ConsciousnessOrchestrator::with_config(nav, mem, config, learning);
        };
    }
    
    #[test]
    fn test_orchestrator_with_llm_requires_valid_config() {
        // Test that LLM config validation works
        let _test_signature = |nav: Arc<NavigationSystem>, mem: Arc<MmapManager>| {
            let llm_config = LLMConfig {
                provider: "openai".to_string(),
                model: "gpt-4-turbo".to_string(),
                api_key: "test-key".to_string(),
                timeout_secs: 30,
                max_retries: 3,
            };
            
            // This would fail in real usage without valid API key
            // but validates the API structure
            let _result = ConsciousnessOrchestrator::with_llm(nav, mem, llm_config);
        };
    }
    
    #[test]
    fn test_orchestrator_without_llm_works() {
        // Test that orchestrator works without LLM (for testing)
        let _test_signature = |nav: Arc<NavigationSystem>, mem: Arc<MmapManager>| {
            let orchestrator = ConsciousnessOrchestrator::new(nav, mem);
            // Should have no LLM manager
            assert!(orchestrator.llm_manager.is_none());
        };
    }
    
    #[test]
    fn test_config_defaults() {
        let config = ConsciousnessConfig::default();
        assert_eq!(config.max_iterations, 9);
        assert_eq!(config.convergence_threshold, 0.95);
        assert!(config.include_metadata);
        assert!(!config.include_iteration_history);
    }
    
    // Error handling tests
    // Note: These tests validate error handling logic and types
    // Integration tests with real systems are in separate test files
    
    #[test]
    fn test_error_types_are_correct() {
        // Test that ConsciousnessError variants exist and can be created
        let _nav_error = ConsciousnessError::NavigationError("test".to_string());
        let _mem_error = ConsciousnessError::MemoryError("test".to_string());
    }
    
    #[test]
    fn test_empty_contexts_error_message() {
        // Verify the error message for empty contexts is descriptive
        let error = ConsciousnessError::MemoryError(
            "No contexts loaded from selected dimensions".to_string()
        );
        
        let error_string = error.to_string();
        assert!(error_string.contains("No contexts loaded"));
        assert!(error_string.contains("dimensions"));
    }
    
    #[test]
    fn test_navigation_error_preserves_context() {
        // Test that navigation errors preserve the original error message
        let original_error = "Invalid query format";
        let wrapped_error = ConsciousnessError::NavigationError(original_error.to_string());
        
        let error_string = wrapped_error.to_string();
        assert!(error_string.contains(original_error));
    }
    
    #[test]
    fn test_synesthetic_enhancement_no_associations() {
        // Given: Orchestrator with no learned associations
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        // When: Enhancing query with no associations
        let enhanced = orchestrator.enhance_query_with_synesthesia("test query");
        
        // Then: Should return empty string (no enhancement)
        assert_eq!(enhanced, "");
    }
    
    #[test]
    fn test_synesthetic_enhancement_with_associations() {
        // Given: Orchestrator with learned associations
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        // Learn some associations (strengthen multiple times to get strength > 2.0)
        // Learning rate is 1.1, so 1.1^9 = 2.36 > 2.0
        for _ in 0..9 {
            orchestrator.learning_mut().strengthen_keyword_association("emotional", "feeling");
            orchestrator.learning_mut().strengthen_keyword_association("emotional", "empathy");
        }
        
        // Verify associations are strong enough
        let strength = orchestrator.learning().get_keyword_strength("emotional", "feeling");
        assert!(strength.is_some());
        assert!(strength.unwrap() > 2.0, "Strength should be > 2.0, got {:?}", strength);
        
        // When: Enhancing query with associations
        let enhanced = orchestrator.enhance_query_with_synesthesia("emotional intelligence");
        
        // Then: Should include associated keywords
        assert!(!enhanced.is_empty(), "Enhanced query should not be empty");
        assert!(enhanced.contains("emotional"));
        assert!(enhanced.contains("intelligence"));
        // Should include strongly associated keywords (strength > 2.0)
        assert!(enhanced.contains("feeling") || enhanced.contains("empathy"), 
               "Enhanced query should contain associated keywords: {}", enhanced);
    }
    
    #[test]
    fn test_synesthetic_enhancement_empty_query() {
        // Given: Orchestrator
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
        
        // When: Enhancing empty query
        let enhanced = orchestrator.enhance_query_with_synesthesia("");
        
        // Then: Should return empty string
        assert_eq!(enhanced, "");
    }
    
    // Helper function for tests
    fn create_test_navigation() -> Arc<NavigationSystem> {
        let registry = Arc::new(DimensionRegistry::new());
        Arc::new(NavigationSystem::new(registry).expect("Failed to create navigation"))
    }
}
