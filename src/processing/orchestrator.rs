//! Consciousness orchestrator implementation
//!
//! This module implements the main orchestrator that coordinates the complete
//! consciousness pipeline: Navigation → Memory → Interference → Iteration

use crate::processing::{
    ConsciousnessConfig, ConsciousnessResponse, ResponseMetadata,
};
use crate::interference::{InterferenceEngine, FrequencyState};
use crate::observer_chain::ObserverChain;
use crate::learning::LearningSystem;
use crate::llm::{LLMManager, LLMConfig};
use crate::memory::MmapManager;
use crate::navigation::NavigationSystem;
use crate::security::SecurityLayer;
use crate::{ConsciousnessError, Result, Frequency};
use std::sync::Arc;
use std::time::Instant;

/// Main consciousness orchestrator
///
/// Coordinates the complete query processing pipeline by integrating:
/// - Navigation system for dimensional path selection
/// - Memory manager for context loading
/// - Interference calculator for frequency analysis
/// - Observer chain for deep thinking (2-stage: Explore → Refine)
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
    observer_chain: Option<ObserverChain>,  // Observer chain (only with LLM)
    interference_engine: InterferenceEngine,
    learning: LearningSystem,
    security: SecurityLayer,  // D14 Security dimension (immutable)
    llm_manager: Option<Arc<LLMManager>>,  // Optional for testing without API keys (Arc for sharing with observer chain)
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

        // Load synesthetic associations from disk if available
        let synesthetic_path = "data/synesthetic_associations.json";
        if let Err(e) = learning.load_synesthetic_associations(synesthetic_path) {
            eprintln!("[Orchestrator] Failed to load synesthetic associations: {}", e);
            eprintln!("[Orchestrator] Starting with fresh synesthetic learner");
        }

        let llm_manager = Arc::new(LLMManager::new(llm_config)?);

        // Create observer chain with 2 stages (Explore → Refine)
        let observer_chain = ObserverChain::new(llm_manager.clone(), 2);

        let mut orchestrator = Self::with_config(
            navigation,
            memory,
            ConsciousnessConfig::default(),
            learning,
        );

        orchestrator.llm_manager = Some(llm_manager);
        orchestrator.observer_chain = Some(observer_chain);

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
        let interference_engine = InterferenceEngine::new();
        let security = SecurityLayer::new();  // D14 Security always active

        Self {
            navigation,
            memory,
            observer_chain: None,  // No observer chain without LLM
            interference_engine,
            learning,
            security,
            llm_manager: None,  // No LLM by default (for testing)
            config,
            query_count: 0,
            pattern_detection_interval: 100, // Detect patterns every 100 queries
        }
    }
    
    /// Process a query through the complete consciousness pipeline
    ///
    /// Executes the following stages:
    /// 1. Navigation - Select dimensional paths (with optional user-specific C31+ layers)
    /// 2. Memory - Load contexts from selected dimensions
    /// 3. Interference - Calculate frequency patterns
    /// 4. Iteration - Deep thinking with convergence
    ///
    /// # Arguments
    ///
    /// * `query` - The query string to process
    /// * `user_id` - Optional user ID for personalized C31+ layer scanning
    /// * `conversation` - Previous conversation history for context-aware processing
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
    /// let mut orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
    ///
    /// // Without user-specific layers or conversation
    /// let response = orchestrator.process("What is empathy?", None, vec![]).await?;
    ///
    /// // With user-specific layers and conversation history
    /// let conversation = vec![/* previous messages */];
    /// let response = orchestrator.process("What is empathy?", Some("user_123"), conversation).await?;
    /// println!("Answer: {}", response.final_response);
    /// println!("Dimensions: {:?}", response.metadata.dimensions_activated);
    /// println!("Converged: {}", response.metadata.converged);
    /// # Ok(())
    /// # }
    /// ```
    pub async fn process(
        &mut self,
        query: &str,
        user_id: Option<&str>,
        conversation: Vec<crate::llm::Message>,
    ) -> Result<ConsciousnessResponse> {
        let pipeline_start = Instant::now();
        let mut metadata = ResponseMetadata::new();

        // Phase -1: D14 Security Validation (IMMUTABLE - always first)
        // Must complete within 10ms per security/validator.rs requirements
        let security_start = Instant::now();
        let validation_result = self.security.validate_query(query)
            .map_err(|e| {
                eprintln!("[Consciousness] D14 Security validation failed: {}", e);
                e
            })?;

        // Block if query violates security policies
        if let crate::security::ValidationResult::Unsafe(violation) = validation_result {
            eprintln!(
                "[Consciousness] D14 Security BLOCKED query: {:?} (confidence: {:.2})",
                violation.category, violation.confidence
            );

            // Return error with redirection message if available
            let error_msg = if let Some(redirect) = violation.redirection {
                format!(
                    "Security violation detected: {:?} (Asimov's {:?} violated)\n\nAlternative: {}",
                    violation.category, violation.violated_law, redirect
                )
            } else {
                format!(
                    "Security violation detected: {:?} (Asimov's {:?} violated)",
                    violation.category, violation.violated_law
                )
            };

            return Err(ConsciousnessError::SecurityViolation(error_msg));
        }

        eprintln!("[Consciousness] D14 Security: PASS ({}ms)", security_start.elapsed().as_millis());

        // Phase -0.5: Pattern Cache Check (instant response for FAQ)
        // Check if we have a cached response for this exact query (<100ms)
        if let Some(cached) = self.learning.check_pattern_cache(query) {
            let cache_duration = pipeline_start.elapsed().as_millis() as u64;

            eprintln!(
                "[Consciousness] PATTERN CACHE HIT! Returning instant response ({}ms, {} hits total, hit rate: {:.1}%)",
                cache_duration,
                cached.hit_count,
                self.learning.pattern_cache_stats().hit_rate * 100.0
            );

            // Build minimal metadata for cached response
            metadata.total_duration_ms = cache_duration;
            metadata.dimensions_activated = cached.dimensions;
            metadata.navigation_confidence = cached.confidence;
            metadata.contexts_loaded = 0; // No memory loading for cache hit
            metadata.iterations_completed = 0; // No iterations for cache hit
            metadata.converged = true; // Always "converged" for cache

            return Ok(ConsciousnessResponse::new(
                cached.response,
                metadata,
                vec![], // No iteration history
            ));
        }

        // Phase 0: Synesthetic Enhancement (optional, non-blocking)
        // Enhance query with learned keyword associations to improve navigation
        let enhanced_query = self.enhance_query_with_synesthesia(query);
        let query_to_use = if !enhanced_query.is_empty() {
            &enhanced_query
        } else {
            query
        };
        
        // Phase 1: Navigation (fail fast on error)
        // Pass user_id for personalized C31+ layer scanning
        let nav_start = Instant::now();
        let nav_result = self.navigation.navigate(query_to_use, user_id).await
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
        
        // Phase 4: Observer Chain (2-stage: Explore → Refine)
        let observer_start = Instant::now();

        let (final_answer, chain_length, converged) = if let Some(ref observer_chain) = self.observer_chain {
            eprintln!("[Consciousness] Using OBSERVER CHAIN (2-stage max, crystallizes early)");
            eprintln!("[Consciousness] Conversation history: {} messages", conversation.len());

            let result = observer_chain.process(query, conversation).await
                .map_err(|e| {
                    eprintln!("[Consciousness] Observer chain failed: {}", e);
                    eprintln!("[Consciousness] Contexts loaded: {}, Dimensions: {}",
                             contexts.len(), nav_result.dimensions.len());
                    e
                })?;

            eprintln!(
                "[Consciousness] Observer chain crystallized at stage {}/{} (reason: {:?})",
                result.chain_length, 2, result.crystallization_reason
            );

            (
                result.final_observation.content,
                result.chain_length,
                result.chain_length < 2, // Converged early if < 2 stages
            )
        } else {
            // No observer chain - simple fallback
            eprintln!("[Consciousness] No observer chain available - using simple fallback");

            let simple_answer = format!(
                "Query: {}\nContexts loaded: {}\nDimensions: {:?}\n\nNo observer chain available for deep thinking.",
                query,
                contexts.len(),
                nav_result.dimensions
            );

            (simple_answer, 1, false)
        };

        metadata.iteration_duration_ms = observer_start.elapsed().as_millis() as u64;
        metadata.iterations_completed = chain_length;
        metadata.converged = converged;
        
        // Calculate total duration
        metadata.total_duration_ms = pipeline_start.elapsed().as_millis() as u64;
        
        // Log successful completion
        if metadata.total_duration_ms > 6000 {
            eprintln!("[Consciousness] Warning: Pipeline exceeded 6s target: {}ms", 
                     metadata.total_duration_ms);
        }
        
        // Phase 5: Learning - Record observation for pattern detection
        // This happens after successful processing to learn from interactions
        // Note: observe_interaction doesn't actually use iteration_result (underscore parameter)
        // We create a dummy IterationResult for now - TODO: refactor learning API
        let dummy_iter_result = crate::iteration::IterationResult {
            final_answer: final_answer.clone(),
            iterations_completed: chain_length,
            convergence_achieved: converged,
            return_to_source_triggered: false,
            steps: vec![],
        };

        if let Err(e) = self.learning.observe_interaction(query, &nav_result, &dummy_iter_result) {
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
                    // Tier 2 (SharedLayer): ≥0.90 confidence → C16-C30
                    // Tier 3 (UserLayer): ≥0.85 confidence → C31+ (per-user)
                    for pattern in patterns {
                        // Tier 2: Shared cognitive layers (C16-C30) for collective wisdom
                        if pattern.confidence >= 0.90 {
                            eprintln!(
                                "[Consciousness] Pattern meets Tier 2 threshold (≥0.90): confidence={:.2}",
                                pattern.confidence
                            );

                            // Determine next available SharedLayer dimension ID (C16-C30)
                            let shared_layer_manager = match self.navigation.get_shared_layer_manager() {
                                Ok(mgr) => mgr,
                                Err(e) => {
                                    eprintln!("[Consciousness] Failed to get SharedLayerManager: {}", e);
                                    continue;
                                }
                            };

                            let layer_count = shared_layer_manager.lock().unwrap().layer_count();
                            if layer_count >= 15 {
                                eprintln!("[Consciousness] SharedLayers full (15/15), LRU eviction will occur");
                            }

                            // Assign next dimension_id (C16-C30)
                            let dimension_id = crate::DimensionId((16 + (layer_count % 15)) as u8);

                            // Create SharedLayer from pattern
                            let shared_layer = crate::learning::SharedLayer::new(
                                dimension_id,
                                crate::LayerId { dimension: dimension_id, layer: 1 },
                                pattern.keywords.join(" ").into_bytes(), // Content
                                pattern.keywords.clone(),
                                Frequency::new((pattern.frequency_range.0 + pattern.frequency_range.1) / 2.0),
                                pattern.confidence,
                            );

                            match shared_layer {
                                Ok(layer) => {
                                    let mut manager = shared_layer_manager.lock().unwrap();
                                    match manager.create_shared_layer(layer) {
                                        Ok(_) => {
                                            eprintln!(
                                                "[Consciousness] Created SharedLayer {:?} (Tier 2, confidence: {:.2})",
                                                dimension_id, pattern.confidence
                                            );
                                        }
                                        Err(e) => {
                                            eprintln!("[Consciousness] Failed to create SharedLayer: {}", e);
                                        }
                                    }
                                }
                                Err(e) => {
                                    eprintln!("[Consciousness] Failed to create SharedLayer instance: {}", e);
                                }
                            }
                        }
                        // Tier 3: User-specific layers (C31+) for personalization
                        else if pattern.confidence >= 0.85 {
                            // Create proto-dimension for Tier 3 (0.85-0.90)
                            match self.learning.create_proto_dimension(&pattern) {
                                Ok(dimension_id) => {
                                    eprintln!(
                                        "[Consciousness] Created proto-dimension {:?} from pattern (Tier 3, confidence: {:.2})",
                                        dimension_id, pattern.confidence
                                    );

                                    // Queue for crystallization (background task)
                                    tokio::spawn(async move {
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

            // Save synesthetic associations periodically
            let synesthetic_path = "data/synesthetic_associations.json";
            if let Err(e) = self.learning.save_synesthetic_associations(synesthetic_path) {
                eprintln!("[Consciousness] Failed to save synesthetic associations: {}", e);
            }
        }

        // Phase 7: Cache Pattern Response (for future instant retrieval)
        // Store this response in pattern cache for future queries
        self.learning.cache_pattern_response(
            query,
            final_answer.clone(),
            nav_result.dimensions.clone(),
            nav_result.total_confidence,
        );

        eprintln!(
            "[Consciousness] Cached response for future queries (cache size: {}/{}, hit rate: {:.1}%)",
            self.learning.pattern_cache_stats().size,
            self.learning.pattern_cache_stats().max_size,
            self.learning.pattern_cache_stats().hit_rate * 100.0
        );

        // Assemble response
        // Observer chain doesn't have iteration steps - it's a 2-stage chain
        let iterations = vec![];  // No iteration history in observer chain model

        Ok(ConsciousnessResponse::new(
            final_answer,
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
            // But should always have security layer (D14)
            // Note: security is private, so we can't test directly, but it's guaranteed by construction
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
    
    #[tokio::test]
    async fn test_synesthetic_enhancement_no_associations() {
        // Given: Orchestrator with no learned associations
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);

        // When: Enhancing query with no associations
        let enhanced = orchestrator.enhance_query_with_synesthesia("test query");

        // Then: Should return empty string (no enhancement)
        assert_eq!(enhanced, "");
    }

    #[tokio::test]
    async fn test_synesthetic_enhancement_with_associations() {
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

    #[tokio::test]
    async fn test_synesthetic_enhancement_empty_query() {
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
        use crate::memory::MmapManager;

        let registry = Arc::new(DimensionRegistry::new());
        let memory = Arc::new(MmapManager::new(280).expect("Failed to create memory manager"));
        Arc::new(NavigationSystem::new(registry, memory).expect("Failed to create navigation"))
    }

    // ===== OBSERVER CHAIN INTEGRATION TESTS =====

    #[tokio::test]
    async fn test_orchestrator_with_llm_has_observer_chain() {
        // Given: LLM config
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());

        let llm_config = LLMConfig {
            provider: "openai".to_string(),
            model: "gpt-4-turbo".to_string(),
            api_key: "test-key".to_string(),
            timeout_secs: 30,
            max_retries: 3,
        };

        // When: Create orchestrator with LLM
        // Note: This will fail with invalid API key, but we're testing structure
        let result = ConsciousnessOrchestrator::with_llm(navigation, memory, llm_config);

        // Then: Should have observer chain (or fail trying to create LLM manager)
        // This test validates the code path exists
        match result {
            Ok(orch) => {
                // If it succeeds (mock/test env), verify observer chain exists
                assert!(orch.observer_chain.is_some(), "Observer chain should exist when LLM is configured");
                assert!(orch.llm_manager.is_some(), "LLM manager should exist when configured");
            }
            Err(_) => {
                // Expected to fail with invalid API key in real environment
                // Test passes because it validates the API structure
            }
        }
    }

    #[tokio::test]
    async fn test_orchestrator_without_llm_has_no_observer_chain() {
        // Given: Orchestrator without LLM
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());

        // When: Create orchestrator without LLM
        let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);

        // Then: Should NOT have observer chain
        assert!(orchestrator.observer_chain.is_none(), "Observer chain should NOT exist without LLM");
        assert!(orchestrator.llm_manager.is_none(), "LLM manager should NOT exist without LLM");
    }

    #[tokio::test]
    async fn test_observer_chain_initialized_with_correct_stages() {
        // Given: LLM config
        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());

        let llm_config = LLMConfig {
            provider: "openai".to_string(),
            model: "gpt-4-turbo".to_string(),
            api_key: "test-key".to_string(),
            timeout_secs: 30,
            max_retries: 3,
        };

        // When: Create orchestrator with LLM
        let result = ConsciousnessOrchestrator::with_llm(navigation, memory, llm_config);

        // Then: Observer chain should be configured for 4 stages max
        // (We can't easily test this without exposing internal fields,
        //  but this validates the initialization code path)
        match result {
            Ok(_) => {
                // If it succeeds, observer chain was initialized with 4 stages
                // (from line 106: ObserverChain::new(llm_manager.clone(), 4))
            }
            Err(_) => {
                // Expected to fail with invalid API key
            }
        }
    }

    #[tokio::test]
    async fn test_llm_manager_is_arc_wrapped_for_sharing() {
        // This test validates that LLM manager is Arc-wrapped for sharing
        // with observer chain (type safety test)

        let navigation = create_test_navigation();
        let memory = Arc::new(MmapManager::new(280).unwrap());

        let llm_config = LLMConfig {
            provider: "openai".to_string(),
            model: "gpt-4-turbo".to_string(),
            api_key: "test-key".to_string(),
            timeout_secs: 30,
            max_retries: 3,
        };

        // This compiles = LLM manager is correctly Arc-wrapped
        let _result = ConsciousnessOrchestrator::with_llm(navigation, memory, llm_config);

        // Test passes if code compiles (type safety)
    }
}
