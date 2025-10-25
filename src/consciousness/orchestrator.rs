//! Consciousness orchestrator implementation
//!
//! This module implements the main orchestrator that coordinates the complete
//! consciousness pipeline: Navigation → Memory → Interference → Iteration

use crate::consciousness::{
    ConsciousnessConfig, ConsciousnessResponse, ResponseMetadata,
    create_simple_interference,
};
use crate::iteration::IterationProcessor;
use crate::memory::MmapManager;
use crate::navigation::NavigationSystem;
use crate::{ConsciousnessError, Result};
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
    config: ConsciousnessConfig,
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
    /// Returns a new orchestrator with default configuration
    pub fn new(navigation: Arc<NavigationSystem>, memory: Arc<MmapManager>) -> Self {
        Self::with_config(navigation, memory, ConsciousnessConfig::default())
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
    ) -> Self {
        let iteration = IterationProcessor::new(
            config.max_iterations,
            config.convergence_threshold,
            6, // complexity_threshold for return-to-source
        );
        
        Self {
            navigation,
            memory,
            iteration,
            config,
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
    pub async fn process(&self, query: &str) -> Result<ConsciousnessResponse> {
        let pipeline_start = Instant::now();
        let mut metadata = ResponseMetadata::new();
        
        // Phase 1: Navigation
        let nav_start = Instant::now();
        let nav_result = self.navigation.navigate(query).await
            .map_err(|e| ConsciousnessError::NavigationError(e.to_string()))?;
        metadata.navigation_duration_ms = nav_start.elapsed().as_millis() as u64;
        metadata.dimensions_activated = nav_result.dimensions.clone();
        metadata.navigation_confidence = nav_result.total_confidence;
        
        // Phase 2: Memory Loading
        let mem_start = Instant::now();
        let contexts = self.memory.load_contexts(&nav_result.paths)?;
        metadata.memory_duration_ms = mem_start.elapsed().as_millis() as u64;
        metadata.contexts_loaded = contexts.len();
        
        // Check if we have any contexts
        if contexts.is_empty() {
            return Err(ConsciousnessError::MemoryError(
                "No contexts loaded from selected dimensions".to_string()
            ));
        }
        
        // Phase 3: Interference Calculation
        let interference = create_simple_interference(&contexts);
        
        // Phase 4: Iteration Processing
        let iter_start = Instant::now();
        let iter_result = self.iteration.process(query, &contexts, &interference).await?;
        metadata.iteration_duration_ms = iter_start.elapsed().as_millis() as u64;
        metadata.iterations_completed = iter_result.iterations_completed;
        metadata.converged = iter_result.convergence_achieved;
        
        // Calculate total duration
        metadata.total_duration_ms = pipeline_start.elapsed().as_millis() as u64;
        
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
}

#[cfg(test)]
mod tests {
    use super::*;
    
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
            let _orchestrator = ConsciousnessOrchestrator::with_config(nav, mem, config);
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
}
