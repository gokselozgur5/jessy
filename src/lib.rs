//! Jessy - Multidimensional AI Consciousness Architecture
//!
//! Copyright (C) 2024 Göksel Özgür
//! Creator & Architect: gokselozgur5
//! Engineering Methodology: Prompt-driven software development
//! Development Tools: Claude Code (Anthropic), Kıro
//!
//! This program is free software: you can redistribute it and/or modify
//! it under the terms of the GNU Affero General Public License as published by
//! the Free Software Foundation, either version 3 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//! GNU Affero General Public License for more details.
//!
//! You should have received a copy of the GNU Affero General Public License
//! along with this program. If not, see <https://www.gnu.org/licenses/>.
//!
//! This crate implements a frequency-based consciousness system that processes queries
//! through multiple dimensional layers using memory-mapped files for zero-copy access.
//!
//! # Architecture
//!
//! The system consists of several integrated modules:
//!
//! - **consciousness**: Main orchestrator that coordinates the complete pipeline
//! - **navigation**: Dimensional path selection and query analysis
//! - **memory**: Zero-copy MMAP-based context loading
//! - **interference**: Frequency pattern calculation
//! - **iteration**: 9-iteration deep thinking with convergence
//! - **security**: Asimov's Laws enforcement
//! - **learning**: Pattern learning and adaptation
//!
//! # Quick Start
//!
//! ```no_run
//! use jessy::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
//! use jessy::navigation::NavigationSystem;
//! use jessy::memory::MmapManager;
//! use std::sync::Arc;
//!
//! # async fn example() -> jessy::Result<()> {
//! // Initialize systems
//! let navigation = Arc::new(NavigationSystem::new()?);
//! let memory = Arc::new(MmapManager::new(280)?);
//!
//! // Create consciousness orchestrator
//! let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
//!
//! // Process query
//! let response = orchestrator.process("What is consciousness?").await?;
//! println!("Answer: {}", response.final_response);
//! println!("Converged: {}", response.metadata.converged);
//! # Ok(())
//! # }
//! ```

pub mod memory;
pub mod dimensions;
pub mod navigation;
pub mod interference;
pub mod iteration;
pub mod learning;
pub mod security;
pub mod consciousness;
pub mod ffi;
pub mod llm;
pub mod config;

use std::collections::HashMap;
use thiserror::Error;

/// Core error types for the consciousness system
#[derive(Error, Debug)]
pub enum ConsciousnessError {
    #[error("Memory mapping failed: {0}")]
    MemoryError(String),
    
    #[error("Memory allocation failed: {0}")]
    AllocationFailed(String),
    
    #[error("Memory limit exceeded: current={current_mb}MB, limit={limit_mb}MB, requested={requested_mb}MB")]
    LimitExceeded {
        current_mb: usize,
        limit_mb: usize,
        requested_mb: usize,
    },
    
    #[error("Layer not found: dimension={dimension}, layer={layer}")]
    LayerNotFound {
        dimension: u8,
        layer: u16,
    },
    
    #[error("Region not found: region_id={region_id}")]
    RegionNotFound {
        region_id: u32,
    },
    
    #[error("Out of bounds access: offset={offset}, size={size}, region_size={region_size}")]
    OutOfBounds {
        offset: usize,
        size: usize,
        region_size: usize,
    },
    
    #[error("Dimension not found: dimension={dimension}")]
    DimensionNotFound {
        dimension: u8,
    },
    
    #[error("Security violation detected: {0}")]
    SecurityViolation(String),
    
    #[error("Analysis paralysis detected, returning to source")]
    AnalysisParalysis,
    
    #[error("Frequency interference calculation failed: {0}")]
    InterferenceError(String),
    
    #[error("Dimension navigation failed: {0}")]
    NavigationError(String),
    
    #[error("Learning system error: {0}")]
    LearningError(String),
    
    #[error("Iteration processing failed: {0}")]
    IterationError(String),
    
    #[error("Invalid input: {0}")]
    InvalidInput(String),
    
    #[error("Operation timeout: {0}")]
    Timeout(String),
}

impl From<crate::learning::LearningError> for ConsciousnessError {
    fn from(err: crate::learning::LearningError) -> Self {
        ConsciousnessError::LearningError(err.to_string())
    }
}

/// Result type for consciousness operations
pub type Result<T> = std::result::Result<T, ConsciousnessError>;

/// Unique identifier for dimensions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct DimensionId(pub u8);

/// Unique identifier for layers within dimensions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct LayerId {
    pub dimension: DimensionId,
    pub layer: u16,
}

/// Frequency measurement in Hz
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frequency(pub f32);

impl Frequency {
    /// Create new frequency, clamped to valid range (0.1-4.5 Hz)
    pub fn new(hz: f32) -> Self {
        Self(hz.clamp(0.1, 4.5))
    }
    
    /// Get the Hz value
    pub fn hz(&self) -> f32 {
        self.0
    }
    
    /// Check if frequency indicates extreme state (>3.5 Hz)
    pub fn is_extreme(&self) -> bool {
        self.0 > 3.5
    }
    
    /// Check if frequency indicates deep contemplative state (<0.5 Hz)
    pub fn is_deep(&self) -> bool {
        self.0 < 0.5
    }
}

/// Core consciousness system configuration
#[derive(Debug, Clone)]
pub struct ConsciousnessConfig {
    /// Total memory allocation for MMAP regions (default: 280MB)
    pub total_memory_mb: usize,
    
    /// Maximum iterations for deep thinking (default: 9)
    pub max_iterations: usize,
    
    /// Convergence threshold for early stopping (default: 0.95)
    pub convergence_threshold: f32,
    
    /// Complexity threshold for return-to-source (default: 6 dimensions)
    pub complexity_threshold: usize,
    
    /// Security response time limit in milliseconds (default: 10ms)
    pub security_timeout_ms: u64,
}

impl Default for ConsciousnessConfig {
    fn default() -> Self {
        Self {
            total_memory_mb: 280,
            max_iterations: 9,
            convergence_threshold: 0.95,
            complexity_threshold: 6,
            security_timeout_ms: 10,
        }
    }
}

/// Main consciousness system entry point
#[derive(Debug)]
pub struct ConsciousnessSystem {
    config: ConsciousnessConfig,
    memory_manager: memory::MmapManager,
    navigator: navigation::MultiverseNavigator,
    interference_engine: interference::InterferenceEngine,
    iteration_processor: iteration::IterationProcessor,
    learning_system: learning::LearningSystem,
    security_layer: security::SecurityLayer,
}

impl ConsciousnessSystem {
    /// Initialize new consciousness system with default configuration
    pub async fn new() -> Result<Self> {
        Self::with_config(ConsciousnessConfig::default()).await
    }
    
    /// Initialize consciousness system with custom configuration
    pub async fn with_config(config: ConsciousnessConfig) -> Result<Self> {
        let memory_manager = memory::MmapManager::new(config.total_memory_mb)?;
        let navigator = navigation::MultiverseNavigator::new();
        let interference_engine = interference::InterferenceEngine::new();
        let iteration_processor = iteration::IterationProcessor::new(
            config.max_iterations,
            config.convergence_threshold,
            config.complexity_threshold,
        );
        let learning_system = learning::LearningSystem::new();
        let security_layer = security::SecurityLayer::new();
        
        Ok(Self {
            config,
            memory_manager,
            navigator,
            interference_engine,
            iteration_processor,
            learning_system,
            security_layer,
        })
    }
    
    /// Process a query through the consciousness system
    pub async fn process_query(&mut self, query: &str) -> Result<ConsciousnessResponse> {
        // Security check first (must complete within timeout)
        self.security_layer.validate_query(query)?;
        
        // Navigate multiverse to find relevant dimensions and layers
        let navigation_result = self.navigator.navigate(query)?;
        
        // Load memory-mapped content for selected paths
        let contexts = self.memory_manager.load_contexts(&navigation_result.paths)?;
        
        // Convert frequencies to FrequencyStates for interference calculation
        let frequency_states: Vec<interference::FrequencyState> = navigation_result.frequencies.iter()
            .zip(navigation_result.dimensions.iter())
            .map(|(freq_hz, dim_id)| {
                let confidence = 0.8; // Default confidence for now
                interference::FrequencyState::new(Frequency::new(*freq_hz as f32), *dim_id, confidence)
            })
            .collect();
        
        // Calculate frequency interference patterns
        let interference = self.interference_engine.calculate(&frequency_states)?;
        
        // Process through 9-iteration deep thinking
        let iteration_result = self.iteration_processor.process(
            query,
            &contexts,
            &interference,
            None,  // No LLM in deprecated path
        ).await?;
        
        // Learn from this interaction
        // Note: This uses old navigator type - should migrate to ConsciousnessOrchestrator
        // For now, skip learning in this deprecated path
        // self.learning_system.observe_interaction(query, &navigation_result, &iteration_result)?;
        
        Ok(ConsciousnessResponse {
            answer: iteration_result.final_answer,
            dominant_frequency: interference.pattern.dominant_frequency,
            dimensions_activated: navigation_result.dimensions,
            iterations_completed: iteration_result.iterations_completed,
            return_to_source_triggered: iteration_result.return_to_source_triggered,
        })
    }
}

/// Response from consciousness system processing
#[derive(Debug, Clone)]
pub struct ConsciousnessResponse {
    /// Final processed answer
    pub answer: String,
    
    /// Dominant frequency that emerged from interference
    pub dominant_frequency: Frequency,
    
    /// List of dimensions that were activated
    pub dimensions_activated: Vec<DimensionId>,
    
    /// Number of iterations completed (may be less than 9 if converged early)
    pub iterations_completed: usize,
    
    /// Whether return-to-source protocol was triggered
    pub return_to_source_triggered: bool,
}

#[cfg(feature = "python-bindings")]
pub mod python {
    use pyo3::prelude::*;
    
    /// Python wrapper for consciousness system
    #[pyclass]
    pub struct PyConsciousnessSystem {
        inner: crate::ConsciousnessSystem,
    }
    
    #[pymethods]
    impl PyConsciousnessSystem {
        #[new]
        pub fn new() -> PyResult<Self> {
            let rt = tokio::runtime::Runtime::new().unwrap();
            let inner = rt.block_on(crate::ConsciousnessSystem::new())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            
            Ok(Self { inner })
        }
        
        pub fn process_query(&mut self, query: &str) -> PyResult<String> {
            let rt = tokio::runtime::Runtime::new().unwrap();
            let response = rt.block_on(self.inner.process_query(query))
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            
            Ok(response.answer)
        }
    }
    
    #[pymodule]
    fn jessy(_py: Python, m: &PyModule) -> PyResult<()> {
        m.add_class::<PyConsciousnessSystem>()?;
        Ok(())
    }
}