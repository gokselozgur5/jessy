//! Diagnostic events and tracing for navigation system
//!
//! This module provides structured diagnostic events for all navigation operations,
//! enabling detailed observability, debugging, and performance analysis.
//!
//! # Event Types
//! - QueryAnalyzed: Query analysis completion with extracted metadata
//! - DimensionScanned: Individual dimension scan results
//! - PathSelected: Path selection decisions
//! - DepthNavigated: Depth navigation through layers
//! - ReturnToSourceTriggered: Complexity management activation
//! - NavigationCompleted: Full navigation completion
//! - ValidationFailed: Query validation failures
//! - Error: Error events from any module
//!
//! # Usage
//! ```rust
//! use jessy::navigation::diagnostics::{DiagnosticEvent, EventEmitter};
//!
//! let emitter = EventEmitter::new();
//! emitter.emit(DiagnosticEvent::QueryAnalyzed {
//!     query: "test query".to_string(),
//!     keywords: vec!["test".to_string()],
//!     question_type: QuestionType::Factual,
//!     urgency: UrgencyLevel::Low,
//!     frequency: 1.5,
//!     duration_ms: 3,
//! });
//! ```
//!
//! Requirements: 10.1-10.5

use crate::{DimensionId, LayerId};
use crate::navigation::{QuestionType, UrgencyLevel};
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

/// Diagnostic event types for navigation system
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "event_type", rename_all = "snake_case")]
pub enum DiagnosticEvent {
    /// Query analysis completed
    QueryAnalyzed {
        /// Original query text
        query: String,
        /// Extracted keywords
        keywords: Vec<String>,
        /// Classified question type
        question_type: QuestionType,
        /// Assigned urgency level
        urgency: UrgencyLevel,
        /// Estimated frequency in Hz
        frequency: f32,
        /// Number of emotional indicators found
        emotional_count: usize,
        /// Number of technical indicators found
        technical_count: usize,
        /// Estimated complexity score
        complexity: f32,
        /// Analysis duration in milliseconds
        duration_ms: u64,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Individual dimension scanned
    DimensionScanned {
        /// Dimension identifier
        dimension_id: DimensionId,
        /// Dimension name
        dimension_name: String,
        /// Calculated confidence score
        confidence: f32,
        /// Keywords matched in this dimension
        matched_keywords: Vec<String>,
        /// Number of keywords matched
        match_count: usize,
        /// Scan duration in milliseconds
        duration_ms: u64,
        /// Whether dimension passed confidence threshold
        activated: bool,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Navigation path selected
    PathSelected {
        /// Dimension identifier
        dimension_id: DimensionId,
        /// Final confidence score
        confidence: f32,
        /// Keyword match score component
        keyword_score: f32,
        /// Frequency alignment score component
        frequency_score: f32,
        /// Number of matched keywords
        matched_keywords_count: usize,
        /// Whether path was selected (vs filtered out)
        selected: bool,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Depth navigation through layers
    DepthNavigated {
        /// Dimension identifier
        dimension_id: DimensionId,
        /// Layer sequence traversed
        layer_sequence: Vec<LayerId>,
        /// Final depth reached
        depth: usize,
        /// Match scores at each layer
        layer_scores: Vec<f32>,
        /// Navigation duration in milliseconds
        duration_ms: u64,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Return-to-source complexity management triggered
    ReturnToSourceTriggered {
        /// Original number of activated dimensions
        original_dimension_count: usize,
        /// Reduced number of dimensions
        reduced_dimension_count: usize,
        /// Complexity score that triggered return-to-source
        complexity_score: f32,
        /// Dimensions that were kept
        kept_dimensions: Vec<DimensionId>,
        /// Dimensions that were filtered out
        filtered_dimensions: Vec<DimensionId>,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Navigation completed successfully
    NavigationCompleted {
        /// Original query
        query: String,
        /// Number of dimensions activated
        dimensions_activated: usize,
        /// Dimension IDs activated
        dimension_ids: Vec<DimensionId>,
        /// Average confidence across paths
        avg_confidence: f32,
        /// Whether return-to-source was triggered
        return_to_source: bool,
        /// Query analysis duration
        query_analysis_ms: u64,
        /// Dimension scan duration
        dimension_scan_ms: u64,
        /// Path selection duration
        path_selection_ms: u64,
        /// Depth navigation duration
        depth_navigation_ms: u64,
        /// Total navigation duration
        total_duration_ms: u64,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Query validation failed
    ValidationFailed {
        /// Query that failed validation
        query: String,
        /// Validation error type
        error_type: String,
        /// Error message
        error_message: String,
        /// Event timestamp
        timestamp: u64,
    },
    
    /// Error occurred during navigation
    Error {
        /// Module where error occurred
        module: String,
        /// Error type/category
        error_type: String,
        /// Error message
        message: String,
        /// Query being processed (if available)
        query: Option<String>,
        /// Event timestamp
        timestamp: u64,
    },
}

impl DiagnosticEvent {
    /// Get event timestamp
    pub fn timestamp(&self) -> u64 {
        match self {
            Self::QueryAnalyzed { timestamp, .. } => *timestamp,
            Self::DimensionScanned { timestamp, .. } => *timestamp,
            Self::PathSelected { timestamp, .. } => *timestamp,
            Self::DepthNavigated { timestamp, .. } => *timestamp,
            Self::ReturnToSourceTriggered { timestamp, .. } => *timestamp,
            Self::NavigationCompleted { timestamp, .. } => *timestamp,
            Self::ValidationFailed { timestamp, .. } => *timestamp,
            Self::Error { timestamp, .. } => *timestamp,
        }
    }
    
    /// Get event type as string
    pub fn event_type(&self) -> &str {
        match self {
            Self::QueryAnalyzed { .. } => "query_analyzed",
            Self::DimensionScanned { .. } => "dimension_scanned",
            Self::PathSelected { .. } => "path_selected",
            Self::DepthNavigated { .. } => "depth_navigated",
            Self::ReturnToSourceTriggered { .. } => "return_to_source_triggered",
            Self::NavigationCompleted { .. } => "navigation_completed",
            Self::ValidationFailed { .. } => "validation_failed",
            Self::Error { .. } => "error",
        }
    }
    
    /// Serialize event to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }
    
    /// Serialize event to pretty JSON string
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
    
    /// Get current Unix timestamp in milliseconds
    fn now() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64
    }
    
    /// Create QueryAnalyzed event
    pub fn query_analyzed(
        query: String,
        keywords: Vec<String>,
        question_type: QuestionType,
        urgency: UrgencyLevel,
        frequency: f32,
        emotional_count: usize,
        technical_count: usize,
        complexity: f32,
        duration_ms: u64,
    ) -> Self {
        Self::QueryAnalyzed {
            query,
            keywords,
            question_type,
            urgency,
            frequency,
            emotional_count,
            technical_count,
            complexity,
            duration_ms,
            timestamp: Self::now(),
        }
    }
    
    /// Create DimensionScanned event
    pub fn dimension_scanned(
        dimension_id: DimensionId,
        dimension_name: String,
        confidence: f32,
        matched_keywords: Vec<String>,
        duration_ms: u64,
        activated: bool,
    ) -> Self {
        let match_count = matched_keywords.len();
        Self::DimensionScanned {
            dimension_id,
            dimension_name,
            confidence,
            matched_keywords,
            match_count,
            duration_ms,
            activated,
            timestamp: Self::now(),
        }
    }
    
    /// Create PathSelected event
    pub fn path_selected(
        dimension_id: DimensionId,
        confidence: f32,
        keyword_score: f32,
        frequency_score: f32,
        matched_keywords_count: usize,
        selected: bool,
    ) -> Self {
        Self::PathSelected {
            dimension_id,
            confidence,
            keyword_score,
            frequency_score,
            matched_keywords_count,
            selected,
            timestamp: Self::now(),
        }
    }
    
    /// Create DepthNavigated event
    pub fn depth_navigated(
        dimension_id: DimensionId,
        layer_sequence: Vec<LayerId>,
        layer_scores: Vec<f32>,
        duration_ms: u64,
    ) -> Self {
        let depth = layer_sequence.len();
        Self::DepthNavigated {
            dimension_id,
            layer_sequence,
            depth,
            layer_scores,
            duration_ms,
            timestamp: Self::now(),
        }
    }
    
    /// Create ReturnToSourceTriggered event
    pub fn return_to_source_triggered(
        original_dimension_count: usize,
        reduced_dimension_count: usize,
        complexity_score: f32,
        kept_dimensions: Vec<DimensionId>,
        filtered_dimensions: Vec<DimensionId>,
    ) -> Self {
        Self::ReturnToSourceTriggered {
            original_dimension_count,
            reduced_dimension_count,
            complexity_score,
            kept_dimensions,
            filtered_dimensions,
            timestamp: Self::now(),
        }
    }
    
    /// Create NavigationCompleted event
    pub fn navigation_completed(
        query: String,
        dimensions_activated: usize,
        dimension_ids: Vec<DimensionId>,
        avg_confidence: f32,
        return_to_source: bool,
        query_analysis_ms: u64,
        dimension_scan_ms: u64,
        path_selection_ms: u64,
        depth_navigation_ms: u64,
        total_duration_ms: u64,
    ) -> Self {
        Self::NavigationCompleted {
            query,
            dimensions_activated,
            dimension_ids,
            avg_confidence,
            return_to_source,
            query_analysis_ms,
            dimension_scan_ms,
            path_selection_ms,
            depth_navigation_ms,
            total_duration_ms,
            timestamp: Self::now(),
        }
    }
    
    /// Create ValidationFailed event
    pub fn validation_failed(
        query: String,
        error_type: String,
        error_message: String,
    ) -> Self {
        Self::ValidationFailed {
            query,
            error_type,
            error_message,
            timestamp: Self::now(),
        }
    }
    
    /// Create Error event
    pub fn error(
        module: String,
        error_type: String,
        message: String,
        query: Option<String>,
    ) -> Self {
        Self::Error {
            module,
            error_type,
            message,
            query,
            timestamp: Self::now(),
        }
    }
}

/// Event emitter for diagnostic events
///
/// Collects and manages diagnostic events with support for:
/// - Event collection and retrieval
/// - JSON export
/// - Event filtering
/// - Thread-safe operation
#[derive(Debug, Clone)]
pub struct EventEmitter {
    events: Arc<Mutex<Vec<DiagnosticEvent>>>,
    enabled: Arc<Mutex<bool>>,
}

impl EventEmitter {
    /// Create new event emitter
    pub fn new() -> Self {
        Self {
            events: Arc::new(Mutex::new(Vec::new())),
            enabled: Arc::new(Mutex::new(true)),
        }
    }
    
    /// Enable event emission
    pub fn enable(&self) {
        *self.enabled.lock().unwrap() = true;
    }
    
    /// Disable event emission
    pub fn disable(&self) {
        *self.enabled.lock().unwrap() = false;
    }
    
    /// Check if event emission is enabled
    pub fn is_enabled(&self) -> bool {
        *self.enabled.lock().unwrap()
    }
    
    /// Emit a diagnostic event
    pub fn emit(&self, event: DiagnosticEvent) {
        if self.is_enabled() {
            self.events.lock().unwrap().push(event);
        }
    }
    
    /// Get all collected events
    pub fn events(&self) -> Vec<DiagnosticEvent> {
        self.events.lock().unwrap().clone()
    }
    
    /// Get events of a specific type
    pub fn events_by_type(&self, event_type: &str) -> Vec<DiagnosticEvent> {
        self.events()
            .into_iter()
            .filter(|e| e.event_type() == event_type)
            .collect()
    }
    
    /// Clear all collected events
    pub fn clear(&self) {
        self.events.lock().unwrap().clear();
    }
    
    /// Get event count
    pub fn count(&self) -> usize {
        self.events.lock().unwrap().len()
    }
    
    /// Export all events as JSON array
    pub fn export_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(&self.events())
    }
    
    /// Export all events as pretty JSON array
    pub fn export_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(&self.events())
    }
    
    /// Export events to JSON Lines format (one event per line)
    pub fn export_jsonl(&self) -> Result<String, serde_json::Error> {
        let events = self.events();
        let mut lines = Vec::new();
        for event in events {
            lines.push(event.to_json()?);
        }
        Ok(lines.join("\n"))
    }
}

impl Default for EventEmitter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_diagnostic_event_creation() {
        let event = DiagnosticEvent::query_analyzed(
            "test query".to_string(),
            vec!["test".to_string()],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.5,
            0,
            1,
            2.0,
            5,
        );
        
        assert_eq!(event.event_type(), "query_analyzed");
        assert!(event.timestamp() > 0);
    }
    
    #[test]
    fn test_event_json_serialization() {
        let event = DiagnosticEvent::dimension_scanned(
            DimensionId(1),
            "Emotion".to_string(),
            0.85,
            vec!["happy".to_string()],
            10,
            true,
        );
        
        let json = event.to_json().unwrap();
        assert!(json.contains("dimension_scanned"));
        assert!(json.contains("Emotion"));
        assert!(json.contains("0.85"));
    }
    
    #[test]
    fn test_event_emitter_basic() {
        let emitter = EventEmitter::new();
        assert_eq!(emitter.count(), 0);
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        assert_eq!(emitter.count(), 1);
    }
    
    #[test]
    fn test_event_emitter_enable_disable() {
        let emitter = EventEmitter::new();
        assert!(emitter.is_enabled());
        
        emitter.disable();
        assert!(!emitter.is_enabled());
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        assert_eq!(emitter.count(), 0); // Event not collected when disabled
        
        emitter.enable();
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        assert_eq!(emitter.count(), 1);
    }
    
    #[test]
    fn test_event_emitter_filter_by_type() {
        let emitter = EventEmitter::new();
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        emitter.emit(DiagnosticEvent::dimension_scanned(
            DimensionId(1),
            "Emotion".to_string(),
            0.8,
            vec![],
            10,
            true,
        ));
        
        emitter.emit(DiagnosticEvent::dimension_scanned(
            DimensionId(2),
            "Cognition".to_string(),
            0.7,
            vec![],
            12,
            true,
        ));
        
        let query_events = emitter.events_by_type("query_analyzed");
        assert_eq!(query_events.len(), 1);
        
        let scan_events = emitter.events_by_type("dimension_scanned");
        assert_eq!(scan_events.len(), 2);
    }
    
    #[test]
    fn test_event_emitter_clear() {
        let emitter = EventEmitter::new();
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        assert_eq!(emitter.count(), 1);
        
        emitter.clear();
        assert_eq!(emitter.count(), 0);
    }
    
    #[test]
    fn test_event_emitter_json_export() {
        let emitter = EventEmitter::new();
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test query".to_string(),
            vec!["test".to_string()],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.5,
            0,
            1,
            2.0,
            5,
        ));
        
        let json = emitter.export_json().unwrap();
        assert!(json.starts_with('['));
        assert!(json.ends_with(']'));
        assert!(json.contains("query_analyzed"));
    }
    
    #[test]
    fn test_event_emitter_jsonl_export() {
        let emitter = EventEmitter::new();
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test1".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test2".to_string(),
            vec![],
            QuestionType::Technical,
            UrgencyLevel::Medium,
            2.0,
            0,
            1,
            2.0,
            6,
        ));
        
        let jsonl = emitter.export_jsonl().unwrap();
        let lines: Vec<&str> = jsonl.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].contains("test1"));
        assert!(lines[1].contains("test2"));
    }
    
    #[test]
    fn test_all_event_types() {
        let emitter = EventEmitter::new();
        
        // QueryAnalyzed
        emitter.emit(DiagnosticEvent::query_analyzed(
            "test".to_string(),
            vec![],
            QuestionType::Factual,
            UrgencyLevel::Low,
            1.0,
            0,
            0,
            1.0,
            5,
        ));
        
        // DimensionScanned
        emitter.emit(DiagnosticEvent::dimension_scanned(
            DimensionId(1),
            "Emotion".to_string(),
            0.8,
            vec![],
            10,
            true,
        ));
        
        // PathSelected
        emitter.emit(DiagnosticEvent::path_selected(
            DimensionId(1),
            0.85,
            0.8,
            1.0,
            5,
            true,
        ));
        
        // DepthNavigated
        emitter.emit(DiagnosticEvent::depth_navigated(
            DimensionId(1),
            vec![LayerId { dimension: DimensionId(1), layer: 0 }],
            vec![0.9],
            15,
        ));
        
        // ReturnToSourceTriggered
        emitter.emit(DiagnosticEvent::return_to_source_triggered(
            10,
            3,
            7.5,
            vec![DimensionId(1), DimensionId(2), DimensionId(3)],
            vec![DimensionId(4), DimensionId(5)],
        ));
        
        // NavigationCompleted
        emitter.emit(DiagnosticEvent::navigation_completed(
            "test".to_string(),
            3,
            vec![DimensionId(1), DimensionId(2), DimensionId(3)],
            0.82,
            false,
            5,
            95,
            10,
            20,
            130,
        ));
        
        // ValidationFailed
        emitter.emit(DiagnosticEvent::validation_failed(
            "".to_string(),
            "EmptyQuery".to_string(),
            "Query cannot be empty".to_string(),
        ));
        
        // Error
        emitter.emit(DiagnosticEvent::error(
            "scanner".to_string(),
            "Timeout".to_string(),
            "Scan timeout exceeded".to_string(),
            Some("test".to_string()),
        ));
        
        assert_eq!(emitter.count(), 8);
    }
    
    #[test]
    fn test_event_emitter_thread_safety() {
        use std::thread;
        
        let emitter = Arc::new(EventEmitter::new());
        let mut handles = vec![];
        
        // Spawn 10 threads that each emit 10 events
        for i in 0..10 {
            let emitter_clone = Arc::clone(&emitter);
            let handle = thread::spawn(move || {
                for j in 0..10 {
                    emitter_clone.emit(DiagnosticEvent::query_analyzed(
                        format!("thread-{}-query-{}", i, j),
                        vec![],
                        QuestionType::Factual,
                        UrgencyLevel::Low,
                        1.0,
                        0,
                        0,
                        1.0,
                        5,
                    ));
                }
            });
            handles.push(handle);
        }
        
        // Wait for all threads
        for handle in handles {
            handle.join().unwrap();
        }
        
        // Should have exactly 100 events
        assert_eq!(emitter.count(), 100);
    }
}
