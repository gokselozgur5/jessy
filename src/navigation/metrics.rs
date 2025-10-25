//! Metrics collection and monitoring for navigation system
//!
//! This module provides metrics tracking for:
//! - Dimensions activated count (histogram)
//! - Confidence score distribution (histogram)
//! - Return-to-source rate (counter)
//! - Scan timeouts (counter)
//! - Insufficient matches (counter)
//! - Validation errors (counter)
//! - Queries per second (counter)
//! - Concurrent requests (gauge)
//!
//! Requirements: 10.3-10.5

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Navigation metrics collector
///
/// Tracks various metrics about navigation system performance and behavior.
/// All metrics are thread-safe and can be accessed concurrently.
///
/// # Thread Safety
/// Uses atomic operations for lock-free metric updates.
#[derive(Debug, Clone)]
pub struct NavigationMetrics {
    /// Total number of queries processed
    queries_total: Arc<AtomicU64>,
    
    /// Number of queries that triggered return-to-source
    return_to_source_total: Arc<AtomicU64>,
    
    /// Number of scan timeouts
    scan_timeouts_total: Arc<AtomicU64>,
    
    /// Number of insufficient matches errors
    insufficient_matches_total: Arc<AtomicU64>,
    
    /// Number of validation errors
    validation_errors_total: Arc<AtomicU64>,
    
    /// Currently active concurrent requests
    concurrent_requests: Arc<AtomicU64>,
    
    /// Sum of dimensions activated (for calculating average)
    dimensions_activated_sum: Arc<AtomicU64>,
    
    /// Sum of confidence scores (for calculating average)
    confidence_sum: Arc<AtomicU64>,
    
    /// Count of confidence measurements
    confidence_count: Arc<AtomicU64>,
}

impl NavigationMetrics {
    /// Create a new metrics collector
    pub fn new() -> Self {
        Self {
            queries_total: Arc::new(AtomicU64::new(0)),
            return_to_source_total: Arc::new(AtomicU64::new(0)),
            scan_timeouts_total: Arc::new(AtomicU64::new(0)),
            insufficient_matches_total: Arc::new(AtomicU64::new(0)),
            validation_errors_total: Arc::new(AtomicU64::new(0)),
            concurrent_requests: Arc::new(AtomicU64::new(0)),
            dimensions_activated_sum: Arc::new(AtomicU64::new(0)),
            confidence_sum: Arc::new(AtomicU64::new(0)),
            confidence_count: Arc::new(AtomicU64::new(0)),
        }
    }
    
    /// Increment query counter
    pub fn record_query(&self) {
        self.queries_total.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Increment return-to-source counter
    pub fn record_return_to_source(&self) {
        self.return_to_source_total.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Increment scan timeout counter
    pub fn record_scan_timeout(&self) {
        self.scan_timeouts_total.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Increment insufficient matches counter
    pub fn record_insufficient_matches(&self) {
        self.insufficient_matches_total.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Increment validation error counter
    pub fn record_validation_error(&self) {
        self.validation_errors_total.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Record dimensions activated for a query
    pub fn record_dimensions_activated(&self, count: usize) {
        self.dimensions_activated_sum.fetch_add(count as u64, Ordering::Relaxed);
    }
    
    /// Record confidence score
    pub fn record_confidence(&self, confidence: f32) {
        // Store confidence as integer (multiply by 1000 for precision)
        let confidence_int = (confidence * 1000.0) as u64;
        self.confidence_sum.fetch_add(confidence_int, Ordering::Relaxed);
        self.confidence_count.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Increment concurrent requests gauge
    pub fn increment_concurrent_requests(&self) {
        self.concurrent_requests.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Decrement concurrent requests gauge
    pub fn decrement_concurrent_requests(&self) {
        self.concurrent_requests.fetch_sub(1, Ordering::Relaxed);
    }
    
    /// Get total queries processed
    pub fn queries_total(&self) -> u64 {
        self.queries_total.load(Ordering::Relaxed)
    }
    
    /// Get return-to-source count
    pub fn return_to_source_total(&self) -> u64 {
        self.return_to_source_total.load(Ordering::Relaxed)
    }
    
    /// Get return-to-source rate (0.0-1.0)
    pub fn return_to_source_rate(&self) -> f64 {
        let total = self.queries_total();
        if total == 0 {
            return 0.0;
        }
        self.return_to_source_total() as f64 / total as f64
    }
    
    /// Get scan timeout count
    pub fn scan_timeouts_total(&self) -> u64 {
        self.scan_timeouts_total.load(Ordering::Relaxed)
    }
    
    /// Get scan timeout rate (0.0-1.0)
    pub fn scan_timeout_rate(&self) -> f64 {
        let total = self.queries_total();
        if total == 0 {
            return 0.0;
        }
        self.scan_timeouts_total() as f64 / total as f64
    }
    
    /// Get insufficient matches count
    pub fn insufficient_matches_total(&self) -> u64 {
        self.insufficient_matches_total.load(Ordering::Relaxed)
    }
    
    /// Get validation error count
    pub fn validation_errors_total(&self) -> u64 {
        self.validation_errors_total.load(Ordering::Relaxed)
    }
    
    /// Get current concurrent requests
    pub fn concurrent_requests(&self) -> u64 {
        self.concurrent_requests.load(Ordering::Relaxed)
    }
    
    /// Get average dimensions activated per query
    pub fn avg_dimensions_activated(&self) -> f64 {
        let total = self.queries_total();
        if total == 0 {
            return 0.0;
        }
        self.dimensions_activated_sum.load(Ordering::Relaxed) as f64 / total as f64
    }
    
    /// Get average confidence score
    pub fn avg_confidence(&self) -> f64 {
        let count = self.confidence_count.load(Ordering::Relaxed);
        if count == 0 {
            return 0.0;
        }
        let sum = self.confidence_sum.load(Ordering::Relaxed);
        (sum as f64 / count as f64) / 1000.0 // Divide by 1000 to restore original scale
    }
    
    /// Get metrics snapshot as formatted string
    pub fn snapshot(&self) -> String {
        format!(
            "Navigation Metrics:\n\
             - Queries Total: {}\n\
             - Return-to-Source: {} ({:.2}%)\n\
             - Scan Timeouts: {} ({:.2}%)\n\
             - Insufficient Matches: {}\n\
             - Validation Errors: {}\n\
             - Concurrent Requests: {}\n\
             - Avg Dimensions Activated: {:.2}\n\
             - Avg Confidence: {:.3}",
            self.queries_total(),
            self.return_to_source_total(),
            self.return_to_source_rate() * 100.0,
            self.scan_timeouts_total(),
            self.scan_timeout_rate() * 100.0,
            self.insufficient_matches_total(),
            self.validation_errors_total(),
            self.concurrent_requests(),
            self.avg_dimensions_activated(),
            self.avg_confidence()
        )
    }
    
    /// Export metrics in Prometheus format
    pub fn prometheus_format(&self) -> String {
        format!(
            "# HELP navigation_queries_total Total number of navigation queries processed\n\
             # TYPE navigation_queries_total counter\n\
             navigation_queries_total {}\n\
             \n\
             # HELP navigation_return_to_source_total Number of queries that triggered return-to-source\n\
             # TYPE navigation_return_to_source_total counter\n\
             navigation_return_to_source_total {}\n\
             \n\
             # HELP navigation_scan_timeouts_total Number of dimension scan timeouts\n\
             # TYPE navigation_scan_timeouts_total counter\n\
             navigation_scan_timeouts_total {}\n\
             \n\
             # HELP navigation_insufficient_matches_total Number of insufficient matches errors\n\
             # TYPE navigation_insufficient_matches_total counter\n\
             navigation_insufficient_matches_total {}\n\
             \n\
             # HELP navigation_validation_errors_total Number of validation errors\n\
             # TYPE navigation_validation_errors_total counter\n\
             navigation_validation_errors_total {}\n\
             \n\
             # HELP navigation_concurrent_requests Current number of concurrent requests\n\
             # TYPE navigation_concurrent_requests gauge\n\
             navigation_concurrent_requests {}\n\
             \n\
             # HELP navigation_avg_dimensions_activated Average dimensions activated per query\n\
             # TYPE navigation_avg_dimensions_activated gauge\n\
             navigation_avg_dimensions_activated {:.2}\n\
             \n\
             # HELP navigation_avg_confidence Average confidence score\n\
             # TYPE navigation_avg_confidence gauge\n\
             navigation_avg_confidence {:.3}\n",
            self.queries_total(),
            self.return_to_source_total(),
            self.scan_timeouts_total(),
            self.insufficient_matches_total(),
            self.validation_errors_total(),
            self.concurrent_requests(),
            self.avg_dimensions_activated(),
            self.avg_confidence()
        )
    }
}

impl Default for NavigationMetrics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_metrics_creation() {
        let metrics = NavigationMetrics::new();
        assert_eq!(metrics.queries_total(), 0);
        assert_eq!(metrics.return_to_source_total(), 0);
        assert_eq!(metrics.concurrent_requests(), 0);
    }
    
    #[test]
    fn test_record_query() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_query();
        assert_eq!(metrics.queries_total(), 2);
    }
    
    #[test]
    fn test_record_return_to_source() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_query();
        metrics.record_return_to_source();
        
        assert_eq!(metrics.return_to_source_total(), 1);
        assert_eq!(metrics.return_to_source_rate(), 0.5); // 1/2 = 50%
    }
    
    #[test]
    fn test_record_scan_timeout() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_query();
        metrics.record_query();
        metrics.record_scan_timeout();
        
        assert_eq!(metrics.scan_timeouts_total(), 1);
        assert!((metrics.scan_timeout_rate() - 0.333).abs() < 0.01); // ~33%
    }
    
    #[test]
    fn test_record_dimensions_activated() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_dimensions_activated(3);
        metrics.record_query();
        metrics.record_dimensions_activated(5);
        
        assert_eq!(metrics.avg_dimensions_activated(), 4.0); // (3+5)/2 = 4
    }
    
    #[test]
    fn test_record_confidence() {
        let metrics = NavigationMetrics::new();
        metrics.record_confidence(0.8);
        metrics.record_confidence(0.6);
        metrics.record_confidence(1.0);
        
        assert!((metrics.avg_confidence() - 0.8).abs() < 0.01); // (0.8+0.6+1.0)/3 = 0.8
    }
    
    #[test]
    fn test_concurrent_requests() {
        let metrics = NavigationMetrics::new();
        metrics.increment_concurrent_requests();
        metrics.increment_concurrent_requests();
        assert_eq!(metrics.concurrent_requests(), 2);
        
        metrics.decrement_concurrent_requests();
        assert_eq!(metrics.concurrent_requests(), 1);
    }
    
    #[test]
    fn test_snapshot_format() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_dimensions_activated(3);
        metrics.record_confidence(0.75);
        
        let snapshot = metrics.snapshot();
        assert!(snapshot.contains("Queries Total: 1"));
        assert!(snapshot.contains("Avg Dimensions Activated: 3.00"));
        assert!(snapshot.contains("Avg Confidence: 0.750"));
    }
    
    #[test]
    fn test_prometheus_format() {
        let metrics = NavigationMetrics::new();
        metrics.record_query();
        metrics.record_return_to_source();
        
        let prometheus = metrics.prometheus_format();
        assert!(prometheus.contains("navigation_queries_total 1"));
        assert!(prometheus.contains("navigation_return_to_source_total 1"));
        assert!(prometheus.contains("# TYPE"));
        assert!(prometheus.contains("# HELP"));
    }
    
    #[test]
    fn test_metrics_thread_safety() {
        use std::thread;
        
        let metrics = Arc::new(NavigationMetrics::new());
        let mut handles = vec![];
        
        // Spawn 10 threads that each record 100 queries
        for _ in 0..10 {
            let metrics_clone = Arc::clone(&metrics);
            let handle = thread::spawn(move || {
                for _ in 0..100 {
                    metrics_clone.record_query();
                }
            });
            handles.push(handle);
        }
        
        // Wait for all threads
        for handle in handles {
            handle.join().unwrap();
        }
        
        // Should have exactly 1000 queries
        assert_eq!(metrics.queries_total(), 1000);
    }
}
