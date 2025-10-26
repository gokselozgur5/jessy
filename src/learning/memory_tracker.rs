// Memory tracking for learning system components
//
// Tracks memory usage across:
// - Observation buffer (circular buffer)
// - Proto-dimensions (heap allocations)
// - Synesthetic associations
//
// Enforces 500MB total limit for learning system

use std::sync::atomic::{AtomicUsize, Ordering};

/// Memory tracker for learning system
///
/// Tracks memory usage and enforces limits to prevent unbounded growth.
/// All memory tracking is thread-safe using atomic operations.
pub struct MemoryTracker {
    /// Observation buffer memory (bytes)
    observation_memory: AtomicUsize,
    
    /// Proto-dimension memory (bytes)
    proto_dimension_memory: AtomicUsize,
    
    /// Synesthetic data memory (bytes)
    synesthetic_memory: AtomicUsize,
    
    /// Total memory limit (bytes)
    limit: usize,
    
    /// Warning threshold (90% of limit)
    warning_threshold: usize,
}

impl MemoryTracker {
    /// Create new memory tracker with 500MB limit
    pub fn new() -> Self {
        Self::with_limit(500 * 1024 * 1024) // 500MB
    }
    
    /// Create memory tracker with custom limit
    pub fn with_limit(limit: usize) -> Self {
        Self {
            observation_memory: AtomicUsize::new(0),
            proto_dimension_memory: AtomicUsize::new(0),
            synesthetic_memory: AtomicUsize::new(0),
            limit,
            warning_threshold: (limit as f64 * 0.9) as usize,
        }
    }
    
    /// Get total memory usage across all components
    pub fn total_usage(&self) -> usize {
        self.observation_memory.load(Ordering::Relaxed)
            + self.proto_dimension_memory.load(Ordering::Relaxed)
            + self.synesthetic_memory.load(Ordering::Relaxed)
    }
    
    /// Get observation buffer memory usage
    pub fn observation_usage(&self) -> usize {
        self.observation_memory.load(Ordering::Relaxed)
    }
    
    /// Get proto-dimension memory usage
    pub fn proto_dimension_usage(&self) -> usize {
        self.proto_dimension_memory.load(Ordering::Relaxed)
    }
    
    /// Get synesthetic data memory usage
    pub fn synesthetic_usage(&self) -> usize {
        self.synesthetic_memory.load(Ordering::Relaxed)
    }
    
    /// Get memory limit
    pub fn limit(&self) -> usize {
        self.limit
    }
    
    /// Check if allocation would exceed limit
    pub fn can_allocate(&self, size: usize) -> bool {
        self.total_usage() + size <= self.limit
    }
    
    /// Check if usage is above warning threshold (90%)
    pub fn is_above_warning_threshold(&self) -> bool {
        self.total_usage() >= self.warning_threshold
    }
    
    /// Get percentage of limit used
    pub fn usage_percentage(&self) -> f64 {
        (self.total_usage() as f64 / self.limit as f64) * 100.0
    }
    
    /// Add observation memory
    pub fn add_observation_memory(&self, size: usize) {
        self.observation_memory.fetch_add(size, Ordering::Relaxed);
    }
    
    /// Remove observation memory
    pub fn remove_observation_memory(&self, size: usize) {
        self.observation_memory.fetch_sub(size, Ordering::Relaxed);
    }
    
    /// Add proto-dimension memory
    pub fn add_proto_dimension_memory(&self, size: usize) {
        self.proto_dimension_memory.fetch_add(size, Ordering::Relaxed);
    }
    
    /// Remove proto-dimension memory
    pub fn remove_proto_dimension_memory(&self, size: usize) {
        self.proto_dimension_memory.fetch_sub(size, Ordering::Relaxed);
    }
    
    /// Add synesthetic memory
    pub fn add_synesthetic_memory(&self, size: usize) {
        self.synesthetic_memory.fetch_add(size, Ordering::Relaxed);
    }
    
    /// Remove synesthetic memory
    pub fn remove_synesthetic_memory(&self, size: usize) {
        self.synesthetic_memory.fetch_sub(size, Ordering::Relaxed);
    }
}

impl Default for MemoryTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_new_tracker_starts_at_zero() {
        let tracker = MemoryTracker::new();
        assert_eq!(tracker.total_usage(), 0);
        assert_eq!(tracker.observation_usage(), 0);
        assert_eq!(tracker.proto_dimension_usage(), 0);
        assert_eq!(tracker.synesthetic_usage(), 0);
    }
    
    #[test]
    fn test_default_limit_is_500mb() {
        let tracker = MemoryTracker::new();
        assert_eq!(tracker.limit(), 500 * 1024 * 1024);
    }
    
    #[test]
    fn test_custom_limit() {
        let tracker = MemoryTracker::with_limit(100 * 1024 * 1024);
        assert_eq!(tracker.limit(), 100 * 1024 * 1024);
    }
    
    #[test]
    fn test_observation_memory_tracking() {
        let tracker = MemoryTracker::new();
        
        tracker.add_observation_memory(1024);
        assert_eq!(tracker.observation_usage(), 1024);
        assert_eq!(tracker.total_usage(), 1024);
        
        tracker.add_observation_memory(2048);
        assert_eq!(tracker.observation_usage(), 3072);
        assert_eq!(tracker.total_usage(), 3072);
        
        tracker.remove_observation_memory(1024);
        assert_eq!(tracker.observation_usage(), 2048);
        assert_eq!(tracker.total_usage(), 2048);
    }
    
    #[test]
    fn test_proto_dimension_memory_tracking() {
        let tracker = MemoryTracker::new();
        
        tracker.add_proto_dimension_memory(16 * 1024 * 1024); // 16MB
        assert_eq!(tracker.proto_dimension_usage(), 16 * 1024 * 1024);
        assert_eq!(tracker.total_usage(), 16 * 1024 * 1024);
        
        tracker.remove_proto_dimension_memory(16 * 1024 * 1024);
        assert_eq!(tracker.proto_dimension_usage(), 0);
        assert_eq!(tracker.total_usage(), 0);
    }
    
    #[test]
    fn test_synesthetic_memory_tracking() {
        let tracker = MemoryTracker::new();
        
        tracker.add_synesthetic_memory(10 * 1024 * 1024); // 10MB
        assert_eq!(tracker.synesthetic_usage(), 10 * 1024 * 1024);
        assert_eq!(tracker.total_usage(), 10 * 1024 * 1024);
    }
    
    #[test]
    fn test_total_usage_calculation() {
        let tracker = MemoryTracker::new();
        
        tracker.add_observation_memory(1 * 1024 * 1024);      // 1MB
        tracker.add_proto_dimension_memory(16 * 1024 * 1024); // 16MB
        tracker.add_synesthetic_memory(10 * 1024 * 1024);     // 10MB
        
        assert_eq!(tracker.total_usage(), 27 * 1024 * 1024); // 27MB
    }
    
    #[test]
    fn test_can_allocate_within_limit() {
        let tracker = MemoryTracker::with_limit(100 * 1024 * 1024); // 100MB
        
        tracker.add_observation_memory(50 * 1024 * 1024); // 50MB used
        
        assert!(tracker.can_allocate(40 * 1024 * 1024)); // 90MB total - OK
        assert!(tracker.can_allocate(50 * 1024 * 1024)); // 100MB total - OK
        assert!(!tracker.can_allocate(51 * 1024 * 1024)); // 101MB total - FAIL
    }
    
    #[test]
    fn test_warning_threshold_at_90_percent() {
        let tracker = MemoryTracker::with_limit(100 * 1024 * 1024); // 100MB
        
        assert!(!tracker.is_above_warning_threshold());
        
        tracker.add_observation_memory(89 * 1024 * 1024); // 89MB
        assert!(!tracker.is_above_warning_threshold());
        
        tracker.add_observation_memory(1 * 1024 * 1024); // 90MB
        assert!(tracker.is_above_warning_threshold());
    }
    
    #[test]
    fn test_usage_percentage() {
        let tracker = MemoryTracker::with_limit(100 * 1024 * 1024); // 100MB
        
        assert_eq!(tracker.usage_percentage(), 0.0);
        
        tracker.add_observation_memory(50 * 1024 * 1024); // 50MB
        assert!((tracker.usage_percentage() - 50.0).abs() < 0.01);
        
        tracker.add_proto_dimension_memory(25 * 1024 * 1024); // 75MB total
        assert!((tracker.usage_percentage() - 75.0).abs() < 0.01);
    }
    
    #[test]
    fn test_thread_safety() {
        use std::sync::Arc;
        use std::thread;
        
        let tracker = Arc::new(MemoryTracker::new());
        let mut handles = vec![];
        
        // Spawn 10 threads, each adding 1MB
        for _ in 0..10 {
            let tracker_clone = tracker.clone();
            let handle = thread::spawn(move || {
                tracker_clone.add_observation_memory(1024 * 1024);
            });
            handles.push(handle);
        }
        
        for handle in handles {
            handle.join().unwrap();
        }
        
        assert_eq!(tracker.total_usage(), 10 * 1024 * 1024);
    }
}
