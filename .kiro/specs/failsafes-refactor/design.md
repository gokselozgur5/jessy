# Design Document - Failsafes System Refactor

## Overview

This document describes the design for refactoring the failsafes package from Python to Rust. The failsafes system is **SAFETY-CRITICAL** - it provides the last line of defense against sensor failures, data quality issues, and system malfunctions. Any bug in this system could result in drone crashes, property damage, or injury.

### Design Philosophy

1. **Safety First**: When in doubt, fail safe (block output rather than pass bad data)
2. **Determinism**: Predictable behavior in all scenarios
3. **Observability**: Comprehensive logging and diagnostics
4. **Testability**: Extensive testing including fault injection
5. **Backward Compatibility**: 100% compatible with Python for smooth migration

### Design Goals

1. **Reliability**: Zero false negatives (missed failures), minimal false positives
2. **Performance**: < 500μs latency with deterministic execution
3. **Maintainability**: Clear state machines, well-documented logic
4. **Safety**: Memory-safe, no panics, comprehensive error handling
5. **Observability**: Rich diagnostics for monitoring and debugging

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Failsafes System                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────┐         ┌──────────────────┐              │
│  │   Odometry       │         │   NavSatFix      │              │
│  │   Failsafe       │◄────────┤   Failsafe       │              │
│  └────────┬─────────┘         └────────┬─────────┘              │
│           │                            │                         │
│           │  Status                    │  Status                 │
│           ▼                            ▼                         │
│  ┌──────────────────────────────────────────────┐               │
│  │         Centralized Error Manager             │               │
│  └──────────────────────────────────────────────┘               │
│           │                            │                         │
│           ▼                            ▼                         │
│  ┌──────────────────┐         ┌──────────────────┐              │
│  │   Hz Monitor     │         │   Error Node     │              │
│  └──────────────────┘         └──────────────────┘              │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Component Architecture

```
failsafes/
├── rust/
│   ├── odometry_failsafe/
│   │   ├── src/
│   │   │   ├── lib.rs              # Public API
│   │   │   ├── node.rs             # ROS2 node
│   │   │   ├── checks.rs           # Quality/twist/delay checks
│   │   │   ├── state_machine.rs    # Failsafe state machine
│   │   │   ├── config.rs           # Configuration
│   │   │   └── types.rs            # Common types
│   │   ├── tests/
│   │   │   ├── checks_tests.rs
│   │   │   ├── state_machine_tests.rs
│   │   │   ├── integration_tests.rs
│   │   │   ├── compatibility_tests.rs
│   │   │   └── fault_injection_tests.rs
│   │   └── Cargo.toml
│   ├── navsatfix_failsafe/
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── node.rs
│   │   │   ├── checks.rs           # Distance/delay checks
│   │   │   ├── state_machine.rs
│   │   │   ├── config.rs
│   │   │   └── types.rs
│   │   ├── tests/
│   │   └── Cargo.toml
│   ├── hz_monitor/
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── node.rs
│   │   │   ├── rate_tracker.rs     # Message rate tracking
│   │   │   └── config.rs
│   │   ├── tests/
│   │   └── Cargo.toml
│   └── error_node/
│       ├── src/
│       │   ├── lib.rs
│       │   ├── node.rs
│       │   ├── aggregator.rs       # Error aggregation
│       │   └── config.rs
│       ├── tests/
│       └── Cargo.toml
├── config/
│   ├── odometry_failsafe.yaml
│   ├── navsatfix_failsafe.yaml
│   ├── hz_monitor.yaml
│   └── error_node.yaml
├── launch/
│   ├── odometry_failsafe.launch.py
│   ├── navsatfix_failsafe.launch.py
│   ├── hz_monitor.launch.py
│   ├── error_node.launch.py
│   └── failsafes_system.launch.py
└── README.md
```

## Components and Interfaces

### 1. Odometry Failsafe

#### State Machine

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeStatus {
    Ok,                    // All checks pass, using input
    Unhealthy,             // Input unhealthy, using ground truth
    Quality,               // Quality check failed, using ground truth
    Twist,                 // Twist limits exceeded, using ground truth
    Delay,                 // Delay threshold exceeded, using ground truth
    Disabled,              // Failsafe disabled, passthrough mode
    GroundTruthBad,        // Ground truth unhealthy, blocking output
}

impl FailsafeStatus {
    pub fn source(&self) -> OdometrySource {
        match self {
            Self::Ok | Self::Disabled => OdometrySource::Input,
            _ => OdometrySource::GroundTruth,
        }
    }
    
    pub fn severity(&self) -> Severity {
        match self {
            Self::Ok => Severity::Info,
            Self::Disabled => Severity::Warning,
            _ => Severity::Error,
        }
    }
}
```

#### Quality Checking

```rust
pub struct QualityChecker {
    buffer: VecDeque<f64>,
    buffer_size: usize,
    threshold: f64,
}

impl QualityChecker {
    /// Compute quality metric from twist values
    pub fn compute_quality(&self, twist: &Twist) -> f64 {
        // Quality metric based on twist magnitude and consistency
        let linear_mag = (twist.linear.x.powi(2) + 
                         twist.linear.y.powi(2) + 
                         twist.linear.z.powi(2)).sqrt();
        
        // Normalize to 0-1 range (implementation matches Python)
        // Lower values indicate better quality
        linear_mag / self.threshold
    }
    
    /// Add quality measurement to buffer
    pub fn add_measurement(&mut self, quality: f64) {
        self.buffer.push_back(quality);
        if self.buffer.len() > self.buffer_size {
            self.buffer.pop_front();
        }
    }
    
    /// Check if quality buffer indicates healthy data
    pub fn is_quality_acceptable(&self) -> bool {
        if self.buffer.len() < self.buffer_size {
            return false;  // Not enough data yet
        }
        
        // Compute statistics over buffer
        let mean = self.buffer.iter().sum::<f64>() / self.buffer.len() as f64;
        mean < self.threshold
    }
}
```

#### Twist Limit Checking

```rust
pub struct TwistLimits {
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

impl TwistLimits {
    /// Check if twist is within limits
    pub fn check(&self, twist: &Twist) -> Result<(), TwistLimitError> {
        if twist.linear.x.abs() > self.max_x {
            return Err(TwistLimitError::XExceeded {
                value: twist.linear.x,
                limit: self.max_x,
            });
        }
        
        if twist.linear.y.abs() > self.max_y {
            return Err(TwistLimitError::YExceeded {
                value: twist.linear.y,
                limit: self.max_y,
            });
        }
        
        if twist.linear.z.abs() > self.max_z {
            return Err(TwistLimitError::ZExceeded {
                value: twist.linear.z,
                limit: self.max_z,
            });
        }
        
        Ok(())
    }
}
```

#### Delay Checking

```rust
pub struct DelayChecker {
    max_delay: Duration,
}

impl DelayChecker {
    /// Check if message is within delay threshold
    pub fn check(&self, msg_time: Time, current_time: Time) -> Result<(), DelayError> {
        let age = current_time - msg_time;
        
        if age > self.max_delay {
            return Err(DelayError::Exceeded {
                age,
                threshold: self.max_delay,
            });
        }
        
        if age < Duration::ZERO {
            return Err(DelayError::FutureTimestamp {
                age,
            });
        }
        
        Ok(())
    }
}
```

#### Node Implementation

```rust
pub struct OdometryFailsafeNode {
    node: Arc<Node>,
    config: OdometryFailsafeConfig,
    
    // State
    current_status: FailsafeStatus,
    ground_truth_last: Option<Odometry>,
    input_last: Option<Odometry>,
    
    // Checkers
    quality_checker: QualityChecker,
    twist_limits: TwistLimits,
    delay_checker: DelayChecker,
    
    // Health tracking
    ground_truth_healthy: bool,
    input_healthy: bool,
    quality_within_limits: bool,
    twist_within_limits: bool,
    delay_within_limits: bool,
    
    // Subscribers
    ground_truth_sub: Subscription<Odometry>,
    input_sub: Subscription<Odometry>,
    
    // Publishers
    output_pub: Publisher<Odometry>,
    status_pub: Publisher<StatusText>,
    
    // Diagnostics
    error_manager: Arc<ErrorManager>,
    performance_monitor: Arc<PerformanceMonitor>,
    
    // Monitoring timer
    monitor_timer: Timer,
}

impl OdometryFailsafeNode {
    pub fn new(config: OdometryFailsafeConfig) -> Result<Self, NodeError>;
    
    fn ground_truth_callback(&mut self, msg: Odometry);
    fn input_callback(&mut self, msg: Odometry);
    fn monitor_callback(&mut self);
    
    fn update_checks(&mut self);
    fn determine_status(&self) -> FailsafeStatus;
    fn publish_status(&self, header: &Header);
    fn report_diagnostics(&self);
}
```

### 2. NavSatFix Failsafe

#### State Machine

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavSatFixFailsafeStatus {
    Ok,                    // All checks pass, using input
    Delay,                 // Delay check failed
    Distance,              // Distance check failed
    OdomStatusTimeout,     // Odometry status not received
    OdomUnhealthy,         // Odometry failsafe unhealthy
    GroundTruthBad,        // Ground truth unhealthy
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroundTruthUnhealthyBehavior {
    Block,   // Block all output
    Xnaut,   // Pass input anyway
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XnautUnhealthyBehavior {
    Passthrough,  // Pass ground truth
    Block,        // Block all output
    Xnaut,        // Pass input anyway
}
```

#### Distance Checking

```rust
pub struct DistanceChecker {
    max_distance: f64,  // meters
}

impl DistanceChecker {
    /// Compute geodesic distance between two NavSatFix positions
    pub fn compute_distance(
        &self,
        fix1: &NavSatFix,
        fix2: &NavSatFix,
    ) -> Result<f64, DistanceError> {
        // Use geographiclib or similar for accurate geodesic distance
        // Handle edge cases: invalid lat/lon, NaN values
        
        if !Self::is_valid_fix(fix1) || !Self::is_valid_fix(fix2) {
            return Err(DistanceError::InvalidFix);
        }
        
        let distance = Self::geodesic_distance(
            fix1.latitude, fix1.longitude,
            fix2.latitude, fix2.longitude,
        );
        
        Ok(distance)
    }
    
    /// Check if distance is within threshold
    pub fn check(
        &self,
        fix1: &NavSatFix,
        fix2: &NavSatFix,
    ) -> Result<(), DistanceError> {
        let distance = self.compute_distance(fix1, fix2)?;
        
        if distance > self.max_distance {
            return Err(DistanceError::Exceeded {
                distance,
                threshold: self.max_distance,
            });
        }
        
        Ok(())
    }
    
    fn is_valid_fix(fix: &NavSatFix) -> bool {
        !fix.latitude.is_nan() && 
        !fix.longitude.is_nan() &&
        fix.latitude.abs() <= 90.0 &&
        fix.longitude.abs() <= 180.0
    }
    
    fn geodesic_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        // Haversine formula or use geographiclib crate
        // Implementation matches Python's geopy.distance.geodesic
        todo!("Implement geodesic distance calculation")
    }
}
```

#### Odometry Integration

```rust
pub struct OdometryIntegration {
    last_status: Option<StatusText>,
    timeout: Duration,
}

impl OdometryIntegration {
    pub fn update_status(&mut self, status: StatusText) {
        self.last_status = Some(status);
    }
    
    pub fn is_healthy(&self, current_time: Time) -> bool {
        if let Some(status) = &self.last_status {
            let age = current_time - Time::from_msg(&status.header.stamp);
            
            if age > self.timeout {
                return false;  // Timeout
            }
            
            // Check status text
            matches!(status.text.as_str(), "ok" | "disabled")
        } else {
            false  // No status received yet
        }
    }
}
```

### 3. Hz Monitor

#### Rate Tracking

```rust
pub struct RateTracker {
    timestamps: VecDeque<Time>,
    window_duration: Duration,
    min_rate: f64,
}

impl RateTracker {
    pub fn add_message(&mut self, timestamp: Time) {
        self.timestamps.push_back(timestamp);
        
        // Remove old timestamps outside window
        let cutoff = timestamp - self.window_duration;
        while let Some(&front) = self.timestamps.front() {
            if front < cutoff {
                self.timestamps.pop_front();
            } else {
                break;
            }
        }
    }
    
    pub fn compute_rate(&self) -> f64 {
        if self.timestamps.len() < 2 {
            return 0.0;
        }
        
        let duration = self.timestamps.back().unwrap() - self.timestamps.front().unwrap();
        let duration_secs = duration.as_secs_f64();
        
        if duration_secs > 0.0 {
            (self.timestamps.len() - 1) as f64 / duration_secs
        } else {
            0.0
        }
    }
    
    pub fn is_rate_acceptable(&self) -> bool {
        self.compute_rate() >= self.min_rate
    }
}
```

### 4. Error Node

#### Error Aggregation

```rust
pub struct ErrorAggregator {
    error_sources: HashMap<String, ErrorSource>,
}

pub struct ErrorSource {
    name: String,
    last_status: Option<StatusText>,
    timeout: Duration,
    error_count: u64,
}

impl ErrorAggregator {
    pub fn add_source(&mut self, name: String, timeout: Duration) {
        self.error_sources.insert(name.clone(), ErrorSource {
            name,
            last_status: None,
            timeout,
            error_count: 0,
        });
    }
    
    pub fn update_status(&mut self, source: &str, status: StatusText) {
        if let Some(src) = self.error_sources.get_mut(source) {
            if status.severity >= StatusText::ERROR {
                src.error_count += 1;
            }
            src.last_status = Some(status);
        }
    }
    
    pub fn get_overall_status(&self, current_time: Time) -> OverallStatus {
        let mut max_severity = Severity::Info;
        let mut error_messages = Vec::new();
        
        for (name, source) in &self.error_sources {
            if let Some(status) = &source.last_status {
                let age = current_time - Time::from_msg(&status.header.stamp);
                
                if age > source.timeout {
                    max_severity = max_severity.max(Severity::Warning);
                    error_messages.push(format!("{}: timeout", name));
                } else if status.severity >= StatusText::ERROR {
                    max_severity = max_severity.max(Severity::Error);
                    error_messages.push(format!("{}: {}", name, status.text));
                }
            } else {
                max_severity = max_severity.max(Severity::Warning);
                error_messages.push(format!("{}: no status", name));
            }
        }
        
        OverallStatus {
            severity: max_severity,
            messages: error_messages,
        }
    }
}
```

## Data Models

### Configuration Structures

```rust
#[derive(Debug, Clone, Deserialize)]
pub struct OdometryFailsafeConfig {
    pub inputs: OdometryInputConfig,
    pub outputs: OdometryOutputConfig,
    pub quality_buffer_size: usize,
    pub quality_threshold: f64,
    pub twist_x_max_error: f64,
    pub twist_y_max_error: f64,
    pub max_delay: f64,  // seconds
    pub monitor_rate: f64,  // Hz
    pub disable: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NavSatFixFailsafeConfig {
    pub inputs: NavSatFixInputConfig,
    pub outputs: NavSatFixOutputConfig,
    pub max_distance: f64,  // meters
    pub max_delay: f64,  // seconds
    pub ground_truth_timeout: f64,  // seconds
    pub fix_timeout: f64,  // seconds
    pub odom_status_timeout: f64,  // seconds
    pub monitor_frequency: f64,  // Hz
    pub ground_truth_unhealthy_behaviour: String,
    pub xnaut_unhealthy_behaviour: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct HzMonitorConfig {
    pub topic: String,
    pub message_type: String,
    pub window_duration: f64,  // seconds
    pub min_rate: f64,  // Hz
    pub monitor_rate: f64,  // Hz
}
```

## Error Handling

### Error Types

```rust
#[derive(Debug, Error)]
pub enum FailsafeError {
    #[error("Quality check failed: {0}")]
    QualityCheckFailed(String),
    
    #[error("Twist limit exceeded: {0}")]
    TwistLimitExceeded(String),
    
    #[error("Delay threshold exceeded: {0}")]
    DelayExceeded(String),
    
    #[error("Distance threshold exceeded: {0}")]
    DistanceExceeded(String),
    
    #[error("Ground truth unhealthy: {0}")]
    GroundTruthUnhealthy(String),
    
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
}
```

### Error Handling Strategy

1. **Validation Errors**: Log warning, use safe default behavior
2. **Check Failures**: Log error, switch to failsafe source
3. **Configuration Errors**: Fail at startup with clear message
4. **ROS2 Errors**: Log error, attempt recovery, escalate if repeated

## Testing Strategy

### Unit Tests

1. **Quality Checker Tests**
   - Buffer management
   - Statistical calculations
   - Threshold checking

2. **Twist Limit Tests**
   - Boundary conditions
   - Each axis independently
   - Combined violations

3. **Delay Checker Tests**
   - Within threshold
   - Exceeded threshold
   - Future timestamps

4. **Distance Checker Tests**
   - Geodesic distance calculation
   - Invalid coordinates
   - Boundary conditions

5. **State Machine Tests**
   - All state transitions
   - Status determination logic
   - Source selection logic

### Integration Tests

1. **Odometry Failsafe Integration**
   - Input healthy → output input
   - Input unhealthy → output ground truth
   - Ground truth unhealthy → block output
   - All state transitions

2. **NavSatFix Failsafe Integration**
   - Distance check scenarios
   - Delay check scenarios
   - Odometry integration scenarios
   - Configurable behavior modes

3. **Python Compatibility Tests**
   - Load Python test vectors
   - Process through Rust implementation
   - Compare outputs (exact match)

### Fault Injection Tests

```rust
#[test]
fn test_sensor_loss_recovery() {
    // Simulate sensor loss and recovery
    // Verify correct failsafe activation and deactivation
}

#[test]
fn test_stale_data_handling() {
    // Simulate increasing message delays
    // Verify failsafe activates at correct threshold
}

#[test]
fn test_invalid_data_handling() {
    // Inject NaN, inf, out-of-range values
    // Verify safe handling without panics
}

#[test]
fn test_rapid_state_transitions() {
    // Rapidly switch between healthy and unhealthy
    // Verify no race conditions or inconsistent state
}
```

### Safety Validation Tests

```rust
#[test]
fn test_no_false_negatives() {
    // Test all failure scenarios
    // Verify failsafe always activates when it should
}

#[test]
fn test_minimal_false_positives() {
    // Test with noisy but valid data
    // Verify failsafe doesn't activate unnecessarily
}

#[test]
fn test_side_by_side_validation() {
    // Run Python and Rust implementations in parallel
    // Verify identical behavior over extended period
}
```

## Performance Considerations

### Latency Optimization

1. **Pre-allocated Buffers**: All buffers allocated at initialization
2. **Lock-free Where Possible**: Use atomic operations for counters
3. **Minimal Branching**: Optimize hot paths
4. **Inline Small Functions**: Use `#[inline]` for checkers

### Memory Management

1. **Fixed-size Buffers**: VecDeque with max capacity
2. **No Runtime Allocations**: All allocations at startup
3. **Stack-allocated Data**: Use stack for temporary data

## Backward Compatibility

### Topic Names

Must match Python implementation exactly:

```yaml
# Odometry Failsafe
inputs:
  odometry: "/input/odometry"
  ground_truth: "/ground_truth/odometry"
outputs:
  odometry: "/failsafe/odometry"
  status: "/failsafe/odometry/status"

# NavSatFix Failsafe
inputs:
  fix: "/input/fix"
  ground_truth: "/ground_truth/fix"
  odom_failsafe_status: "/failsafe/odometry/status"
outputs:
  fix: "/failsafe/fix"
  status: "/failsafe/fix/status"
```

### Parameter Names

All parameter names must match Python exactly.

### Behavior Compatibility

All state transitions and source selection logic must match Python exactly.

## Migration Strategy

### Phase 1: Side-by-Side Validation (4 weeks)

1. Deploy Rust nodes alongside Python nodes
2. Both process same inputs
3. Compare outputs continuously
4. Collect 100+ hours of validation data
5. Fix any discrepancies

### Phase 2: Controlled Rollout (2 weeks)

1. Use feature flag to switch implementations
2. Start with test vehicles in controlled environment
3. Monitor for any issues
4. Gradually expand to more vehicles

### Phase 3: Full Migration (1 week)

1. Switch all vehicles to Rust implementation
2. Monitor closely for first week
3. Remove Python implementation
4. Update documentation

### Rollback Plan

1. Keep Python implementation available
2. Feature flag allows instant rollback
3. Document rollback procedure
4. Test rollback procedure before migration

## Dependencies

```toml
[dependencies]
# ROS2
rclrs = "0.4"

# Math
nalgebra = "0.32"

# Geodesic distance
geographiclib-rs = "0.2"  # or implement Haversine

# Collections
# (use std::collections)

# Error handling
thiserror = "1.0"
anyhow = "1.0"

# Serialization
serde = { version = "1.0", features = ["derive"] }
serde_yaml = "0.9"

# Logging
tracing = "0.1"

# Time
chrono = "0.4"  # if needed for time calculations
```

## Conclusion

This design provides a robust, safe, and performant Rust implementation of the failsafes system. The extensive testing strategy, including fault injection and side-by-side validation, ensures that this safety-critical system will operate correctly in all scenarios. The gradual migration strategy with comprehensive monitoring provides a safe path to production deployment.
