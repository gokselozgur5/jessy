# Design Document - Utils Navigation Error Tracker Refactor

## Overview

This document describes the design for refactoring the Navigation Error Tracker from Python to Rust. This is a simple, focused component that calculates position error between two GPS sources. The design emphasizes simplicity, performance, and backward compatibility.

### Design Goals

1. **Simplicity**: Minimal, focused implementation
2. **Performance**: < 200μs latency with deterministic execution
3. **Backward Compatibility**: 100% compatible with Python implementation
4. **Maintainability**: Clear, well-documented code

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────┐
│           Navigation Error Tracker System                │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐         ┌──────────────┐              │
│  │  Fix Input   │         │ Ground Truth │              │
│  │  (EKF/GNSS)  │         │    (GNSS)    │              │
│  └──────┬───────┘         └──────┬───────┘              │
│         │                        │                       │
│         └────────┬───────────────┘                       │
│                  │                                        │
│         ┌────────▼────────┐                              │
│         │  Message Sync   │                              │
│         └────────┬────────┘                              │
│                  │                                        │
│         ┌────────▼────────┐                              │
│         │   Validation    │                              │
│         └────────┬────────┘                              │
│                  │                                        │
│         ┌────────▼────────┐                              │
│         │    Geodesic     │                              │
│         │    Distance     │                              │
│         └────────┬────────┘                              │
│                  │                                        │
│         ┌────────▼────────┐                              │
│         │  Error Output   │                              │
│         │  (PointStamped) │                              │
│         └─────────────────┘                              │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### Component Architecture

```
utils/
└── rust/
    └── navigation_error_tracker/
        ├── src/
        │   ├── lib.rs              # Public API
        │   ├── node.rs             # ROS2 node implementation
        │   ├── distance.rs         # Geodesic distance calculation
        │   ├── validation.rs       # Input validation
        │   ├── config.rs           # Configuration structures
        │   └── types.rs            # Common types
        ├── tests/
        │   ├── distance_tests.rs   # Unit tests for distance
        │   ├── validation_tests.rs # Edge case tests
        │   ├── integration_tests.rs # ROS2 integration tests
        │   └── compatibility_tests.rs # Python compatibility tests
        ├── benches/
        │   └── tracker_bench.rs    # Performance benchmarks
        └── Cargo.toml
```

## Components and Interfaces

### 1. Distance Calculation Module (`distance.rs`)

**Purpose**: Calculate geodesic distance between two GPS positions

**Key Functions**:

```rust
/// Calculate geodesic distance between two NavSatFix positions
pub fn geodesic_distance(
    lat1: f64,
    lon1: f64,
    lat2: f64,
    lon2: f64,
) -> Result<f64, DistanceError>;

/// Calculate height difference
pub fn height_difference(alt1: f64, alt2: f64) -> f64;

/// Calculate navigation error (horizontal distance + height difference)
pub fn calculate_navigation_error(
    fix: &NavSatFix,
    ground_truth: &NavSatFix,
) -> Result<NavigationError, DistanceError>;
```

**Geodesic Distance Implementation**:

```rust
/// Haversine formula for geodesic distance
/// Accurate for most use cases, faster than Vincenty
pub fn geodesic_distance(
    lat1: f64,
    lon1: f64,
    lat2: f64,
    lon2: f64,
) -> Result<f64, DistanceError> {
    const EARTH_RADIUS: f64 = 6371000.0; // meters
    
    // Convert to radians
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let delta_lat = (lat2 - lat1).to_radians();
    let delta_lon = (lon2 - lon1).to_radians();
    
    // Haversine formula
    let a = (delta_lat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
    
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    
    let distance = EARTH_RADIUS * c;
    
    Ok(distance)
}
```

**Design Decisions**:
- Use Haversine formula (simpler, faster than Vincenty)
- Accuracy: ~0.5% error for distances < 1000km (sufficient for drones)
- Use `f64` for all calculations (matches Python precision)
- Return `Result` for error handling

### 2. Validation Module (`validation.rs`)

**Purpose**: Validate GPS coordinates and handle edge cases

**Key Functions**:

```rust
/// Validate NavSatFix message
pub fn validate_navsatfix(fix: &NavSatFix) -> Result<(), ValidationError>;

/// Check if coordinates are valid
pub fn is_valid_coordinate(lat: f64, lon: f64) -> bool {
    !lat.is_nan() && 
    !lon.is_nan() &&
    lat.abs() <= 90.0 &&
    lon.abs() <= 180.0
}

/// Check if altitude is valid
pub fn is_valid_altitude(alt: f64) -> bool {
    !alt.is_nan() && alt.is_finite()
}
```

**Edge Cases Handled**:
1. **NaN coordinates**: Skip calculation, log warning
2. **Out of range lat/lon**: Skip calculation, log error
3. **NaN altitude**: Use 0.0 for height difference
4. **Infinite values**: Skip calculation, log error

### 3. Configuration Module (`config.rs`)

**Purpose**: Configuration structures and parameter loading

**Key Structures**:

```rust
#[derive(Debug, Clone, Deserialize)]
pub struct NavigationErrorTrackerConfig {
    pub inputs: InputConfig,
    pub outputs: OutputConfig,
    pub sync_tolerance_ms: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct InputConfig {
    pub fix_topic: String,
    pub ground_truth_topic: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct OutputConfig {
    pub error_topic: String,
}
```

**Default Values**:

```yaml
inputs:
  fix_topic: "/ekf/output/fix"
  ground_truth_topic: "/mavros/global_position/raw/fix"
outputs:
  error_topic: "/navigation_error_node/error"
sync_tolerance_ms: 10.0
```

### 4. ROS2 Node Module (`node.rs`)

**Purpose**: ROS2 integration and message handling

**Key Structure**:

```rust
pub struct NavigationErrorTrackerNode {
    node: Arc<Node>,
    config: NavigationErrorTrackerConfig,
    
    // Subscribers
    fix_sub: Subscription<NavSatFix>,
    ground_truth_sub: Subscription<NavSatFix>,
    
    // Publisher
    error_pub: Publisher<PointStamped>,
    
    // Message synchronization
    sync: MessageSynchronizer<NavSatFix, NavSatFix>,
    
    // Diagnostics
    error_manager: Arc<ErrorManager>,
    performance_monitor: Arc<PerformanceMonitor>,
    
    // Statistics
    valid_count: AtomicU64,
    invalid_count: AtomicU64,
}

impl NavigationErrorTrackerNode {
    pub fn new(config: NavigationErrorTrackerConfig) -> Result<Self, NodeError>;
    
    fn synchronized_callback(
        &mut self,
        fix: NavSatFix,
        ground_truth: NavSatFix,
    ) -> Result<(), ProcessingError>;
    
    fn process_messages(
        &mut self,
        fix: &NavSatFix,
        ground_truth: &NavSatFix,
    ) -> Result<PointStamped, ProcessingError>;
    
    fn build_error_message(
        &self,
        header: &Header,
        horizontal_error: f64,
        height_error: f64,
    ) -> PointStamped;
}
```

**Message Processing Flow**:

```rust
fn process_messages(
    &mut self,
    fix: &NavSatFix,
    ground_truth: &NavSatFix,
) -> Result<PointStamped, ProcessingError> {
    // 1. Validate inputs
    validate_navsatfix(fix)?;
    validate_navsatfix(ground_truth)?;
    
    // 2. Calculate navigation error
    let error = calculate_navigation_error(fix, ground_truth)?;
    
    // 3. Build output message
    let msg = self.build_error_message(
        &fix.header,
        error.horizontal_distance,
        error.height_difference,
    );
    
    // 4. Update statistics
    self.valid_count.fetch_add(1, Ordering::Relaxed);
    
    Ok(msg)
}
```

### 5. Message Synchronization

**Approach**: Simple approximate time synchronization

```rust
pub struct MessageSynchronizer<T1, T2> {
    queue1: VecDeque<T1>,
    queue2: VecDeque<T2>,
    max_queue_size: usize,
    time_tolerance: Duration,
}

impl<T1, T2> MessageSynchronizer<T1, T2> 
where
    T1: HasHeader,
    T2: HasHeader,
{
    pub fn add_message1(&mut self, msg: T1) -> Option<(T1, T2)> {
        self.queue1.push_back(msg);
        self.trim_queue1();
        self.find_synchronized_pair()
    }
    
    pub fn add_message2(&mut self, msg: T2) -> Option<(T1, T2)> {
        self.queue2.push_back(msg);
        self.trim_queue2();
        self.find_synchronized_pair()
    }
    
    fn find_synchronized_pair(&mut self) -> Option<(T1, T2)> {
        // Find closest timestamp match within tolerance
        // Remove matched messages from queues
        // Return synchronized pair
    }
}
```

## Data Models

### Input Messages

```rust
// From ROS2 sensor_msgs
pub struct NavSatFix {
    pub header: Header,
    pub status: NavSatStatus,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}
```

### Output Message

```rust
// From ROS2 geometry_msgs
pub struct PointStamped {
    pub header: Header,
    pub point: Point,
}

pub struct Point {
    pub x: f64,  // Horizontal distance error (meters)
    pub y: f64,  // Always 0.0
    pub z: f64,  // Height difference (meters)
}
```

### Internal Data Types

```rust
pub struct NavigationError {
    pub horizontal_distance: f64,  // meters
    pub height_difference: f64,    // meters
}

#[derive(Debug, Error)]
pub enum DistanceError {
    #[error("Invalid coordinates: lat={0}, lon={1}")]
    InvalidCoordinates(f64, f64),
    
    #[error("NaN in coordinates")]
    NaNCoordinates,
}

#[derive(Debug, Error)]
pub enum ValidationError {
    #[error("Invalid latitude: {0}")]
    InvalidLatitude(f64),
    
    #[error("Invalid longitude: {0}")]
    InvalidLongitude(f64),
    
    #[error("NaN in NavSatFix")]
    NaNValue,
}
```

## Error Handling

### Error Strategy

1. **Validation Errors**: Log warning, skip calculation, continue processing
2. **Distance Calculation Errors**: Log error, publish diagnostic, skip message
3. **Configuration Errors**: Fail at startup with clear error message
4. **ROS2 Errors**: Log error, attempt recovery

### Error Integration

```rust
impl NavigationErrorTrackerNode {
    fn handle_processing_error(&self, error: ProcessingError) {
        let error_code = match error {
            ProcessingError::InvalidCoordinates(_) => ErrorCode::InvalidSensorData,
            ProcessingError::NaN(_) => ErrorCode::DataQualityIssue,
            _ => ErrorCode::ProcessingFailure,
        };
        
        self.error_manager.report_error(
            error_code,
            Severity::Warning,
            &format!("Navigation error tracking failed: {}", error),
        );
        
        self.invalid_count.fetch_add(1, Ordering::Relaxed);
    }
}
```

## Testing Strategy

### Unit Tests

1. **Distance Calculation Tests**
   - Known coordinates with expected distances
   - Edge cases (poles, equator, antimeridian)
   - Numerical precision validation

2. **Validation Tests**
   - Valid coordinates pass
   - Invalid coordinates rejected
   - NaN handling
   - Out of range handling

3. **Edge Case Tests**
   - NaN in latitude
   - NaN in longitude
   - NaN in altitude
   - Out of range values
   - Infinite values

### Integration Tests

1. **ROS2 Integration**
   - Message subscription and publishing
   - Message synchronization
   - Topic name compatibility

2. **Python Compatibility Tests**
   - Load Python test vectors
   - Process through Rust implementation
   - Compare outputs (tolerance: 1e-6 meters)

### Performance Benchmarks

1. **Latency Benchmarks**
   - Measure callback execution time
   - Target: < 200μs per message pair
   - Measure variance (target: < 20μs)

2. **Comparison Benchmarks**
   - Rust vs Python latency
   - Target: 2x improvement minimum

## Performance Considerations

### Memory Management

1. **Zero Runtime Allocations**
   - Pre-allocate all buffers at initialization
   - Use stack-allocated data structures
   - Reuse message buffers

2. **Cache Efficiency**
   - Small, contiguous data structures
   - Inline small functions

### Optimization Strategies

1. **Compiler Optimizations**
   - Release builds with LTO
   - Target-specific optimizations

2. **Algorithmic Optimizations**
   - Haversine formula (simpler than Vincenty)
   - Avoid unnecessary copies
   - Minimize branching

## Backward Compatibility

### Topic Names

```yaml
# Must match Python implementation exactly
inputs:
  fix_topic: "/ekf/output/fix"
  ground_truth_topic: "/mavros/global_position/raw/fix"
outputs:
  error_topic: "/navigation_error_node/error"
```

### Message Types

- Input: `sensor_msgs/NavSatFix`
- Output: `geometry_msgs/PointStamped`
- All standard ROS2 messages

### Output Format

```
PointStamped:
  header: (from fix message)
  point:
    x: horizontal_distance (meters)
    y: 0.0
    z: height_difference (meters)
```

### Numerical Compatibility

- Use `f64` for all calculations
- Haversine formula (matches Python's geopy)
- Validate outputs match within 1e-6 meters

## Migration Strategy

### Phase 1: Side-by-Side Validation (1 week)

1. Deploy Rust node alongside Python node
2. Both subscribe to same inputs
3. Compare outputs in real-time
4. Collect validation data

### Phase 2: Gradual Rollout (1 week)

1. Use feature flag to switch implementations
2. Start with test vehicles
3. Monitor for issues

### Phase 3: Full Migration (1 week)

1. Switch all vehicles to Rust implementation
2. Remove Python implementation
3. Update documentation

## Dependencies

```toml
[dependencies]
# ROS2
rclrs = "0.4"

# Math (if needed for advanced geodesic)
# geographiclib-rs = "0.2"  # Optional, for Vincenty

# Error handling
thiserror = "1.0"
anyhow = "1.0"

# Serialization
serde = { version = "1.0", features = ["derive"] }
serde_yaml = "0.9"

# Logging
tracing = "0.1"

[dev-dependencies]
approx = "0.5"  # For floating-point comparisons
criterion = "0.5"  # For benchmarking
```

## Conclusion

This design provides a simple, performant, and maintainable Rust implementation of the Navigation Error Tracker. The focus on simplicity and backward compatibility ensures a smooth migration path while achieving significant performance improvements.
