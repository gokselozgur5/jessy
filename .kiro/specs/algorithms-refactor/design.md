# Design Document - GPS Odometry Algorithm Refactor

## Overview

This document describes the design for refactoring the GPS Odometry algorithm from Python to Rust. The system transforms GPS velocity from the global ENU frame to the robot's body frame using IMU orientation data. The Rust implementation will follow the established architecture patterns from the drone driver refactor, including trait-based HAL, centralized error management, and non-blocking orchestration.

### Design Goals

1. **Performance**: Achieve < 1ms latency with deterministic execution
2. **Numerical Stability**: Handle edge cases (NaN, zero quaternions) robustly
3. **Backward Compatibility**: 100% compatible with Python implementation
4. **Maintainability**: Clear separation of concerns, testable components
5. **Safety**: Memory-safe, no runtime allocations, comprehensive error handling

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GPS Odometry System                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐         ┌──────────────┐                  │
│  │ GPS Velocity │         │  IMU Data    │                  │
│  │  (ENU Frame) │         │ (Quaternion) │                  │
│  └──────┬───────┘         └──────┬───────┘                  │
│         │                        │                           │
│         └────────┬───────────────┘                           │
│                  │                                            │
│         ┌────────▼────────┐                                  │
│         │  Message Sync   │                                  │
│         │   & Validation  │                                  │
│         └────────┬────────┘                                  │
│                  │                                            │
│         ┌────────▼────────┐                                  │
│         │   Quaternion    │                                  │
│         │  Transformation │                                  │
│         └────────┬────────┘                                  │
│                  │                                            │
│         ┌────────▼────────┐                                  │
│         │ Noise Generator │ (Optional)                       │
│         │   (Gaussian)    │                                  │
│         └────────┬────────┘                                  │
│                  │                                            │
│         ┌────────▼────────┐                                  │
│         │    Odometry     │                                  │
│         │    Publisher    │                                  │
│         └─────────────────┘                                  │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Component Architecture

```
prod-xnaut-core-rewrite/
├── rust/
│   └── algorithms/                 # Algorithms workspace
│       ├── Cargo.toml              # Workspace manifest
│       ├── gps_odometry/           # GPS Odometry algorithm
│       │   ├── src/
│       │   │   ├── lib.rs              # Public API
│       │   │   ├── node.rs             # ROS2 node implementation
│       │   │   ├── transform.rs        # Quaternion transformation logic
│       │   │   ├── noise.rs            # Gaussian noise generator
│       │   │   ├── validation.rs       # Input validation & edge cases
│       │   │   ├── config.rs           # Configuration structures
│       │   │   └── types.rs            # Common types & constants
│       │   ├── tests/
│       │   │   ├── transform_tests.rs  # Unit tests for transformations
│       │   │   ├── validation_tests.rs # Edge case tests
│       │   │   ├── integration_tests.rs # ROS2 integration tests
│       │   │   └── compatibility_tests.rs # Python compatibility tests
│       │   ├── benches/
│       │   │   └── odometry_bench.rs   # Performance benchmarks
│       │   ├── Cargo.toml
│       │   └── IMPLEMENTATION.md
│       └── (future: gimp/, map_reader/, waypoint_publisher/)
├── config/
│   └── gps_odometry.yaml           # Configuration file
├── launch/
│   └── gps_odometry.launch.py      # Launch file
└── README.md
```

**Note**: The `rust/algorithms/` workspace structure mirrors the Python `algorithms/` package structure from the original repository, allowing for multiple algorithm implementations (GPS Odometry, GIMP, Map Reader, Waypoint Publisher) to coexist and share common dependencies.

## Components and Interfaces

### 1. Quaternion Transformation Module (`transform.rs`)

**Purpose**: Core mathematical operations for coordinate frame transformations

**Key Functions**:

```rust
/// Transform velocity from ENU frame to body frame using quaternion rotation
pub fn transform_velocity_to_body_frame(
    velocity_enu: &Vector3<f64>,
    orientation_quat: &UnitQuaternion<f64>,
) -> Result<Vector3<f64>, TransformError>;

/// Normalize quaternion to unit magnitude
pub fn normalize_quaternion(
    quat: &Quaternion<f64>
) -> Result<UnitQuaternion<f64>, TransformError>;

/// Validate quaternion (non-zero magnitude, no NaN)
pub fn validate_quaternion(
    quat: &Quaternion<f64>
) -> Result<(), TransformError>;

/// Validate velocity vector (handle NaN in vertical component)
pub fn validate_velocity(
    velocity: &Vector3<f64>
) -> Result<Vector3<f64>, TransformError>;
```

**Design Decisions**:
- Use `nalgebra` crate for quaternion operations (industry-standard, well-tested)
- Return `Result` types for all operations to handle edge cases explicitly
- Use `UnitQuaternion` type to enforce normalization at compile time where possible
- Handle NaN in vertical velocity by mapping to zero (matches Python behavior)
- Return zero velocity for invalid horizontal velocity or quaternion

**Numerical Stability**:
- Check quaternion magnitude > epsilon (1e-10) before normalization
- Use `f64` for all calculations to match Python precision
- Explicit NaN checks before mathematical operations
- No panics - all errors returned as `Result`

### 2. Noise Generator Module (`noise.rs`)

**Purpose**: Generate configurable Gaussian noise for simulation and testing

**Key Structures**:

```rust
#[derive(Debug, Clone)]
pub struct NoiseConfig {
    pub mean_xy: f64,
    pub mean_z: f64,
    pub std_xy: f64,
    pub std_z: f64,
    pub scale_xy: f64,
    pub scale_z: f64,
}

pub struct NoiseGenerator {
    config: NoiseConfig,
    rng: ThreadRng,
}

impl NoiseGenerator {
    pub fn new(config: NoiseConfig) -> Self;
    
    /// Generate noise for XY components
    pub fn generate_xy_noise(&mut self) -> f64;
    
    /// Generate noise for Z component
    pub fn generate_z_noise(&mut self) -> f64;
    
    /// Apply noise to velocity vector
    pub fn apply_noise(
        &mut self,
        velocity: &mut Vector3<f64>,
        apply_xy: bool,
        apply_z: bool,
    );
}
```

**Design Decisions**:
- Use `rand` and `rand_distr` crates for Gaussian distribution
- Thread-local RNG for performance (no locking)
- Separate XY and Z noise generation (different parameters)
- Optional application via boolean flags
- Regenerate noise on each call (matches Python behavior)

### 3. Validation Module (`validation.rs`)

**Purpose**: Input validation and edge case handling

**Key Functions**:

```rust
/// Validate and sanitize GPS velocity message
pub fn validate_gps_velocity(
    msg: &TwistStamped
) -> Result<Vector3<f64>, ValidationError>;

/// Validate and sanitize IMU orientation
pub fn validate_imu_orientation(
    msg: &Imu
) -> Result<UnitQuaternion<f64>, ValidationError>;

/// Check if velocity contains NaN values
pub fn has_nan_velocity(velocity: &Vector3<f64>) -> bool;

/// Check if quaternion is valid (non-zero, no NaN)
pub fn is_valid_quaternion(quat: &Quaternion<f64>) -> bool;
```

**Edge Cases Handled**:
1. **NaN in vertical velocity**: Map to zero (common in GPS data)
2. **NaN in horizontal velocity**: Return zero velocity for all axes
3. **Zero quaternion magnitude**: Return zero velocity
4. **Negative quaternion magnitude**: Return zero velocity (shouldn't happen, but defensive)
5. **NaN in quaternion**: Return zero velocity

**Design Decisions**:
- Explicit validation before transformation
- Clear error types for each failure mode
- Logging at appropriate levels (WARN for NaN, ERROR for repeated failures)
- Match Python behavior exactly for compatibility

### 4. Configuration Module (`config.rs`)

**Purpose**: Configuration structures and parameter loading

**Key Structures**:

```rust
#[derive(Debug, Clone, Deserialize)]
pub struct GpsOdometryConfig {
    pub inputs: InputConfig,
    pub outputs: OutputConfig,
    pub noise: NoiseConfig,
    pub covariance: CovarianceConfig,
    pub apply_noise_xy: bool,
    pub apply_noise_z: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct InputConfig {
    pub gps_velocity_topic: String,
    pub imu_topic: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct OutputConfig {
    pub odometry_topic: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CovarianceConfig {
    pub linear_twist_x: f64,
    pub linear_twist_y: f64,
    pub linear_twist_z: f64,
}
```

**Design Decisions**:
- Use `serde` for YAML deserialization
- Match Python parameter names exactly
- Validate configuration at load time
- Support runtime parameter updates for noise settings
- Default values match Python implementation

### 5. ROS2 Node Module (`node.rs`)

**Purpose**: ROS2 integration and message handling

**Key Structure**:

```rust
pub struct GpsOdometryNode {
    node: Arc<Node>,
    config: GpsOdometryConfig,
    noise_generator: Option<NoiseGenerator>,
    
    // Subscribers
    gps_velocity_sub: Subscription<TwistStamped>,
    imu_sub: Subscription<Imu>,
    
    // Publisher
    odometry_pub: Publisher<Odometry>,
    
    // Message synchronization
    sync: MessageSynchronizer<TwistStamped, Imu>,
    
    // Diagnostics
    error_manager: Arc<ErrorManager>,
    performance_monitor: Arc<PerformanceMonitor>,
}

impl GpsOdometryNode {
    pub fn new(config: GpsOdometryConfig) -> Result<Self, NodeError>;
    
    fn synchronized_callback(
        &mut self,
        gps_velocity: TwistStamped,
        imu: Imu,
    ) -> Result<(), ProcessingError>;
    
    fn process_messages(
        &mut self,
        gps_velocity: &TwistStamped,
        imu: &Imu,
    ) -> Result<Odometry, ProcessingError>;
    
    fn build_odometry_message(
        &self,
        header: &Header,
        velocity_body: &Vector3<f64>,
    ) -> Odometry;
}
```

**Design Decisions**:
- Use message synchronization (similar to Python's `register_synchronized`)
- Integrate with existing error management system
- Integrate with performance monitoring
- Non-blocking callback execution
- Publish diagnostics on processing errors

### 6. Message Synchronization

**Approach**: Approximate time synchronization

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
    /// Add message to queue 1 and attempt synchronization
    pub fn add_message1(&mut self, msg: T1) -> Option<(T1, T2)>;
    
    /// Add message to queue 2 and attempt synchronization
    pub fn add_message2(&mut self, msg: T2) -> Option<(T1, T2)>;
    
    /// Find best matching pair within time tolerance
    fn find_synchronized_pair(&mut self) -> Option<(T1, T2)>;
}
```

**Design Decisions**:
- Queue-based synchronization (similar to ROS message_filters)
- Configurable time tolerance (default: 10ms)
- Drop old messages to prevent unbounded growth
- Return synchronized pairs when timestamps align within tolerance

## Data Models

### Input Messages

```rust
// From ROS2 geometry_msgs
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}

pub struct Twist {
    pub linear: Vector3,   // Velocity in m/s
    pub angular: Vector3,  // Not used in this node
}

// From ROS2 sensor_msgs
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    // ... other fields not used
}
```

### Output Message

```rust
// From ROS2 nav_msgs
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,     // Not populated
    pub twist: TwistWithCovariance,   // Velocity + covariance
}

pub struct TwistWithCovariance {
    pub twist: Twist,
    pub covariance: [f64; 36],  // 6x6 matrix (only diagonal elements used)
}
```

### Internal Data Types

```rust
// Using nalgebra types
pub type Vector3 = nalgebra::Vector3<f64>;
pub type Quaternion = nalgebra::Quaternion<f64>;
pub type UnitQuaternion = nalgebra::UnitQuaternion<f64>;

// Error types
#[derive(Debug, Error)]
pub enum TransformError {
    #[error("Invalid quaternion: {0}")]
    InvalidQuaternion(String),
    
    #[error("Invalid velocity: {0}")]
    InvalidVelocity(String),
    
    #[error("Numerical instability: {0}")]
    NumericalInstability(String),
}

#[derive(Debug, Error)]
pub enum ValidationError {
    #[error("NaN in velocity: {0}")]
    NaNVelocity(String),
    
    #[error("Invalid quaternion magnitude: {0}")]
    InvalidQuaternionMagnitude(f64),
}
```

## Error Handling

### Error Strategy

1. **Validation Errors**: Log warning, return zero velocity, continue processing
2. **Transformation Errors**: Log error, publish diagnostic, return zero velocity
3. **Configuration Errors**: Fail at startup with clear error message
4. **ROS2 Errors**: Log error, attempt recovery, escalate if repeated

### Error Integration

```rust
// Integrate with centralized error management
impl GpsOdometryNode {
    fn handle_processing_error(&self, error: ProcessingError) {
        let error_code = match error {
            ProcessingError::InvalidQuaternion(_) => ErrorCode::InvalidSensorData,
            ProcessingError::NaNVelocity(_) => ErrorCode::DataQualityIssue,
            _ => ErrorCode::ProcessingFailure,
        };
        
        self.error_manager.report_error(
            error_code,
            Severity::Warning,
            &format!("GPS odometry processing error: {}", error),
        );
    }
}
```

### Diagnostic Publishing

```rust
// Publish diagnostics for monitoring
impl GpsOdometryNode {
    fn publish_diagnostics(&self) {
        let status = DiagnosticStatus {
            level: self.determine_diagnostic_level(),
            name: "gps_odometry".to_string(),
            message: self.get_status_message(),
            hardware_id: "".to_string(),
            values: vec![
                KeyValue { key: "processing_rate".to_string(), value: format!("{:.1}", self.get_rate()) },
                KeyValue { key: "error_count".to_string(), value: format!("{}", self.error_count) },
                KeyValue { key: "nan_count".to_string(), value: format!("{}", self.nan_count) },
            ],
        };
        
        self.diagnostic_pub.publish(&status);
    }
}
```

## Testing Strategy

### Unit Tests

1. **Quaternion Transformation Tests**
   - Identity quaternion (no rotation)
   - 90-degree rotations (X, Y, Z axes)
   - Arbitrary rotations
   - Numerical precision validation

2. **Edge Case Tests**
   - NaN in vertical velocity
   - NaN in horizontal velocity
   - Zero quaternion magnitude
   - Very small quaternion magnitude
   - NaN in quaternion components

3. **Noise Generation Tests**
   - Statistical properties (mean, std dev)
   - Reproducibility with seed
   - XY vs Z noise separation
   - Scale factor application

4. **Validation Tests**
   - Valid inputs pass through
   - Invalid inputs rejected with correct error
   - Edge cases handled correctly

### Integration Tests

1. **ROS2 Integration**
   - Message subscription and publishing
   - Message synchronization
   - Topic name compatibility
   - Message type compatibility

2. **Python Compatibility Tests**
   - Load Python test vectors
   - Process through Rust implementation
   - Compare outputs (tolerance: 1e-6)
   - Verify 100% compatibility

3. **End-to-End Tests**
   - Launch node with configuration
   - Publish test messages
   - Verify output messages
   - Check diagnostic messages

### Performance Benchmarks

1. **Latency Benchmarks**
   - Measure callback execution time
   - Target: < 1ms per message pair
   - Measure variance (target: < 100μs)

2. **Throughput Benchmarks**
   - Maximum message rate
   - CPU usage at various rates
   - Memory usage (should be constant)

3. **Comparison Benchmarks**
   - Rust vs Python latency
   - Rust vs Python throughput
   - Target: 2x improvement minimum

### Validation Methodology

```rust
#[test]
fn test_python_compatibility() {
    // Load Python test vectors from JSON
    let test_vectors: Vec<TestVector> = load_test_vectors("python_baseline.json");
    
    for vector in test_vectors {
        // Process through Rust implementation
        let rust_output = process_gps_odometry(
            &vector.gps_velocity,
            &vector.imu,
            &vector.config,
        );
        
        // Compare with Python output
        assert_vector_near(
            &rust_output.velocity,
            &vector.expected_velocity,
            1e-6,  // Tolerance
        );
    }
}
```

## Performance Considerations

### Memory Management

1. **Zero Runtime Allocations**
   - Pre-allocate all buffers at initialization
   - Use stack-allocated data structures in hot path
   - Reuse message buffers where possible

2. **Cache Efficiency**
   - Keep hot data structures small and contiguous
   - Avoid pointer chasing in critical path
   - Use `#[inline]` for small frequently-called functions

### Optimization Strategies

1. **Compiler Optimizations**
   - Release builds with LTO (Link-Time Optimization)
   - Target-specific optimizations (e.g., AVX2 for x86_64)
   - Profile-guided optimization for hot paths

2. **Algorithmic Optimizations**
   - Use `nalgebra`'s optimized quaternion operations
   - Avoid unnecessary copies (use references)
   - Minimize branching in hot path

3. **Parallelization**
   - Not needed for single message processing
   - Consider for batch processing if needed in future

## Backward Compatibility

### Topic Names

```yaml
# Must match Python implementation exactly
inputs:
  gps_velocity_topic: "/mavros/global_position/raw/gps_vel"
  imu_topic: "/mavros/imu/data"

outputs:
  odometry_topic: "/gps/odometry"
```

### Message Types

- Input: `geometry_msgs/TwistStamped`, `sensor_msgs/Imu`
- Output: `nav_msgs/Odometry`
- All standard ROS2 messages (no custom types)

### Parameter Names

```yaml
# Must match Python parameter names
noise:
  mean_xy: 0.0
  mean_z: 0.0
  std_xy: 0.2
  std_z: 0.2
  scale_xy: 1.0
  scale_z: 1.0

covariance:
  linear_twist_x: 0.0
  linear_twist_y: 0.0
  linear_twist_z: 0.0

apply_noise_xy: false
apply_noise_z: false
```

### Numerical Compatibility

- Use `f64` for all calculations (matches Python `float`)
- Same quaternion operation sequence as Python
- Same NaN handling logic
- Validate outputs match within 1e-6 tolerance

### Launch File Compatibility

```python
# Rust launch file should be drop-in replacement
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='algorithms',
            executable='gps_odometry_rust',  # New executable name
            name='gps_odometry_node',        # Same node name
            parameters=[config_file],
            output='screen',
        ),
    ])
```

## Migration Strategy

### Phase 1: Side-by-Side Validation (2 weeks)

1. Deploy Rust node alongside Python node
2. Both subscribe to same inputs
3. Compare outputs in real-time
4. Collect validation data
5. Fix any discrepancies

### Phase 2: Gradual Rollout (1 week)

1. Use feature flag to switch between implementations
2. Start with test vehicles
3. Monitor for issues
4. Expand to more vehicles

### Phase 3: Full Migration (1 week)

1. Switch all vehicles to Rust implementation
2. Remove Python implementation
3. Update documentation
4. Archive Python code for reference

## Dependencies

### Rust Crates

```toml
[dependencies]
# ROS2 integration
rclrs = "0.4"
rosidl_runtime_rs = "0.4"

# Math libraries
nalgebra = "0.32"

# Random number generation
rand = "0.8"
rand_distr = "0.4"

# Error handling
thiserror = "1.0"
anyhow = "1.0"

# Serialization
serde = { version = "1.0", features = ["derive"] }
serde_yaml = "0.9"

# Logging
tracing = "0.1"
tracing-subscriber = "0.3"

# Performance monitoring
metrics = "0.21"

[dev-dependencies]
# Testing
approx = "0.5"  # For floating-point comparisons
criterion = "0.5"  # For benchmarking
serde_json = "1.0"  # For loading test vectors
```

### System Dependencies

- ROS2 Humble or later
- Rust 1.70 or later
- MAVROS (for GPS and IMU data)

## Open Questions and Future Enhancements

### Open Questions

1. **Message Synchronization**: Should we use exact time sync or approximate? (Decision: approximate with 10ms tolerance)
2. **Noise Generator**: Should we support other distributions? (Decision: Gaussian only for now, extensible later)
3. **Covariance**: Should we compute covariance dynamically? (Decision: No, use configured values like Python)

### Future Enhancements

1. **Extended Kalman Filter**: Integrate with EKF for state estimation
2. **Multi-Sensor Fusion**: Support multiple GPS sources
3. **Adaptive Noise**: Adjust noise based on signal quality
4. **SIMD Optimization**: Use SIMD instructions for vector operations
5. **GPU Acceleration**: Offload transformations to GPU for high-rate processing

## Conclusion

This design provides a robust, performant, and maintainable Rust implementation of the GPS Odometry algorithm. By following established patterns from the drone driver refactor and maintaining strict backward compatibility, we ensure a smooth migration path while achieving significant performance improvements. The comprehensive testing strategy and validation methodology give us confidence in the correctness and reliability of the implementation.
