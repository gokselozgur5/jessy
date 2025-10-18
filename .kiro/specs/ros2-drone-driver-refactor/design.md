# Design Document: ROS2 Drone Driver Refactor

**Requirements:** REQ-1.1 through REQ-12.3  
**Status:** Implementation in Progress  
**Last Updated:** 2025-10-18

## Overview

This document describes the detailed design for refactoring the drone driver system from Python to Rust while maintaining 100% backward compatibility with existing ROS2 interfaces. The design follows a layered architecture with strict separation of concerns, comprehensive error handling, and performance optimization.

### Design Goals

1. **Performance**: <10ms latency for critical paths, 100Hz+ sensor rates
2. **Reliability**: Comprehensive error handling with graceful degradation
3. **Maintainability**: Junior-proof design with strong typing and clear interfaces
4. **Compatibility**: Drop-in replacement for Python implementation
5. **Testability**: TDD approach with >85% code coverage
6. **Traceability**: All code linked to requirements

---

## Architecture Overview

### System Architecture Diagram

```mermaid
graph TB
    subgraph "Hardware Layer"
        IMU[IMU Device<br/>/dev/imu0]
        GPS[GPS Device<br/>/dev/ttyUSB0]
        FC[Flight Controller<br/>/dev/ttyACM0]
        CAM[Camera<br/>/dev/video0]
    end
    
    subgraph "HAL Layer (Rust)"
        ImuHal[ImuHal Trait]
        GpsHal[GpsHal Trait]
        FcHal[FlightControllerHal Trait]
        CamHal[CameraHal Trait]
        
        LinuxImu[LinuxImuHal]
        LinuxGps[LinuxGpsHal]
        LinuxFc[LinuxFcHal]
        LinuxCam[LinuxCameraHal]
        
        MockImu[MockImuHal]
        MockGps[MockGpsHal]
        MockFc[MockFcHal]
        MockCam[MockCameraHal]
    end
    
    subgraph "Driver Layer (Rust)"
        ImuDriver[IMU Driver<br/>DriverNodeBase]
        GpsDriver[GPS Driver<br/>DriverNodeBase]
        FcDriver[FC Driver<br/>DriverNodeBase]
        CamDriver[Camera Driver<br/>DriverNodeBase]
        
        DriverBase[DriverNodeBase<br/>Lifecycle Management]
        DriverBehavior[DriverBehavior Trait<br/>Hardware-specific logic]
    end
    
    subgraph "Cross-Cutting Concerns (Rust)"
        ErrorMgr[Error Manager<br/>Codes & Reporting]
        TraceMgr[Traceability Manager<br/>REQ IDs]
        PerfMon[Performance Monitor<br/>Latency & Throughput]
        QoSMgr[QoS Manager<br/>Profiles]
    end
    
    subgraph "ROS2 Layer"
        ImuTopic[/sensor/imu]
        GpsTopic[/sensor/gps]
        FcTopic[/flight_controller/status]
        CamTopic[/camera/image]
        DiagTopic[/diagnostics]
        HeartbeatTopic[/heartbeat]
    end
    
    subgraph "Orchestration Layer (Python)"
        HealthMon[Health Monitor]
        FaultMgr[Fault Manager]
        LifecycleMgr[Lifecycle Manager]
    end
    
    %% Hardware to HAL
    IMU --> ImuHal
    GPS --> GpsHal
    FC --> FcHal
    CAM --> CamHal
    
    %% HAL implementations
    ImuHal --> LinuxImu
    ImuHal --> MockImu
    GpsHal --> LinuxGps
    GpsHal --> MockGps
    FcHal --> LinuxFc
    FcHal --> MockFc
    CamHal --> LinuxCam
    CamHal --> MockCam
    
    %% HAL to Driver
    LinuxImu --> ImuDriver
    LinuxGps --> GpsDriver
    LinuxFc --> FcDriver
    LinuxCam --> CamDriver
    
    %% Driver architecture
    ImuDriver --> DriverBase
    GpsDriver --> DriverBase
    FcDriver --> DriverBase
    CamDriver --> DriverBase
    DriverBase --> DriverBehavior
    
    %% Cross-cutting concerns
    DriverBase --> ErrorMgr
    DriverBase --> TraceMgr
    DriverBase --> PerfMon
    DriverBase --> QoSMgr
    
    %% Driver to ROS2
    ImuDriver --> ImuTopic
    GpsDriver --> GpsTopic
    FcDriver --> FcTopic
    CamDriver --> CamTopic
    DriverBase --> DiagTopic
    DriverBase --> HeartbeatTopic
    
    %% ROS2 to Orchestration
    HeartbeatTopic --> HealthMon
    DiagTopic --> FaultMgr
    HealthMon --> LifecycleMgr
    FaultMgr --> LifecycleMgr
    
    style ImuDriver fill:#90EE90
    style GpsDriver fill:#90EE90
    style FcDriver fill:#90EE90
    style CamDriver fill:#FFE4B5
    style DriverBase fill:#87CEEB
    style ErrorMgr fill:#DDA0DD
    style TraceMgr fill:#DDA0DD
    style PerfMon fill:#DDA0DD
    style QoSMgr fill:#DDA0DD
```

### Layer Responsibilities

**Hardware Layer**:
- Physical devices (IMU, GPS, Flight Controller, Camera)
- Device-specific protocols (I2C, SPI, UART, USB)
- No business logic

**HAL (Hardware Abstraction Layer)**:
- Trait-based abstraction over hardware
- Linux implementations for production
- Mock implementations for testing
- Error handling and timeout management
- Zero business logic

**Driver Layer**:
- ROS2 lifecycle node management
- Hardware-agnostic business logic via DriverBehavior trait
- Message conversion (HAL data → ROS2 messages)
- Publishing to ROS2 topics
- Heartbeat and diagnostics

**Cross-Cutting Concerns**:
- Error management (codes, severity, reporting)
- Traceability (REQ ID tracking)
- Performance monitoring (latency, throughput)
- QoS configuration

**Orchestration Layer** (Python):
- Health monitoring across all drivers
- Fault detection and failover
- Lifecycle management
- System-wide coordination

---

## Component Design

### HAL Layer Design

**Trait Definition**:
```rust
// hal/src/imu.rs
pub trait ImuHal: Send {
    fn read(&mut self) -> HalResult<ImuData>;
    
    #[cfg(any(test, feature = "test-utils"))]
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

pub struct ImuData {
    pub timestamp: u64,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}
```

**Linux Implementation**:
```rust
// hal/src/linux_imu.rs
pub struct LinuxImuHal {
    device_path: String,
    file: File,
    buffer: [u8; 256],
}

impl LinuxImuHal {
    pub fn new(device_path: String) -> HalResult<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&device_path)
            .map_err(|_| HalError::NotFound)?;
            
        Ok(Self {
            device_path,
            file,
            buffer: [0u8; 256],
        })
    }
}

impl ImuHal for LinuxImuHal {
    fn read(&mut self) -> HalResult<ImuData> {
        // Read from I2C device
        self.file.read_exact(&mut self.buffer[..14])
            .map_err(|_| HalError::Timeout)?;
            
        // Parse binary data
        Ok(ImuData {
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            accel_x: i16::from_le_bytes([self.buffer[0], self.buffer[1]]) as f32 / 16384.0,
            // ... parse remaining fields
        })
    }
}
```

**Mock Implementation**:
```rust
// hal/src/mock.rs
pub struct MockImuHal {
    fail_mode: MockFailMode,
    read_count: Arc<Mutex<u32>>,
}

impl MockImuHal {
    pub fn new() -> Self {
        Self {
            fail_mode: MockFailMode::None,
            read_count: Arc::new(Mutex::new(0)),
        }
    }
    
    pub fn new_timeout() -> Self {
        Self {
            fail_mode: MockFailMode::Timeout,
            read_count: Arc::new(Mutex::new(0)),
        }
    }
    
    pub fn set_fail_mode(&mut self, mode: MockFailMode) {
        self.fail_mode = mode;
    }
}

impl ImuHal for MockImuHal {
    fn read(&mut self) -> HalResult<ImuData> {
        *self.read_count.lock().unwrap() += 1;
        
        match self.fail_mode {
            MockFailMode::None => Ok(ImuData {
                timestamp: 1000,
                accel_x: 0.1,
                accel_y: 0.2,
                accel_z: 9.81,
                gyro_x: 0.01,
                gyro_y: 0.02,
                gyro_z: 0.03,
            }),
            MockFailMode::Timeout => Err(HalError::Timeout),
            MockFailMode::HardwareFault => Err(HalError::HardwareFault),
            MockFailMode::NotInitialized => Err(HalError::NotInitialized),
        }
    }
}
```

### Driver Layer Design

**DriverBehavior Trait**:
```rust
// drivers/src/base.rs
pub trait DriverBehavior: Send {
    fn initialize_hardware(&mut self) -> HalResult<()>;
    fn read_and_publish(&mut self) -> HalResult<()>;
    fn shutdown_hardware(&mut self) -> HalResult<()>;
}
```

**DriverNodeBase** (Generic Lifecycle Node):
```rust
// drivers/src/base.rs
pub struct DriverNodeBase<B: DriverBehavior> {
    behavior: Arc<Mutex<B>>,
    component_id: String,
    requirement_id: String,
    state: LifecycleState,
    error_manager: Arc<ErrorManager>,
    trace_manager: Arc<TraceabilityManager>,
    perf_monitor: Arc<PerformanceMonitor>,
}

impl<B: DriverBehavior> DriverNodeBase<B> {
    pub fn new(
        name: &str,
        behavior: B,
        component_id: String,
        requirement_id: String,
    ) -> Result<Self> {
        Ok(Self {
            behavior: Arc::new(Mutex::new(behavior)),
            component_id,
            requirement_id,
            state: LifecycleState::Unconfigured,
            error_manager: Arc::new(ErrorManager::new()),
            trace_manager: Arc::new(TraceabilityManager::new()),
            perf_monitor: Arc::new(PerformanceMonitor::new()),
        })
    }
    
    pub fn on_configure(&mut self) -> CallbackReturn {
        match self.behavior.lock().unwrap().initialize_hardware() {
            Ok(_) => {
                self.state = LifecycleState::Inactive;
                CallbackReturn::Success
            }
            Err(e) => {
                self.error_manager.report_error(
                    ErrorCode::DriverInitFailed,
                    &format!("Hardware init failed: {:?}", e),
                    Some(&self.component_id),
                );
                CallbackReturn::Failure
            }
        }
    }
    
    pub fn on_activate(&mut self) -> CallbackReturn {
        self.state = LifecycleState::Active;
        // Start publishing timer
        CallbackReturn::Success
    }
    
    pub fn timer_callback(&mut self) {
        let start = Instant::now();
        
        match self.behavior.lock().unwrap().read_and_publish() {
            Ok(_) => {
                let latency = start.elapsed();
                self.perf_monitor.record_latency(latency);
            }
            Err(e) => {
                self.handle_error(e);
            }
        }
    }
}
```

**IMU Driver Behavior**:
```rust
// drivers/src/imu_driver.rs
pub struct ImuDriverBehavior {
    hal: Box<dyn ImuHal>,
    device_path: String,
    publish_count: u64,
    initialized: bool,
}

impl DriverBehavior for ImuDriverBehavior {
    fn initialize_hardware(&mut self) -> HalResult<()> {
        // Test read to verify hardware
        self.hal.read()?;
        self.initialized = true;
        Ok(())
    }
    
    fn read_and_publish(&mut self) -> HalResult<()> {
        if !self.initialized {
            return Err(HalError::NotInitialized);
        }
        
        // Read from HAL
        let imu_data = self.hal.read()?;
        
        // Convert to ROS2 message
        let ros_msg = self.convert_to_ros_message(&imu_data);
        
        // Publish (placeholder until rclrs available)
        self.publish_count += 1;
        
        Ok(())
    }
    
    fn shutdown_hardware(&mut self) -> HalResult<()> {
        self.initialized = false;
        Ok(())
    }
}
```

---

##
 Test-Driven Development (TDD) Strategy

### TDD Workflow

**All implementation follows strict TDD red-green-refactor cycle** (Req 7.1, 7.2, 7.3):

1. **Red Phase**: Write failing tests first that specify desired behavior
   - Define test cases before any implementation
   - Tests should fail initially (no implementation exists yet)
   - Tests document expected behavior and API

2. **Green Phase**: Write minimal code to make tests pass
   - Implement only what's needed to pass tests
   - Focus on correctness, not optimization
   - All tests must pass before moving forward

3. **Refactor Phase**: Optimize while keeping tests green
   - Improve performance, readability, maintainability
   - Tests ensure refactoring doesn't break functionality
   - Run benchmarks to verify performance requirements

### Unit Testing Strategy

**HAL Layer Tests** (Write FIRST before implementation):
```rust
// hal/tests/imu_tests.rs
#[test]
fn test_imu_init_success() {
    let imu = MockImuHal::init("/dev/imu0", Duration::from_millis(100));
    assert!(imu.is_ok());
}

#[test]
fn test_imu_read_timeout() {
    let mut imu = MockImuHal::init("/dev/imu0", Duration::from_millis(100)).unwrap();
    imu.set_delay(Duration::from_millis(200)); // Exceed timeout
    
    let result = imu.read(Duration::from_millis(100));
    assert_eq!(result.unwrap_err(), HalStatus::Timeout);
}

#[bench]
fn bench_imu_read(b: &mut Bencher) {
    let mut imu = MockImuHal::init("/dev/imu0", Duration::from_millis(100)).unwrap();
    b.iter(|| {
        imu.read(Duration::from_millis(10))
    });
    // Assert: <1ms per read
}
```

Test coverage:
- Timeout behavior with simulated delays
- All error code paths (timeout, invalid param, hardware fault, not initialized)
- Drop trait cleanup
- Benchmark performance (<1ms timing constraint)
- Mock hardware interfaces using trait implementations
- Coverage target: 90%

**Driver Layer Tests** (Write FIRST before implementation):
```rust
// drivers/tests/imu_driver_tests.rs
#[test]
fn test_lifecycle_configure_success() {
    let behavior = ImuDriverBehavior::new_with_mock();
    let driver = DriverNodeBase::new("test_imu", behavior, "IMU".into(), "REQ-1.2".into()).unwrap();
    
    let result = driver.on_configure();
    assert_eq!(result, CallbackReturn::Success);
}

#[test]
fn test_read_and_publish_converts_correctly() {
    let mut behavior = ImuDriverBehavior::new_with_mock();
    behavior.mock_hal.set_data(ImuData {
        header: SensorHeader { timestamp_ns: 1000, sequence_id: 1, validity_flags: 0xFF },
        acceleration: [1.0, 2.0, 3.0],
        angular_velocity: [0.1, 0.2, 0.3],
        temperature: 25.0,
    });
    
    let result = behavior.read_and_publish();
    assert!(result.is_ok());
    
    // Verify published message
    let msg = behavior.get_last_published_message();
    assert_eq!(msg.linear_acceleration.x, 1.0);
    assert_eq!(msg.angular_velocity.z, 0.3);
}

#[test]
fn test_retry_logic_on_timeout() {
    let mut behavior = ImuDriverBehavior::new_with_mock();
    behavior.mock_hal.fail_count = 2; // Fail twice, succeed on 3rd
    
    let result = behavior.read_and_publish();
    assert!(result.is_ok());
    assert_eq!(behavior.mock_hal.call_count, 3); // Verify 3 attempts
}
```

Test coverage:
- Lifecycle state transitions (all valid and invalid transitions)
- DriverBehavior trait method calls
- Error handling and retry logic (max 3 attempts, exponential backoff)
- Heartbeat and diagnostic publishing
- Traceability IDs are set correctly
- Use mock HAL implementations
- Verify QoS configuration
- Coverage target: 85%

**Orchestration Layer Tests** (Python, Write FIRST):
```python
# tests/test_health_monitor.py
def test_component_timeout_detection():
    """Test that health monitor detects timeout within 200ms (REQ-11.1)"""
    monitor = HealthMonitor()
    monitor.set_parameter('heartbeat_timeout_ms', 200)
    
    # Send initial heartbeat
    monitor.heartbeat_callback(create_heartbeat_msg('imu_driver'))
    
    # Wait 250ms (exceeds timeout)
    time.sleep(0.25)
    
    # Check heartbeats should detect timeout
    monitor.check_heartbeats()
    
    # Verify failover was triggered
    assert monitor.failover_triggered_for == 'imu_driver'

def test_system_health_aggregation():
    """Test that system health aggregates all component states"""
    monitor = HealthMonitor()
    
    # Send heartbeats from multiple components
    monitor.heartbeat_callback(create_heartbeat_msg('imu_driver', HealthState.HEALTHY))
    monitor.heartbeat_callback(create_heartbeat_msg('gps_driver', HealthState.DEGRADED))
    
    health = monitor.get_system_health()
    assert health['imu_driver'].state == HealthState.HEALTHY
    assert health['gps_driver'].state == HealthState.DEGRADED
```

Test coverage:
- Integration tests with multiple mock drivers
- Fault detection within 200ms (REQ-11.1)
- Failover mechanisms
- Health monitoring aggregation
- Degraded mode operation
- Coverage target: 80%

### Integration Testing

**End-to-End Tests** (Write after unit tests pass):
- Hardware simulator publishes sensor data
- Verify data flows through HAL → Driver → ROS2 topics
- Measure end-to-end latency (<10ms for critical paths)
- Verify QoS compatibility between publishers and subscribers

**Fault Injection Tests**:
- Inject hardware timeouts at HAL layer
- Inject communication failures
- Verify graceful degradation
- Verify failover mechanisms activate
- Measure recovery time (<200ms for fault detection)

### Performance Testing

**Benchmarking Requirements** (Req 12.3):
- All performance-critical functions must have benchmark tests
- Benchmarks run in CI on every PR
- Regression detection: >5% slowdown fails CI
- Metrics: latency (p50, p95, p99), throughput, CPU usage, memory

**Benchmark Example**:
```rust
#[bench]
fn bench_imu_read_and_publish(b: &mut Bencher) {
    let mut driver = create_test_imu_driver();
    b.iter(|| {
        driver.behavior.lock().unwrap().read_and_publish()
    });
}

#[test]
fn test_latency_requirement() {
    let mut driver = create_test_imu_driver();
    let start = Instant::now();
    driver.behavior.lock().unwrap().read_and_publish().unwrap();
    let duration = start.elapsed();
    
    assert!(duration < Duration::from_millis(10), 
            "Latency {} exceeds 10ms requirement", duration.as_millis());
}
```

### TDD Benefits for This Project

**Junior-Proof** (Req 12.1):
- Tests document expected behavior and API usage
- New developers can understand system by reading tests
- Compiler + tests catch mistakes before runtime

**Regression Prevention**:
- Refactoring safe with comprehensive test suite
- Performance benchmarks catch regressions automatically
- CI fails on test failures or performance degradation

**Design Quality** (Req 2.1):
- TDD forces modular, testable design
- Interfaces emerge naturally from test requirements
- Tight coupling becomes obvious when writing tests

**Confidence**:
- High test coverage enables aggressive refactoring
- Safe to optimize performance (tests verify correctness)
- Safe to add features (tests verify no breakage)

### CI/CD Integration

**Pre-commit Hooks**:
- Run `cargo test` on changed files
- Run `cargo clippy` for linting
- Run `cargo fmt --check` for formatting
- Verify requirement IDs in commit messages

**CI Pipeline**:
```yaml
# .github/workflows/ci.yml
- name: Run tests
  run: cargo test --all-features
  
- name: Run benchmarks
  run: cargo bench --no-fail-fast
  
- name: Check for performance regression
  run: |
    cargo bench --bench imu_driver -- --save-baseline main
    cargo bench --bench imu_driver -- --baseline main
    # Fail if >5% regression
```

**Coverage Reporting**:
- Use `cargo tarpaulin` for coverage
- Fail CI if coverage drops below thresholds (HAL: 90%, Drivers: 85%, Orchestration: 80%)
- Generate coverage reports for each PR


### ADR-006: Test-Driven Development (TDD) Mandatory

**Decision**: All implementation must follow strict TDD red-green-refactor cycle

**Rationale**:
- **Junior-Proof**: Tests document expected behavior and API, preventing misuse (Req 12.1)
- **Performance Validation**: Benchmarks catch regressions automatically (Req 12.3)
- **Design Quality**: TDD forces modular, testable interfaces (Req 2.1, 2.2)
- **Confidence**: High coverage enables aggressive refactoring and optimization
- **Regression Prevention**: Comprehensive test suite prevents breaking changes
- **Documentation**: Tests serve as executable documentation
- **Perfectionist Approach**: Aligns with senior engineer's quality standards

**Consequences**:
- Slower initial development (write tests first)
- Requires discipline to follow red-green-refactor strictly
- Junior developers must learn TDD methodology
- More code to maintain (tests + implementation)
- Long-term velocity increase (fewer bugs, safer refactoring)
- Higher confidence in production deployment
