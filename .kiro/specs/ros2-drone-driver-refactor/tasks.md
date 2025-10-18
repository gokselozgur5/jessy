# Implementation Plan

- [x] 1. Set up project structure and foundational infrastructure
  - Create ROS2 workspace with Rust + Python hybrid architecture
  - Create Cargo workspace with members: hal, drivers, qos, error, traceability, performance
  - Set up package.xml for ROS2 integration with ament_cargo build type
  - Create Python packages for orchestration (health_monitor, lifecycle_manager, fault_manager)
  - Configure Cargo.toml with rclrs, sensor_msgs, std_msgs dependencies
  - Set up clippy (linting) and rustfmt (formatting) configuration
  - Create directory structure: src/, tests/, config/, launch/, python/
  - _Requirements: REQ-2.1, REQ-2.2, REQ-12.1_

- [x] 1.1 Set up CI/CD pipeline and pre-commit hooks
  - Configure GitHub Actions or GitLab CI with build, test, lint, and benchmark stages
  - Create pre-commit hooks for clang-format, cpplint, clang-tidy
  - Set up automated traceability matrix generation
  - Configure coverage reporting with 80%+ thresholds
  - _Requirements: REQ-12.1, REQ-12.3_

- [x] 2. Implement Hardware Abstraction Layer (HAL) with TDD approach
- [x] 2.1 Write HAL interface tests FIRST (TDD Red phase)
  - Create hal/tests/imu_tests.rs with test cases for ImuHal trait
  - Write test for successful init() with valid device path
  - Write test for init() timeout behavior
  - Write test for read() with valid data
  - Write test for read() timeout behavior
  - Write test for error code paths (InvalidParam, HardwareFault, NotInitialized)
  - Write test verifying Drop trait cleanup
  - Write benchmark test for read() operation (<1ms target)
  - _Requirements: REQ-7.1, REQ-1.1, REQ-5.5_

- [x] 2.2 Create HAL mock implementations for tests
  - Implement MockImuHal struct with configurable delays and error injection
  - Implement MockGpsHal with simulated GPS data
  - Implement MockFlightControllerHal for command simulation
  - Add methods to inject specific error conditions
  - _Requirements: REQ-7.4_

- [x] 2.3 Implement HAL interfaces to pass tests (TDD Green phase)
  - Create hal/src/types.rs with HalStatus enum, HalResult type, SensorHeader struct
  - Create hal/src/imu.rs with ImuHal trait and ImuData struct
  - Implement Bmi088Imu struct with init(), read(), shutdown() methods to pass tests
  - Create hal/src/gps.rs with GpsHal trait and GpsData struct
  - Create hal/src/flight_controller.rs with FlightControllerHal trait
  - Add explicit timeout parameters (Duration) to all blocking operations
  - Use pre-allocated buffers ([u8; 256]), no heap allocation in read path
  - Implement Drop trait for automatic cleanup
  - Use #[repr(C)] for FFI compatibility if needed
  - Run tests until all pass (cargo test)
  - _Requirements: REQ-1.1, REQ-5.5, REQ-6.1_

- [ ]* 2.4 Refactor HAL implementation (TDD Refactor phase)
  - Optimize for performance while keeping tests green
  - Remove code duplication
  - Improve error messages
  - Run benchmarks to verify <1ms latency
  - _Requirements: REQ-1.1_

- [x] 3. Implement error management system with TDD approach
- [x] 3.1 Write error management tests FIRST (TDD Red phase)
  - Create error/tests/manager_tests.rs
  - Write test for report_error() with Fatal severity
  - Write test for report_error() with Error severity
  - Write test for report_error() with Warning and Info severities
  - Write test for retry_with_backoff() succeeding on 2nd attempt
  - Write test for retry_with_backoff() failing after max attempts (3)
  - Write test for exponential backoff timing (initial_delay, 2x, 4x)
  - Write test for recovery strategy registration and execution
  - Write test for diagnostic message publishing
  - _Requirements: REQ-7.1, REQ-10.1, REQ-10.2, REQ-10.5_

- [x] 3.2 Implement error management to pass tests (TDD Green phase)
  - Create error/src/manager.rs with ErrorSeverity enum and ErrorContext struct
  - Create error/src/codes.rs module with centralized error code constants
  - Implement ErrorManager struct with report_error() method and severity-based handling
  - Implement retry_with_backoff() generic function with exponential backoff (max 3 attempts)
  - Implement recovery strategy registration using HashMap<u32, Box<dyn Fn() -> bool>>
  - Add ROS2 diagnostic message publishing for errors using rclrs
  - Use Result<T, ErrorContext> pattern throughout codebase
  - Run tests until all pass (cargo test)
  - _Requirements: REQ-10.1, REQ-10.2, REQ-10.3, REQ-10.4, REQ-10.5_

- [x] 4. Implement traceability system with TDD approach
- [x] 4.1 Write traceability tests FIRST (TDD Red phase)
  - Create traceability/tests/manager_tests.rs
  - Write test for begin_trace() creating TraceContext with unique ID
  - Write test for end_trace() recording success/failure
  - Write test for query_by_requirement() returning matching traces
  - Write test for query_by_component() returning matching traces
  - Write test for ring buffer overflow behavior (oldest traces dropped)
  - Write benchmark test for trace overhead (<100ns target)
  - Write test for concurrent tracing from multiple threads
  - _Requirements: REQ-7.1, REQ-9.1, REQ-9.4, REQ-9.5_

- [x] 4.2 Implement traceability system to pass tests (TDD Green phase)
  - Create traceability/src/manager.rs with TraceContext struct
  - Implement TraceManager with begin_trace() and end_trace()
  - Implement ring buffer for bounded memory trace storage using VecDeque
  - Implement query_by_requirement() and query_by_component()
  - Add trace ID generation using uuid crate or timestamp-based
  - Use Arc<Mutex<>> for thread-safe access
  - Run tests until all pass (cargo test)
  - _Requirements: REQ-9.1, REQ-9.2, REQ-9.4, REQ-9.5_

- [x] 5. Implement QoS configuration system with TDD approach
- [x] 5.1 Write QoS tests FIRST (TDD Red phase)
  - Create qos/tests/profiles_tests.rs
  - Write test verifying CONTROL_COMMAND has Reliable + KeepLast(1) + 10ms deadline
  - Write test verifying HIGH_FREQ_SENSOR has BestEffort
  - Write test for validate_compatibility() with compatible profiles (should pass)
  - Write test for validate_compatibility() with incompatible reliability (should fail)
  - Write test for validate_compatibility() with incompatible durability (should fail)
  - Write test verifying profiles are const and cannot be modified
  - Write test verifying all profiles have rationale and requirement_refs
  - _Requirements: REQ-7.1, REQ-3.1, REQ-3.2, REQ-3.4_

- [x] 5.2 Implement QoS system to pass tests (TDD Green phase)
  - Create qos/src/profiles.rs with QoSConfig struct
  - Define const QoSConfig::CONTROL_COMMAND with Reliable, KeepLast(1), 10ms deadline
  - Define const QoSConfig::HIGH_FREQ_SENSOR with BestEffort for low latency
  - Define const QoSConfig::DIAGNOSTIC and QoSConfig::HEARTBEAT
  - Implement validate_compatibility() function for QoS checking
  - Add rationale &'static str and requirement_refs &'static [&'static str] to each profile
  - Use rclrs::QoSProfile for ROS2 integration
  - Run tests until all pass (cargo test)
  - _Requirements: REQ-3.1, REQ-3.2, REQ-3.3, REQ-3.4, REQ-3.5_

- [x] 6. Implement driver node base pattern with TDD approach
- [x] 6.1 Write driver base tests FIRST (TDD Red phase)
  - Create drivers/tests/base_tests.rs
  - Write test for lifecycle transition: Unconfigured -> Configured (on_configure success)
  - Write test for lifecycle transition: Configured -> Active (on_activate)
  - Write test for lifecycle transition: Active -> Inactive (on_deactivate)
  - Write test for lifecycle transition: Inactive -> Unconfigured (on_cleanup)
  - Write test for on_configure failure when initialize_hardware() fails
  - Write test for heartbeat publishing at expected rate
  - Write test for diagnostic publishing with component_id and requirement_id
  - Write test for trace_operation() wrapping and timing
  - Write test verifying DriverBehavior trait methods are called correctly
  - _Requirements: REQ-7.2, REQ-2.4, REQ-11.5_

- [x] 6.2 Implement driver base to pass tests (TDD Green phase)
  - Create drivers/src/base.rs with DriverBehavior trait
  - Define HealthState enum (Healthy, Degraded, Failed, Recovering)
  - Define DriverBehavior trait with initialize_hardware(), read_and_publish(), shutdown_hardware() methods
  - Create DriverNodeBase<T: DriverBehavior> generic struct
  - Implement lifecycle callbacks (on_configure, on_activate, on_deactivate, on_cleanup) - cannot be overridden
  - Add publish_heartbeat() method
  - Add publish_diagnostics() method
  - Add trace_operation() generic method with closure for automatic tracing
  - Integrate PerformanceMonitor, ErrorManager, TraceManager using Arc<Mutex<>>
  - Add component_id and requirement_id String fields
  - Use rclrs::LifecycleNode for ROS2 integration
  - Run tests until all pass (cargo test)
  - _Requirements: REQ-2.4, REQ-11.5, REQ-12.1_

- [x] 7. Implement performance monitoring system
  - Create performance_monitor.hpp/.cpp with latency tracking
  - Implement start_measurement() and end_measurement() with RAII guard
  - Track p50, p95, p99 latencies using histogram
  - Implement publish_metrics() to ROS2 topic at 1Hz
  - Add CPU and memory usage monitoring
  - Implement threshold violation detection and alerting
  - _Requirements: REQ-1.5, REQ-12.3_

- [x] 7.1 Write performance monitoring tests
  - Test latency measurement accuracy
  - Test percentile calculations
  - Test metrics publishing
  - Verify minimal overhead (<50ns per measurement)
  - _Requirements: REQ-7.1_

- [x] 8. Implement IMU driver node with TDD approach
- [x] 8.1 Write IMU driver tests FIRST (TDD Red phase)
  - Create drivers/tests/imu_driver_tests.rs
  - Write test for initialize_hardware() success with MockImuHal
  - Write test for initialize_hardware() failure and error reporting
  - Write test for read_and_publish() converting ImuData to sensor_msgs::msg::Imu correctly
  - Write test for read_and_publish() with HAL timeout and retry logic
  - Write test verifying 100Hz publishing rate (10ms period)
  - Write test for lifecycle transitions (configure, activate, deactivate, cleanup)
  - Write benchmark test for end-to-end latency (<10ms target)
  - Write test for handle_error() retry behavior (max 3 attempts)
  - _Requirements: REQ-7.2, REQ-1.1, REQ-1.2, REQ-5.5_

- [x] 8.2 Implement IMU driver to pass tests (TDD Green phase)
  - Create drivers/src/imu_driver.rs with ImuDriverBehavior struct
  - Implement DriverBehavior trait for ImuDriverBehavior
  - Implement initialize_hardware() calling Bmi088Imu::init()
  - Create rclrs::Publisher<sensor_msgs::msg::Imu> with QoSConfig::HIGH_FREQ_SENSOR
  - Implement read_and_publish() converting ImuData to sensor_msgs::msg::Imu
  - Create main() function with 100Hz timer (Duration::from_millis(10))
  - Implement handle_error() with retry logic for timeouts
  - Add performance tracing around HAL calls using trace_operation()
  - Run tests until all pass (cargo test)
  - Run benchmarks to verify <10ms latency
  - _Requirements: REQ-1.1, REQ-1.2, REQ-3.2, REQ-5.5_

- [x] 9. Implement GPS driver node
  - Create gps_driver_node.hpp/.cpp extending DriverNodeBase
  - Implement initialize_hardware() calling hal_gps_init()
  - Implement read_and_publish() with 10Hz timer (100ms period)
  - Convert hal_gps_data_t to sensor_msgs::msg::NavSatFix
  - Use QoSRegistry::get_high_freq_sensor_qos() for publisher
  - Implement handle_error() with retry logic
  - Add performance tracing
  - _Requirements: REQ-1.2, REQ-3.2, REQ-5.5_

- [x] 9.1 Write GPS driver integration tests
  - Test with mock HAL implementation
  - Verify 10Hz publishing rate
  - Test error handling
  - Test lifecycle transitions
  - _Requirements: REQ-7.2_

- [x] 10. Implement flight controller driver node
  - Create fc_driver_node.hpp/.cpp extending DriverNodeBase
  - Implement initialize_hardware() calling hal_fc_init()
  - Create subscriber for control commands using QoSRegistry::get_control_command_qos()
  - Implement command callback with <10ms latency requirement
  - Convert ROS2 ControlCommand message to HAL format
  - Call hal_fc_write() with timeout
  - Implement handle_error() with FATAL severity for communication failures
  - Add performance tracing and latency measurement
  - _Requirements: REQ-1.1, REQ-3.1, REQ-5.1, REQ-10.2_

- [x] 10.1 Write flight controller driver integration tests
  - Test with mock HAL implementation
  - Verify <10ms command processing latency
  - Test error handling and FATAL error escalation
  - Test lifecycle transitions
  - _Requirements: REQ-7.2_

- [x] 11. Implement health monitoring system in Python
  - Create python/orchestration/health_monitor.py as rclpy.Node
  - Implement register_component() with heartbeat interval and timeout configuration
  - Create subscriber for driver heartbeats (diagnostic_msgs::msg::DiagnosticStatus)
  - Create check_heartbeats() timer running at 5Hz (200ms period)
  - Detect component timeouts within 200ms using time.time() comparison (REQ-11.1)
  - Implement get_system_health() method returning component health dict
  - Add callback mechanism for timeout, degraded, and failed events
  - Publish aggregated system health diagnostics to /drone/diagnostics/system_health
  - _Requirements: REQ-11.1, REQ-11.5_

- [x] 11.1 Write health monitor unit tests
  - Test component registration
  - Test heartbeat timeout detection (200ms)
  - Test callback invocation
  - Test system health aggregation
  - _Requirements: REQ-7.3_

- [x] 12. Implement fault management system
  - Create fault_manager.hpp/.cpp as rclcpp::Node
  - Implement register_redundancy() for primary/backup component pairs
  - Implement failover_to_backup() with automatic lifecycle management
  - Implement isolate_component() to prevent fault propagation
  - Implement enable_degraded_mode() with sensor fusion logic
  - Integrate with HealthMonitor for fault detection
  - Integrate with LifecycleManager for component control
  - _Requirements: REQ-11.2, REQ-11.3, REQ-11.4_

- [x] 12.1 Write fault manager integration tests
  - Test redundancy registration
  - Test automatic failover on component failure
  - Test fault isolation
  - Test degraded mode operation
  - _Requirements: REQ-7.3_

- [x] 13. Implement lifecycle management system
  - Create lifecycle_manager.hpp/.cpp as rclcpp::Node
  - Implement component registration with startup/shutdown ordering
  - Implement start_all_components() with ordered activation
  - Implement stop_all_components() with reverse-ordered deactivation
  - Implement restart_component() for individual component recovery
  - Add service interfaces for manual lifecycle control
  - Integrate with FaultManager for coordinated recovery
  - _Requirements: REQ-2.4, REQ-8.3_

- [x] 13.1 Write lifecycle manager tests
  - Test ordered startup/shutdown
  - Test individual component restart
  - Test service interfaces
  - _Requirements: REQ-7.3_

- [x] 14. Implement feature flag system
  - Create feature_flags.hpp/.cpp with static configuration
  - Implement use_refactored_imu_driver() and similar flags for each driver
  - Load flags from YAML configuration file
  - Implement runtime flag checking in driver initialization
  - Add flag status to diagnostic messages
  - Create launch file parameter overrides for flags
  - _Requirements: REQ-8.4, REQ-8.5_

- [x] 15. Create ROS2 message definitions
  - Define ControlCommand.msg with roll_rate, pitch_rate, yaw_rate, thrust, control_mode
  - Define DiagnosticStatus.msg with component_id, requirement_id, health_state, error_code
  - Define PerformanceMetrics.msg with latency percentiles, CPU, memory
  - Define SystemHealth.msg with component health array
  - Add range validation comments to message definitions
  - _Requirements: REQ-1.1, REQ-9.4, REQ-10.3_

- [x] 16. Create launch files and configuration in Python
  - Create launch/drone_system.launch.py to start all driver nodes (Rust) and orchestration (Python)
  - Use LifecycleNode actions for Rust drivers (imu_driver, gps_driver, fc_driver)
  - Use Node actions for Python orchestration (health_monitor, lifecycle_manager, fault_manager)
  - Create config/qos_profiles.yaml documenting QoS rationale
  - Create config/error_codes.yaml for error code documentation
  - Create config/feature_flags.yaml for deployment control (use_refactored_imu, etc.)
  - Create config/performance_thresholds.yaml for monitoring
  - Add DeclareLaunchArgument for feature flags with default values
  - Add parameter validation in launch files
  - _Requirements: REQ-8.4, REQ-12.1_

- [x] 17. Implement structured logging system
  - Create structured_logger.hpp/.cpp with severity levels
  - Implement log_with_context() including component_id, requirement_id, trace_id
  - Integrate with ROS2 logging (RCLCPP_INFO, RCLCPP_ERROR, etc.)
  - Add JSON-formatted log output option for parsing
  - Implement log filtering by component and severity
  - _Requirements: REQ-9.4, REQ-10.3_

- [x] 18. Create code generation templates
  - Create template for new driver node with all boilerplate
  - Create template for new HAL interface
  - Create template for new error code definition
  - Create Python script to generate code from templates
  - Add validation to ensure generated code follows patterns
  - _Requirements: REQ-12.2_

- [x] 19. Implement traceability matrix generation
  - Create Python script to parse requirement IDs from code comments
  - Generate requirements.md → design.md → source files mapping
  - Generate HTML traceability matrix report
  - Integrate into CI pipeline
  - Add validation to detect missing requirement references
  - _Requirements: REQ-9.3, REQ-9.5_

- [x] 20. Create comprehensive documentation
  - Write README.md with architecture overview and getting started guide
  - Document each ADR in docs/adr/ directory
  - Create operator manual with error code reference and troubleshooting
  - Create developer guide with coding standards and patterns
  - Document QoS rationale and configuration guide
  - Add inline Doxygen comments to all public APIs
  - _Requirements: REQ-12.5_

- [x] 21. Create end-to-end integration test suite
  - Create test scenario with all drivers running simultaneously
  - Simulate sensor data and control commands
  - Verify end-to-end message flow
  - Inject faults and verify recovery
  - Measure system-wide performance under load
  - Run 24-hour stability test
  - _Requirements: REQ-7.2, REQ-7.3_

- [x] 22. Create performance benchmark suite
  - Benchmark HAL read/write operations
  - Benchmark driver node latency (command to hardware)
  - Benchmark message publishing latency
  - Benchmark error handling overhead
  - Benchmark tracing overhead
  - Create CI job to run benchmarks and detect regressions (>5%)
  - _Requirements: REQ-7.3, REQ-12.3_

- [x] 23. Implement backward compatibility layer with TDD
- [x] 23.1 Write compatibility tests FIRST (TDD Red phase)
  - Create tests/compatibility_tests.rs ✓
  - Write test verifying topic names match legacy Python system exactly ✓
  - Write test verifying message types are identical to Python ✓
  - Write test verifying parameter names and types match Python ✓
  - Write test for Python subscriber receiving Rust publisher messages ✓
  - Write test for Rust subscriber receiving Python publisher messages ✓
  - Write test for simultaneous Python and Rust driver operation ✓
  - Write test verifying namespace structure preserved ✓
  - Tests currently FAILING (expected - TDD Red phase) ✓
  - _Requirements: REQ-13.1, REQ-13.2, REQ-13.3, REQ-13.4, REQ-13.6_

- [x] 23.2 Implement compatibility layer to pass tests (TDD Green phase)
  - Update all driver topic names to match existing Python topics exactly
  - Verify GPS driver uses `/uav/mavros/global_position/raw/fix`
  - Verify vision pose uses `/uav/mavros/vision_pose/pose_cov`
  - Update parameter names to match Python rclightning parameters
  - Create hybrid launch file supporting both Python and Rust drivers
  - Add feature flags for gradual migration (use_rust_gps, use_rust_imu, etc.)
  - Implement topic remapping for side-by-side operation
  - Create compatibility QoS profiles matching Python defaults
  - Run tests until all pass
  - _Requirements: REQ-8.3, REQ-13.1, REQ-13.2, REQ-13.3, REQ-13.4, REQ-13.6_

- [ ]* 23.3 Create compatibility validation suite
  - Record rosbag from existing Python drivers
  - Create playback test comparing Python vs Rust outputs
  - Create 24-hour side-by-side operation test
  - Document migration checklist
  - _Requirements: REQ-13.6_

- [ ] 24. Create deployment and rollback procedures
  - Document phased deployment strategy (camera → GPS → IMU → FC)
  - Create rollback scripts for each component
  - Document monitoring checklist for each deployment phase
  - Create runbook for common failure scenarios
  - Add health check scripts for post-deployment validation
  - _Requirements: REQ-8.5_

- [ ] 25. Final integration and system validation
  - Deploy all refactored drivers with feature flags enabled
  - Run full integration test suite
  - Verify all performance requirements met (<10ms latency, <40% CPU)
  - Verify all error handling and fault tolerance requirements
  - Verify complete traceability from requirements to runtime
  - Generate final traceability matrix and coverage reports
  - _Requirements: REQ-1.1, REQ-1.2, REQ-1.4, REQ-9.3_
