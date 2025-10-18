# Requirements Document - Utils Navigation Error Tracker Refactor

## Introduction

This specification defines the requirements for refactoring the Navigation Error Tracker from the utils package from Python to Rust. The Navigation Error Tracker calculates and publishes the navigation error between two NavSatFix sources (e.g., EKF output vs GNSS ground truth). This refactor aims to improve performance and determinism while maintaining 100% backward compatibility.

**Note**: Other utils components (AWS synchronizer, ROSBag recorder) will remain in Python as they are not performance-critical and involve external services.

## Glossary

- **Navigation_Error_Tracker**: The Rust-based ROS2 node that computes position error between two GPS sources
- **NavSatFix**: ROS2 message type containing GPS position (latitude, longitude, altitude)
- **Euclidean_Distance**: Straight-line distance in meters between two positions
- **Geodesic_Distance**: Distance along Earth's surface between two positions
- **Ground_Truth**: Reference GPS position (typically raw GNSS)
- **EKF_Output**: Estimated GPS position from Extended Kalman Filter

## Requirements

### Requirement 1: Position Error Calculation

**User Story:** As a navigation engineer, I want continuous calculation of position error, so that I can monitor localization accuracy in real-time.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL subscribe to two NavSatFix topics (fix and ground_truth)
2. WHEN THE Navigation_Error_Tracker receives messages from both topics, THE Navigation_Error_Tracker SHALL compute Euclidean distance error in XY plane
3. THE Navigation_Error_Tracker SHALL compute height difference in Z axis
4. THE Navigation_Error_Tracker SHALL publish error as PointStamped message with X (horizontal distance), Y (zero), Z (height difference)
5. THE Navigation_Error_Tracker SHALL use geodesic distance calculation for horizontal error

### Requirement 2: Message Synchronization

**User Story:** As a system integrator, I want temporally aligned error calculations, so that error measurements correspond to the same time instant.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL synchronize messages from both input topics
2. WHEN THE messages are synchronized, THE Navigation_Error_Tracker SHALL use timestamps within configurable tolerance
3. THE Navigation_Error_Tracker SHALL publish error with timestamp matching the fix message
4. THE Navigation_Error_Tracker SHALL handle missing or delayed messages gracefully
5. THE Navigation_Error_Tracker SHALL log warnings when synchronization fails

### Requirement 3: Invalid Data Handling

**User Story:** As a system operator, I want robust handling of invalid GPS data, so that the system continues operating during GPS outages.

#### Acceptance Criteria

1. WHEN THE Navigation_Error_Tracker receives NavSatFix with NaN coordinates, THE Navigation_Error_Tracker SHALL skip error calculation
2. WHEN THE Navigation_Error_Tracker receives NavSatFix with invalid coordinates (out of range), THE Navigation_Error_Tracker SHALL skip error calculation
3. THE Navigation_Error_Tracker SHALL log warnings for invalid data
4. THE Navigation_Error_Tracker SHALL continue processing when valid data resumes
5. THE Navigation_Error_Tracker SHALL publish diagnostic status indicating data quality

### Requirement 4: Backward Compatibility

**User Story:** As a system operator, I want the Rust implementation to be drop-in compatible with Python, so that I can migrate without changing other system components.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL subscribe to the same topic names as the Python implementation
2. THE Navigation_Error_Tracker SHALL publish to the same topic names as the Python implementation
3. THE Navigation_Error_Tracker SHALL use the same message types as the Python implementation
4. THE Navigation_Error_Tracker SHALL accept the same configuration parameters as the Python implementation
5. WHEN THE Navigation_Error_Tracker processes identical inputs as Python, THE Navigation_Error_Tracker SHALL produce numerically equivalent outputs within tolerance (1e-6 meters)

### Requirement 5: Performance Requirements

**User Story:** As a real-time systems engineer, I want low-latency error calculation, so that error metrics are available for real-time monitoring.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL process each message pair within 200 microseconds on target hardware
2. THE Navigation_Error_Tracker SHALL exhibit deterministic execution time with variance less than 20 microseconds
3. THE Navigation_Error_Tracker SHALL allocate memory during initialization only, with zero allocations during runtime
4. THE Navigation_Error_Tracker SHALL demonstrate at least 2x performance improvement over Python implementation
5. THE Navigation_Error_Tracker SHALL support message rates up to 100 Hz

### Requirement 6: Configuration and Launch

**User Story:** As a system deployer, I want flexible configuration options, so that I can adapt the tracker to different sensor configurations.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL load configuration from YAML parameter files
2. THE Navigation_Error_Tracker SHALL support configurable input topic names
3. THE Navigation_Error_Tracker SHALL support configurable output topic name
4. THE Navigation_Error_Tracker SHALL support configurable synchronization tolerance
5. THE Navigation_Error_Tracker SHALL validate configuration parameters at startup

### Requirement 7: Testing and Validation

**User Story:** As a quality assurance engineer, I want comprehensive test coverage, so that I can verify correctness and prevent regressions.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL include unit tests for geodesic distance calculation
2. THE Navigation_Error_Tracker SHALL include unit tests for invalid data handling
3. THE Navigation_Error_Tracker SHALL include integration tests comparing Rust output to Python baseline
4. THE Navigation_Error_Tracker SHALL include benchmark tests measuring latency
5. THE Navigation_Error_Tracker SHALL achieve 100% compatibility with Python implementation in validation tests

### Requirement 8: Documentation

**User Story:** As a developer, I want clear documentation, so that I can understand, maintain, and extend the system.

#### Acceptance Criteria

1. THE Navigation_Error_Tracker SHALL include inline code documentation for all public functions
2. THE Navigation_Error_Tracker SHALL provide README with usage examples and configuration guide
3. THE Navigation_Error_Tracker SHALL document geodesic distance calculation methodology
4. THE Navigation_Error_Tracker SHALL include migration guide from Python to Rust implementation
5. THE Navigation_Error_Tracker SHALL document performance characteristics
