# Backward Compatibility Guide

## Overview

The refactored Rust drivers MUST maintain 100% backward compatibility with the existing Python implementation to enable incremental deployment without breaking current integrations.

## Topic Compatibility Matrix

### Existing Python Topics (from prod-xnaut-core)

**autopilot_interface package:**
- Input: `/fusion_engine/odom` (nav_msgs/Odometry)
- Input: `/uav/mavros/global_position/raw/fix` (sensor_msgs/NavSatFix)
- Input: `/uav/mavros/global_position/gp_origin` (geographic_msgs/GeoPointStamped)
- Output: `/uav/mavros/vision_pose/pose_cov` (geometry_msgs/PoseWithCovarianceStamped)

**algorithms package:**
- Input: `/xnaut/mavros/global_position/raw/fix` (sensor_msgs/NavSatFix)
- Input: `/mavros/waypoint_reached` (mavros_msgs/WaypointReached)
- Output: `/fix` (sensor_msgs/NavSatFix)

### Refactored Rust Topics (MUST match exactly)

**Rust drivers MUST use identical topic names:**
```rust
// drivers/src/imu_driver.rs
const IMU_TOPIC: &str = "/uav/mavros/imu/data";  // Match existing if present

// drivers/src/gps_driver.rs  
const GPS_FIX_TOPIC: &str = "/uav/mavros/global_position/raw/fix";  // EXACT match

// drivers/src/fc_driver.rs
const VISION_POSE_TOPIC: &str = "/uav/mavros/vision_pose/pose_cov";  // EXACT match
```

## Message Type Compatibility

### Standard ROS2 Messages (Already Compatible)
- `sensor_msgs/NavSatFix` - GPS data
- `sensor_msgs/Imu` - IMU data
- `geometry_msgs/PoseWithCovarianceStamped` - Pose with covariance
- `nav_msgs/Odometry` - Odometry data
- `geographic_msgs/GeoPointStamped` - Geographic point

**Action Required:** None - Rust rclrs uses same message definitions

### Custom Messages (If Any)
If the existing system uses custom message types, create compatibility adapters:

```rust
// messages/src/adapters.rs
pub fn convert_legacy_to_standard(legacy_msg: LegacyCustomMsg) -> StandardMsg {
    // Conversion logic
}
```

## Parameter Compatibility

### Existing Python Parameters (from rclightning)

**NavSatFixExtNavNode parameters:**
- `ellipsoid_model`: string (default: "wgs84")
- `service_home_position`: string (default: "/uav/mavros/home_position/req_update")

**MapReaderNode parameters:**
- `localization_error_std_dev`: float
- `delay_mean`: float
- `delay_std_dev`: float
- `waypoints_off`: list[int]
- `waypoints_on`: list[int]
- `max_messages_per_second`: int

### Refactored Rust Parameters (MUST match names and types)

```rust
// drivers/src/gps_driver.rs
pub struct GpsDriverParams {
    pub ellipsoid_model: String,  // EXACT name match
    pub service_home_position: String,  // EXACT name match
    pub device_path: String,  // New parameter (optional)
    pub update_rate_hz: i64,  // New parameter (optional)
}

impl GpsDriverParams {
    pub fn from_ros_params(node: &Node) -> Self {
        Self {
            // Use EXACT parameter names from Python
            ellipsoid_model: node.declare_parameter("ellipsoid_model", "wgs84".into()),
            service_home_position: node.declare_parameter("service_home_position", 
                "/uav/mavros/home_position/req_update".into()),
            // New parameters with defaults
            device_path: node.declare_parameter("device_path", "/dev/gps0".into()),
            update_rate_hz: node.declare_parameter("update_rate_hz", 10),
        }
    }
}
```

## Namespace Compatibility

### Existing Namespace Structure
```
/uav/
  mavros/
    vision_pose/
      pose_cov
    global_position/
      raw/fix
      gp_origin
    imu/
      data
/xnaut/
  mavros/
    global_position/
      raw/fix
/fusion_engine/
  odom
/mavros/
  waypoint_reached
```

### Refactored System (MUST preserve structure)
```rust
// launch/drone_system.launch.py
LifecycleNode(
    package='drone_drivers',
    executable='gps_driver',
    name='gps_driver',
    namespace='uav/mavros/global_position',  // EXACT match
    remappings=[
        ('raw/fix', 'raw/fix'),  // Preserve topic structure
    ],
)
```

## Simultaneous Operation During Transition

### Feature Flag Strategy

**Phase 1: Side-by-side operation**
```python
# launch/hybrid_system.launch.py
DeclareLaunchArgument('use_rust_gps', default_value='false'),

# Old Python GPS driver
Node(
    package='autopilot_interface',
    executable='navsatfix_node',
    name='gps_driver_python',
    condition=UnlessCondition(LaunchConfiguration('use_rust_gps')),
),

# New Rust GPS driver
LifecycleNode(
    package='drone_drivers',
    executable='gps_driver',
    name='gps_driver_rust',
    condition=IfCondition(LaunchConfiguration('use_rust_gps')),
),
```

**Phase 2: A/B testing**
- Run both drivers simultaneously
- Publish Python to `/legacy/...` topics
- Publish Rust to standard topics
- Compare outputs for validation

```python
# Remap legacy driver to different namespace
Node(
    package='autopilot_interface',
    executable='navsatfix_node',
    name='gps_driver_python',
    remappings=[
        ('/uav/mavros/global_position/raw/fix', '/legacy/uav/mavros/global_position/raw/fix'),
    ],
),
```

## Configuration File Compatibility

### Existing YAML Config Format
```yaml
# config/map_reader_placeholder.yaml
mapreader:
  ros__parameters:
    localization_error_std_dev: 100.0
    delay_mean: 0.1
    delay_std_dev: 0.5
    waypoints_off: [3, 5, 7]
    waypoints_on: [4, 6, 8]
    max_messages_per_second: 5
```

### Refactored Config (MUST support same format)
```yaml
# config/gps_driver.yaml
gps_driver:
  ros__parameters:
    # Legacy parameters (EXACT names)
    ellipsoid_model: "wgs84"
    service_home_position: "/uav/mavros/home_position/req_update"
    
    # New parameters (optional, with defaults)
    device_path: "/dev/gps0"
    update_rate_hz: 10
    enable_performance_monitoring: true
```

## API Compatibility

### Service Interfaces
If existing Python nodes expose services, Rust must provide identical interfaces:

```rust
// services/src/home_position.rs
use mavros_msgs::srv::CommandHome;

pub struct HomePositionService {
    service: rclrs::Service<CommandHome>,
}

impl HomePositionService {
    pub fn new(node: &Node) -> Self {
        // Use EXACT service name from Python
        let service = node.create_service(
            "/uav/mavros/home_position/req_update",
            |request: CommandHome::Request| {
                // Handle request
            },
        );
        Self { service }
    }
}
```

## Testing Backward Compatibility

### Integration Tests

```rust
// tests/compatibility_tests.rs
#[test]
fn test_topic_names_match_legacy() {
    let gps_driver = GpsDriver::new();
    assert_eq!(gps_driver.get_output_topic(), "/uav/mavros/global_position/raw/fix");
}

#[test]
fn test_message_format_compatible() {
    let rust_msg = create_gps_message();
    let python_msg = load_legacy_message_from_bag();
    
    // Verify all fields match
    assert_eq!(rust_msg.latitude, python_msg.latitude);
    assert_eq!(rust_msg.longitude, python_msg.longitude);
    // ... verify all fields
}

#[test]
fn test_parameters_compatible() {
    let node = create_test_node();
    node.set_parameter("ellipsoid_model", "wgs84");
    
    let params = GpsDriverParams::from_ros_params(&node);
    assert_eq!(params.ellipsoid_model, "wgs84");
}
```

### Python Integration Test
```python
# tests/test_rust_python_compatibility.py
def test_rust_driver_compatible_with_python_subscriber():
    """Verify Python code can subscribe to Rust driver topics"""
    
    # Start Rust driver
    rust_driver = launch_rust_gps_driver()
    
    # Create Python subscriber (existing code)
    node = rclpy.create_node('test_subscriber')
    received_msgs = []
    
    def callback(msg):
        received_msgs.append(msg)
    
    sub = node.create_subscription(
        NavSatFix,
        '/uav/mavros/global_position/raw/fix',
        callback,
        10
    )
    
    # Wait for messages
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # Verify we received messages from Rust driver
    assert len(received_msgs) > 0
    assert isinstance(received_msgs[0], NavSatFix)
```

## Migration Checklist

- [ ] Verify all topic names match exactly (case-sensitive)
- [ ] Verify all message types are identical
- [ ] Verify all parameter names and types match
- [ ] Verify namespace structure preserved
- [ ] Test Python subscribers can receive Rust publisher messages
- [ ] Test Rust subscribers can receive Python publisher messages
- [ ] Test with existing launch files (should work with minimal changes)
- [ ] Test with existing config files (should work without modification)
- [ ] Verify QoS compatibility (Python default vs Rust configured)
- [ ] Test simultaneous operation (both drivers running)
- [ ] Document any breaking changes (should be NONE)

## Rollback Strategy

If compatibility issues arise:

1. **Immediate Rollback**
   ```bash
   ros2 param set /lifecycle_manager use_rust_gps false
   # System automatically switches back to Python driver
   ```

2. **Topic Remapping (Emergency)**
   ```python
   # Remap Rust driver to different topic temporarily
   remappings=[
       ('raw/fix', 'raw/fix_rust'),
   ]
   ```

3. **Feature Flag Disable**
   ```yaml
   # config/feature_flags.yaml
   use_refactored_gps_driver: false
   ```

## Compatibility Validation

Before deploying any refactored driver:

1. Run compatibility test suite (100% pass required)
2. Record rosbag from Python driver
3. Replay rosbag to Rust driver
4. Compare outputs (must be identical within tolerance)
5. Run side-by-side for 24 hours
6. Monitor for any integration issues
7. Get approval from integration team

## Known Compatibility Considerations

### QoS Policies
- **Python default**: Reliable, Volatile, KeepLast(10)
- **Rust configured**: May differ for performance
- **Solution**: Provide compatibility QoS profiles that match Python defaults

### Timing Differences
- **Python**: May have variable latency due to GC
- **Rust**: Consistent low latency
- **Impact**: Downstream nodes may receive data faster
- **Solution**: Add optional rate limiting to match Python timing if needed

### Error Handling
- **Python**: May silently fail or log warnings
- **Rust**: Explicit error handling with diagnostics
- **Impact**: More error messages visible
- **Solution**: Configure log levels to match Python verbosity
