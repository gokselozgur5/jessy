# Rust-ROS2 Integration Guide

**Requirements:** REQ-1.1, REQ-1.2, REQ-2.4  
**Status:** Reference Documentation  
**Last Updated:** 2025-10-18

## Overview

This guide provides detailed instructions for integrating Rust code with ROS2 using rclrs (ROS Client Library for Rust). It covers version pinning, build system setup, cross-compilation, and async runtime considerations.

---

## rclrs Version Pinning Strategy

### Current Versions

```toml
# Cargo.toml
[dependencies]
rclrs = "0.4.1"  # Pinned to specific version
```

### Version Compatibility Matrix

| rclrs Version | ROS2 Distribution | Rust Version | Status      |
|---------------|-------------------|--------------|-------------|
| 0.4.1         | Humble (22.04)    | 1.70+        | ✅ Stable   |
| 0.4.0         | Humble (22.04)    | 1.70+        | ⚠️ Buggy    |
| 0.3.x         | Galactic          | 1.65+        | ❌ EOL      |

### Pinning Rationale

**Why pin versions?**
- **Stability**: Avoid breaking changes from automatic updates
- **Reproducibility**: Ensure consistent builds across environments
- **Testing**: Validate specific version combinations
- **Production Safety**: Prevent unexpected behavior in production

**When to update:**
- Security vulnerabilities discovered
- Critical bug fixes released
- New features required for functionality
- After thorough testing in development environment

### Update Process

```bash
# 1. Check for updates
cargo outdated

# 2. Update in development branch
cargo update rclrs

# 3. Run full test suite
cargo test --all

# 4. Run integration tests
./scripts/integration_tests.sh

# 5. Benchmark performance
cargo bench

# 6. If all pass, update version in Cargo.toml
# 7. Document changes in CHANGELOG.md
```

---

## ament_cargo Build System Setup

### Overview

`ament_cargo` is the bridge between ROS2's ament build system and Rust's cargo. It allows Rust packages to be built as part of a ROS2 workspace.

### Package Structure

```
drone_drivers_rust/
├── Cargo.toml          # Rust package manifest
├── package.xml         # ROS2 package manifest
├── src/
│   └── lib.rs
├── tests/
└── benches/
```

### package.xml Configuration

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone_drivers_rust</name>
  <version>0.1.0</version>
  <description>High-performance Rust drone drivers</description>
  <maintainer email="team@drone.com">Drone Team</maintainer>
  <license>MIT</license>

  <!-- Build tools -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cargo</buildtool_depend>
  
  <!-- ROS2 dependencies -->
  <depend>rclrs</depend>
  <depend>sensor_msgs</depend>
  <depend>diagnostic_msgs</depend>
  <depend>lifecycle_msgs</depend>
  <depend>std_msgs</depend>
  
  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

### CMakeLists.txt Configuration

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_drivers_rust)

# Find ament_cargo
find_package(ament_cmake REQUIRED)
find_package(ament_cargo REQUIRED)

# Find ROS2 dependencies
find_package(rclrs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Build Rust package
ament_cargo_build(
  PACKAGE_NAME ${PROJECT_NAME}
  CARGO_MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
)

# Install Rust binaries
ament_cargo_install(
  PACKAGE_NAME ${PROJECT_NAME}
  CARGO_MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
)

# Export dependencies
ament_export_dependencies(
  rclrs
  sensor_msgs
  diagnostic_msgs
)

ament_package()
```

### Cargo.toml Configuration

```toml
[package]
name = "drone_drivers_rust"
version = "0.1.0"
edition = "2021"

[lib]
name = "drone_drivers"
path = "src/lib.rs"

[[bin]]
name = "imu_driver"
path = "src/bin/imu_driver.rs"

[[bin]]
name = "gps_driver"
path = "src/bin/gps_driver.rs"

[dependencies]
rclrs = "0.4.1"
sensor_msgs = { version = "0.4", features = ["rclrs"] }
diagnostic_msgs = { version = "0.4", features = ["rclrs"] }
lifecycle_msgs = { version = "0.4", features = ["rclrs"] }

# Internal dependencies
hal = { path = "hal" }
error = { path = "error" }
traceability = { path = "traceability" }

# External dependencies
thiserror = "1.0"
tracing = "0.1"
tokio = { version = "1.0", features = ["full"] }

[dev-dependencies]
criterion = "0.5"

[profile.release]
opt-level = 3
lto = true
codegen-units = 1
```

### Building with colcon

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
cd ~/ros2_ws
colcon build --packages-select drone_drivers_rust

# Source workspace
source install/setup.bash

# Run driver
ros2 run drone_drivers_rust imu_driver
```

### Build Optimization

```bash
# Release build with optimizations
colcon build --packages-select drone_drivers_rust --cmake-args -DCMAKE_BUILD_TYPE=Release

# Parallel build
colcon build --packages-select drone_drivers_rust --parallel-workers 8

# Clean build
colcon build --packages-select drone_drivers_rust --cmake-clean-cache
```

---

## Cross-Compilation for ARM

### Target Platforms

| Platform      | Target Triple              | Use Case                    |
|---------------|----------------------------|-----------------------------|
| Raspberry Pi 4| aarch64-unknown-linux-gnu  | Development/Testing         |
| Jetson Nano   | aarch64-unknown-linux-gnu  | Production (GPU available)  |
| Jetson Xavier | aarch64-unknown-linux-gnu  | Production (High performance)|

### Setup Cross-Compilation Toolchain

```bash
# Install Rust target
rustup target add aarch64-unknown-linux-gnu

# Install cross-compilation tools (Ubuntu/Debian)
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Install cross-compilation tools (macOS)
brew install aarch64-linux-gnu-gcc
```

### Configure Cargo for Cross-Compilation

Create `.cargo/config.toml`:

```toml
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"

[target.aarch64-unknown-linux-gnu.rclrs]
# Point to ARM sysroot with ROS2 libraries
rustflags = [
    "-C", "link-arg=--sysroot=/opt/ros2-arm-sysroot",
    "-L", "/opt/ros2-arm-sysroot/opt/ros/humble/lib",
]
```

### Cross-Compile ROS2 Dependencies

```bash
# On development machine, create ARM sysroot
mkdir -p /opt/ros2-arm-sysroot

# Copy ROS2 libraries from target device
scp -r pi@raspberrypi:/opt/ros/humble /opt/ros2-arm-sysroot/opt/ros/

# Or build ROS2 from source for ARM
# (See ROS2 cross-compilation documentation)
```

### Build for ARM

```bash
# Set environment variables
export PKG_CONFIG_SYSROOT_DIR=/opt/ros2-arm-sysroot
export PKG_CONFIG_PATH=/opt/ros2-arm-sysroot/opt/ros/humble/lib/pkgconfig

# Build with cargo
cargo build --target aarch64-unknown-linux-gnu --release

# Or build with colcon
colcon build \
  --packages-select drone_drivers_rust \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
```

### Deploy to Target Device

```bash
# Copy binary to target
scp target/aarch64-unknown-linux-gnu/release/imu_driver pi@raspberrypi:~/

# SSH to target and run
ssh pi@raspberrypi
./imu_driver
```

### Docker-Based Cross-Compilation

```dockerfile
# Dockerfile.arm64
FROM ros:humble

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install ARM target
RUN rustup target add aarch64-unknown-linux-gnu

# Install cross-compilation tools
RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu

# Copy source
WORKDIR /workspace
COPY . .

# Build
RUN cargo build --target aarch64-unknown-linux-gnu --release
```

```bash
# Build Docker image
docker build -f Dockerfile.arm64 -t drone-drivers-arm64 .

# Extract binary
docker create --name temp drone-drivers-arm64
docker cp temp:/workspace/target/aarch64-unknown-linux-gnu/release/imu_driver ./
docker rm temp
```

---

## Async Runtime Considerations

### Tokio vs rclrs Executor

**Challenge**: rclrs has its own executor, Tokio has its own executor. How to integrate?

**Solution**: Use Tokio for async I/O, rclrs executor for ROS2 callbacks.

### Architecture

```rust
use tokio::runtime::Runtime;
use rclrs::{Context, Node};

pub struct AsyncDriver {
    node: Arc<Node>,
    tokio_runtime: Runtime,
}

impl AsyncDriver {
    pub fn new() -> Result<Self> {
        // Create ROS2 node
        let context = Context::new(std::env::args())?;
        let node = context.create_node("async_driver")?;
        
        // Create Tokio runtime
        let tokio_runtime = Runtime::new()?;
        
        Ok(Self {
            node: Arc::new(node),
            tokio_runtime,
        })
    }
    
    pub fn spin(&self) {
        // Spawn Tokio tasks
        let node = self.node.clone();
        self.tokio_runtime.spawn(async move {
            // Async I/O operations
            Self::async_io_task(node).await;
        });
        
        // Spin ROS2 executor (blocking)
        rclrs::spin(self.node.clone()).unwrap();
    }
    
    async fn async_io_task(node: Arc<Node>) {
        // Use Tokio for async I/O
        let mut interval = tokio::time::interval(Duration::from_millis(10));
        
        loop {
            interval.tick().await;
            
            // Read from hardware asynchronously
            let data = Self::read_hardware_async().await;
            
            // Publish to ROS2 (sync call from async context)
            // This is safe because publishing is thread-safe
            node.publish(&data);
        }
    }
}
```

### Best Practices

**1. Separate Concerns**:
- Use Tokio for I/O-bound operations (file I/O, network, hardware)
- Use rclrs executor for ROS2 callbacks (subscriptions, services)

**2. Avoid Blocking in Async**:
```rust
// ❌ Bad: Blocking in async context
async fn bad_example() {
    std::thread::sleep(Duration::from_secs(1));  // Blocks executor!
}

// ✅ Good: Use async sleep
async fn good_example() {
    tokio::time::sleep(Duration::from_secs(1)).await;
}
```

**3. Use Channels for Communication**:
```rust
use tokio::sync::mpsc;

// Async task sends data
let (tx, mut rx) = mpsc::channel(100);

tokio::spawn(async move {
    let data = read_sensor().await;
    tx.send(data).await.unwrap();
});

// ROS2 callback receives data
while let Some(data) = rx.recv().await {
    publisher.publish(&data);
}
```

**4. Handle Backpressure**:
```rust
// Use bounded channels to prevent memory exhaustion
let (tx, rx) = mpsc::channel(100);  // Max 100 messages buffered

// If channel is full, oldest messages are dropped
if tx.try_send(data).is_err() {
    warn!("Channel full, dropping message");
}
```

### Performance Considerations

**Tokio Runtime Configuration**:
```rust
// Multi-threaded runtime for I/O-bound work
let runtime = tokio::runtime::Builder::new_multi_thread()
    .worker_threads(4)
    .thread_name("tokio-worker")
    .enable_all()
    .build()?;

// Single-threaded runtime for CPU-bound work
let runtime = tokio::runtime::Builder::new_current_thread()
    .enable_all()
    .build()?;
```

**Avoid Context Switching**:
```rust
// ❌ Bad: Frequent context switches
for i in 0..1000 {
    tokio::spawn(async move {
        // Each spawn creates a task
    });
}

// ✅ Good: Batch operations
tokio::spawn(async move {
    for i in 0..1000 {
        // Single task handles all iterations
    }
});
```

---

## ROS2 Message Generation

### Using rosidl_generator_rs

```bash
# Install rosidl_generator_rs
sudo apt-get install ros-humble-rosidl-generator-rs

# Generate Rust bindings for custom messages
ros2 run rosidl_generator_rs generate_rust_bindings \
  --package-name my_custom_msgs \
  --output-dir src/generated
```

### Custom Message Example

```msg
# my_custom_msgs/msg/SensorData.msg
std_msgs/Header header
float32 temperature
float32 pressure
float32 humidity
```

```rust
// Generated Rust code
use my_custom_msgs::msg::SensorData;

let msg = SensorData {
    header: Header {
        stamp: Time::now(),
        frame_id: "sensor_frame".to_string(),
    },
    temperature: 25.0,
    pressure: 1013.25,
    humidity: 60.0,
};

publisher.publish(&msg)?;
```

---

## Debugging and Troubleshooting

### Common Issues

**1. rclrs not found**:
```bash
# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Check PKG_CONFIG_PATH
echo $PKG_CONFIG_PATH
# Should include /opt/ros/humble/lib/pkgconfig
```

**2. Cross-compilation linker errors**:
```bash
# Ensure sysroot is correct
export PKG_CONFIG_SYSROOT_DIR=/opt/ros2-arm-sysroot

# Check library paths
export LD_LIBRARY_PATH=/opt/ros2-arm-sysroot/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

**3. Async runtime panics**:
```rust
// Enable tokio console for debugging
tokio::runtime::Builder::new_multi_thread()
    .enable_all()
    .on_thread_start(|| {
        println!("Thread started");
    })
    .on_thread_stop(|| {
        println!("Thread stopped");
    })
    .build()?;
```

### Logging and Tracing

```rust
use tracing::{info, warn, error};
use tracing_subscriber;

// Initialize tracing
tracing_subscriber::fmt()
    .with_max_level(tracing::Level::DEBUG)
    .init();

// Use in code
info!("Driver initialized");
warn!("Sensor timeout");
error!("Hardware failure");
```

### Performance Profiling

```bash
# Profile with perf
perf record -g ./target/release/imu_driver
perf report

# Profile with flamegraph
cargo install flamegraph
cargo flamegraph --bin imu_driver

# Profile with valgrind
valgrind --tool=callgrind ./target/release/imu_driver
kcachegrind callgrind.out.*
```

---

## CI/CD Integration

### GitHub Actions Example

```yaml
name: Build and Test

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup ROS2
      uses: ros-tooling/setup-ros@v0.6
      with:
        required-ros-distributions: humble
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        override: true
    
    - name: Build
      run: |
        source /opt/ros/humble/setup.bash
        colcon build --packages-select drone_drivers_rust
    
    - name: Test
      run: |
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        colcon test --packages-select drone_drivers_rust
    
    - name: Benchmark
      run: |
        cargo bench --no-fail-fast
```

---

## References

- [rclrs Documentation](https://github.com/ros2-rust/ros2_rust)
- [ament_cargo Documentation](https://github.com/ros2-rust/ros2_rust/tree/main/ament_cargo)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Tokio Documentation](https://tokio.rs/)
- [Rust Embedded Book](https://rust-embedded.github.io/book/)
- [Cross-Compilation Guide](https://rust-lang.github.io/rustup/cross-compilation.html)
