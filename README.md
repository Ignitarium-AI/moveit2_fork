# MoveIt 2 - Planning Scene Monitor & Occupancy Map Monitor

## Build and Unit Test Guide

This guide provides detailed instructions for building and testing the **Planning Scene Monitor** and **Occupancy Map Monitor** components of MoveIt 2.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Workspace Setup](#workspace-setup)
- [Building the Packages](#building-the-packages)
- [Running Unit Tests](#running-unit-tests)
- [Test Descriptions](#test-descriptions)
- [Advanced Build Options](#advanced-build-options)
- [Troubleshooting](#troubleshooting)

---

## Overview

### Planning Scene Monitor (`moveit_ros_planning`)

The **Planning Scene Monitor** is a critical component responsible for:
- Maintaining an up-to-date representation of the robot's environment
- Monitoring robot state through the `CurrentStateMonitor`
- Tracking trajectory execution via the `TrajectoryMonitor`
- Integrating occupancy map updates into the planning scene
- Publishing planning scene updates to ROS topics

**Location**: `moveit_ros/planning/planning_scene_monitor/`

**Key Classes**:
- `PlanningSceneMonitor` - Core monitor class
- `CurrentStateMonitor` - Monitors robot joint states
- `TrajectoryMonitor` - Tracks executed trajectories

### Occupancy Map Monitor (`moveit_ros_occupancy_map_monitor`)

The **Occupancy Map Monitor** manages 3D occupancy grid representations:
- Creates and updates octree-based occupancy maps (using OctoMap)
- Processes point cloud data from depth sensors
- Integrates with the planning scene for collision checking
- Provides pluggable updater architecture for different sensor types

**Location**: `moveit_ros/occupancy_map_monitor/`

**Key Classes**:
- `OccupancyMapMonitor` - Main occupancy map management
- `OccupancyMapUpdater` - Base class for sensor-specific updaters

---

## Prerequisites

### System Requirements

- **OS**: Ubuntu 22.04 (Jammy), Ubuntu 24.04 (Noble), or later
- **ROS 2 Distribution**: Humble, Jazzy, or Rolling
- **Build System**: colcon
- **Compiler**: GCC 11+ or Clang 14+ (with C++17 support)

### Required Dependencies

Install ROS 2 and build tools:

```bash
# Install ROS 2 (example for Humble on Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep

# Install MoveIt dependencies
sudo apt install \
  ros-humble-moveit-core \
  ros-humble-moveit-msgs \
  ros-humble-geometric-shapes \
  ros-humble-rclcpp \
  ros-humble-tf2-ros \
  liboctomap-dev \
  libeigen3-dev \
  libboost-all-dev
```

### Test Dependencies

```bash
sudo apt install \
  ros-humble-ament-cmake-gtest \
  ros-humble-ament-cmake-gmock \
  ros-humble-ros-testing \
  ros-humble-launch-testing-ament-cmake
```

---

## Workspace Setup

### 1. Create a Colcon Workspace

```bash
mkdir -p ~/moveit2_ws/src
cd ~/moveit2_ws/src
```

### 2. Clone MoveIt 2 Repository

```bash
git clone https://github.com/moveit/moveit2_fork.git -b main
```

### 3. Clone Additional Dependencies

```bash
# Clone moveit_msgs if not already present
git clone https://github.com/moveit/moveit_msgs.git -b main

# Clone moveit_resources for test robot models
git clone https://github.com/moveit/moveit_resources.git -b main
```

### 4. Install Dependencies with rosdep

```bash
cd ~/moveit2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Building the Packages

### Build All MoveIt Packages

```bash
cd ~/moveit2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build Only Occupancy Map Monitor

```bash
colcon build --packages-select moveit_ros_occupancy_map_monitor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build Only Planning Scene Monitor

Since the Planning Scene Monitor is part of `moveit_ros_planning`, build it with:

```bash
colcon build --packages-select moveit_ros_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build with Tests Enabled

By default, tests are built. To explicitly ensure they're included:

```bash
colcon build --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
```

### Build with Debug Symbols

For development and debugging:

```bash
colcon build --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Rebuild from Scratch

```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild
colcon build --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning
```

---

## Running Unit Tests

### Source the Workspace

Always source your workspace before running tests:

```bash
cd ~/moveit2_ws
source install/setup.bash
```

### Run All Tests for Both Packages

```bash
colcon test --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning
```

### View Test Results

```bash
colcon test-result --all --verbose
```

### Run Tests with Detailed Output

```bash
colcon test --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning \
  --event-handlers console_direct+
```

### Run Specific Test Suites

#### Occupancy Map Monitor Tests

```bash
# Run the test executable directly
./build/moveit_ros_occupancy_map_monitor/occupancy_map_monitor_tests

# Or with colcon
colcon test --packages-select moveit_ros_occupancy_map_monitor \
  --ctest-args tests occupancy_map_monitor_tests
```

#### Planning Scene Monitor Tests

The planning scene monitor has multiple test executables:

```bash
# Current State Monitor tests
./build/moveit_ros_planning/planning_scene_monitor/current_state_monitor_tests

# Trajectory Monitor tests
./build/moveit_ros_planning/planning_scene_monitor/trajectory_monitor_tests

# Planning Scene Monitor integration tests (launch-based)
colcon test --packages-select moveit_ros_planning \
  --ctest-args tests planning_scene_monitor_test
```

### Run Tests with Coverage

```bash
colcon build --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"

colcon test --packages-select moveit_ros_occupancy_map_monitor moveit_ros_planning

# Generate coverage report (requires lcov)
lcov --capture --directory build/ --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html
```

### Run Individual Test Cases

```bash
# List available tests
./build/moveit_ros_occupancy_map_monitor/occupancy_map_monitor_tests --gtest_list_tests

# Run specific test
./build/moveit_ros_occupancy_map_monitor/occupancy_map_monitor_tests \
  --gtest_filter=OccupancyMapMonitorTests.TestConstruction
```

---

## Test Descriptions

### Occupancy Map Monitor Tests

**Test File**: `moveit_ros/occupancy_map_monitor/test/occupancy_map_monitor_tests.cpp`

- **Test Suite**: `OccupancyMapMonitorTests`
  - `TestConstruction` - Validates proper initialization of OccupancyMapMonitor
  - `TestStartStop` - Tests monitor start/stop lifecycle
  - `TestOctomapCreation` - Verifies octree map creation
  - `TestMessageProcessing` - Tests point cloud message handling

**Test Framework**: Google Mock (GMock)

**What is Tested**:
- Proper construction with ROS node and TF buffer
- Monitor lifecycle (start, stop, reset)
- Octree initialization and parameters
- Thread safety and concurrent access
- Memory management

### Planning Scene Monitor Tests

#### Current State Monitor Tests

**Test File**: `moveit_ros/planning/planning_scene_monitor/test/current_state_monitor_tests.cpp`

- **Test Suite**: `CurrentStateMonitorTests`
  - `TestStateMonitorStartStop` - Monitor lifecycle
  - `TestWaitForCurrentState` - Timeout and state availability
  - `TestStateUpdate` - Joint state updates from topics
  - `TestBoundsEnforcement` - Joint limit checking

**What is Tested**:
- Joint state subscription and processing
- State update callbacks
- Waiting for complete robot state
- Joint bounds enforcement
- TF integration for multi-robot setups

#### Trajectory Monitor Tests

**Test File**: `moveit_ros/planning/planning_scene_monitor/test/trajectory_monitor_tests.cpp`

- **Test Suite**: `TrajectoryMonitorTests`
  - `TestTrajectoryRecording` - Recording executed trajectories
  - `TestSampling` - Trajectory sampling at different rates
  - `TestOverwrite` - Handling overlapping trajectories

**What is Tested**:
- Trajectory recording from execution
- Sampling and interpolation
- Multi-DOF trajectory handling
- Timestamp synchronization

#### Planning Scene Monitor Integration Tests

**Test File**: `moveit_ros/planning/planning_scene_monitor/test/planning_scene_monitor_test.cpp`
**Launch File**: `moveit_ros/planning/planning_scene_monitor/test/launch/planning_scene_monitor.test.py`

- **Integration Tests** (require ROS nodes)
  - End-to-end planning scene updates
  - Collision object integration
  - Attached object handling
  - Scene diff publishing

**What is Tested**:
- Full planning scene monitor initialization
- Integration with robot model loader
- Scene update publishing and subscription
- Collision world synchronization

---

## Troubleshooting

### Build Errors

#### Missing Dependencies

```bash
# Error: Could not find liboctomap
sudo apt install liboctomap-dev

# Error: moveit_core not found
rosdep install --from-paths src --ignore-src -r -y
```

#### Compiler Errors

```bash
# Ensure correct compiler version
gcc --version  # Should be 11 or higher

# Try cleaning and rebuilding
rm -rf build/ install/
colcon build
```

### Test Failures

#### Tests Timeout

```bash
# Increase test timeout
colcon test --packages-select moveit_ros_planning \
  --pytest-args --timeout=120
```

#### Missing Test Resources

```bash
# Ensure test robot models are available
colcon build --packages-select moveit_resources_panda_moveit_config
```

#### ROS Communication Issues

```bash
# Check ROS_DOMAIN_ID if tests interfere with running systems
export ROS_DOMAIN_ID=42
colcon test
```

### Runtime Issues

#### TF Errors

```bash
# Planning scene monitor requires TF
# Ensure robot state publisher is running in tests
```

#### Occupancy Map Not Updating

```bash
# Check point cloud topic subscriptions
ros2 topic list
ros2 topic echo /camera/depth/points
```

### Performance Issues

```bash
# Use release builds for performance testing
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check for unnecessary logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
```

---

## Additional Resources

- **MoveIt 2 Documentation**: https://moveit.picknik.ai/
- **API Documentation**: Generate with `doxygen Doxyfile`
- **Tutorials**: https://moveit.picknik.ai/main/doc/tutorials/tutorials.html
- **GitHub Repository**: https://github.com/moveit/moveit2
- **Issue Tracker**: https://github.com/moveit/moveit2/issues

---

## License

MoveIt 2 is licensed under the BSD-3-Clause License. See LICENSE.txt for details.

---


