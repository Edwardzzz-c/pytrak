# Changelog

## [2.0.0] - 2025-10-12 - ROS2 Migration

### Major Changes

- **Complete migration from ROS1 to ROS2**
- Package now requires ROS2 (Humble, Iron, or Jazzy recommended)
- Build system changed from catkin to ament_cmake
- All code updated to use ROS2 APIs

### Added

- `ROS2_MIGRATION_GUIDE.md` - Comprehensive migration documentation
- `QUICKSTART_ROS2.md` - Quick start guide for ROS2 users
- `CHANGELOG.md` - This file
- Python launch files (`.launch.py`) for all launch configurations
- Explicit parameter declarations in C++ node
- Timer-based main loop in C++ node for better control
- Smart pointer usage throughout C++ code
- Enhanced error handling and exceptions

### Changed

- `package.xml` - Updated to format 3 with ROS2 dependencies
- `CMakeLists.txt` - Complete rewrite for ament_cmake
- `src/CMakeLists.txt` - Updated for ROS2 build system
- `nodes/CMakeLists.txt` - Updated for ROS2 executable installation
- `tools/CMakeLists.txt` - Updated for ROS2
- `nodes/trakstar_node.cpp` - Converted to use rclcpp Node class
  - Changed from functional to OOP approach
  - Uses rclcpp::Node as base class
  - Timer-based publishing instead of rate-based loop
  - Updated all ROS API calls to ROS2 equivalents
  - Changed TF from tf to tf2
  - Updated message includes and namespaces
- `nodes/publish_calibrated_data.py` - Converted from rospy to rclpy
  - Node now inherits from rclpy.Node
  - Updated all logging calls
  - Updated TF2 API usage
- `nodes/calibration_data_collection.py` - Converted from rospy to rclpy
  - Node now inherits from rclpy.Node
  - Updated all logging calls
  - Updated rate and timing APIs
- `nodes/trakstar_mounting_calibration.py` - Updated shebang to Python 3
- `README.md` - Updated with ROS2 instructions and migration notes
- `requirements.txt` - Added transforms3d and scikit-learn dependencies
- All Python shebangs changed from `python` to `python3`

### Removed

- `setup.py` - Not needed for ament_cmake packages
- Old ROS1 XML launch files:
  - `launch/trakstar.launch` (replaced with `.launch.py`)
  - `launch/ascension.launch` (replaced with `.launch.py`)
  - `launch/trakstar_futek.launch` (replaced with `.launch.py`)

### Migration Details

#### API Changes

- ROS C++ API: `ros` → `rclcpp`
- ROS Python API: `rospy` → `rclpy`
- TF: `tf` → `tf2` and `tf2_ros`
- Messages: `package/MessageName.h` → `package/msg/message_name.hpp`
- Build: `catkin_make` → `colcon build`
- Launch: XML → Python

#### Message Generation

- Changed from `add_message_files()` + `generate_messages()` to `rosidl_generate_interfaces()`
- Messages now installed in standard ROS2 locations

#### Parameter Handling

- Parameters must be declared before use
- Changed from `getParam()` to `declare_parameter()` + `get_parameter()`
- Launch files use declarative parameter syntax

#### Logging

- C++: `ROS_INFO()` → `RCLCPP_INFO(logger, ...)`
- Python: `rospy.loginfo()` → `self.get_logger().info()`

#### Time

- C++: `ros::Time::now()` → `this->now()`
- Python: `rospy.Time()` → `Time()` from `rclpy.time`

### Dependencies

Added explicit dependencies on:

- rclcpp (replacing roscpp)
- rclpy (replacing rospy)
- tf2, tf2_ros, tf2_geometry_msgs (replacing tf)
- rosidl_default_generators (for message generation)
- ament_cmake (replacing catkin)

### Backward Compatibility

- **Not backward compatible with ROS1**
- Message definitions remain compatible
- Hardware interface unchanged

### Testing Status

- [x] Package builds without errors
- [x] No linter errors
- [ ] Runtime testing with hardware (requires Trakstar device)
- [ ] Calibration workflow testing
- [ ] Multi-sensor testing

### Known Issues

None at this time. Hardware testing pending.

### Migration Notes

For users migrating from ROS1:

1. See `ROS2_MIGRATION_GUIDE.md` for detailed API changes
2. See `QUICKSTART_ROS2.md` for quick setup
3. Update any custom scripts that interact with this package
4. Launch file syntax has completely changed - see examples
5. Topic names and message types remain the same

### Contributors

- Original ROS1 package: Sean Seungkook Yun, Ava Chen
- ROS2 migration: Cheng Zhang (2025)

---

## [0.0.1] - ROS1 Version

Initial ROS1 implementation supporting Trakstar magnetic tracker with:

- Multi-sensor support
- TF broadcasting
- Calibration utilities
- Python analysis tools

