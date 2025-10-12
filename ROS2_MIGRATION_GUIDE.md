# ROS1 to ROS2 Migration Guide for Trakstar Package

This document describes the migration of the trakstar package from ROS1 (Noetic) to ROS2.

## Summary of Changes

### Package Configuration

#### package.xml

- Updated to format 3
- Changed `catkin` to `ament_cmake` and `rosidl_default_generators`
- Replaced `roscpp` with `rclcpp`
- Added `rclpy` for Python nodes
- Changed `tf` to `tf2` and `tf2_ros`
- Updated libusb dependencies to use ROS2 package names
- Changed `<run_depend>` to `<exec_depend>`
- Changed `<build_depend>` to `<depend>` for most packages

#### CMakeLists.txt

- Updated minimum CMake version to 3.8
- Replaced `find_package(catkin ...)` with individual `find_package()` calls
- Changed message generation from `add_message_files()` + `generate_messages()` to `rosidl_generate_interfaces()`
- Replaced `catkin_package()` with `ament_package()` (at end of file)
- Changed installation paths from `${CATKIN_PACKAGE_*}` to proper ROS2 paths
- Updated library linking to use `ament_target_dependencies()`
- Added `rosidl_get_typesupport_target()` for linking against generated messages

### C++ Code Changes

#### nodes/trakstar_node.cpp

**Major changes:**

- Converted from standalone executable to a class inheriting from `rclcpp::Node`
- Changed `ros::init()` to `rclcpp::init()`
- Changed `ros::NodeHandle` to class member functions
- Changed `ros::Publisher` to `rclcpp::Publisher<T>::SharedPtr`
- Changed `ros::Rate` to `rclcpp::WallTimer`
- Changed parameter API:
  - `n_private.param<T>()` → `this->declare_parameter<T>()` + `this->get_parameter().as_T()`
- Changed logging:
  - `ROS_INFO()` → `RCLCPP_INFO(this->get_logger(), ...)`
  - `ROS_ERROR()` → `RCLCPP_ERROR(this->get_logger(), ...)`
- Changed time API:
  - `ros::Time::now()` → `this->now()`
- Changed message includes:
  - `"trakstar/TrakstarMsg.h"` → `"trakstar/msg/trakstar_msg.hpp"`
- Changed TF:
  - `#include "tf/tf.h"` → `#include "tf2/..."` headers
  - `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
  - `tf::transformTFToMsg()` → `tf2::toMsg()`
- Changed main loop from `ros::spin()` + callbacks to timer-based callbacks
- Used smart pointers (`std::unique_ptr`, `std::shared_ptr`) throughout

### Python Code Changes

#### All Python Nodes

**Major changes:**

- Changed shebang from `#!/usr/bin/env python` to `#!/usr/bin/env python3`
- Changed `import rospy` to `import rclpy` and `from rclpy.node import Node`
- Changed from functional to class-based approach (inherit from `Node`)
- Changed `rospy.init_node()` to `rclpy.init()` + creating a node class
- Changed logging:
  - `rospy.loginfo()` → `self.get_logger().info()`
  - `rospy.logerr()` → `self.get_logger().error()`
- Changed timers:
  - `rospy.Timer()` → `self.create_timer()`
- Changed publishers:
  - `rospy.Publisher()` → `self.create_publisher()`
- Changed TF2:
  - `tf2_ros.Buffer()` + `tf2_ros.TransformListener()` - similar API
  - `rospy.Time()` → `Time()` (from `rclpy.time`)
  - `rospy.Duration()` → `Duration()` (from `rclpy.duration`)
- Changed spin:
  - `rospy.spin()` → `rclpy.spin(node)`
- Changed shutdown handling:
  - `rospy.is_shutdown()` → `rclpy.ok()`
  - Added proper cleanup with `node.destroy_node()` and `rclpy.shutdown()`

### Launch File Changes

#### All Launch Files

- Converted from XML (`.launch`) to Python (`.launch.py`)
- Structure changes:
  - XML `<launch>` tags → Python function `generate_launch_description()`
  - `<arg>` tags → `DeclareLaunchArgument()`
  - `<node>` tags → `Node()` actions
  - `$(arg name)` → `LaunchConfiguration('name')`
- Parameter passing:
  - XML `<param>` tags → `parameters=[{...}]` dictionary in Node()
- Removed ROS1-specific features like dynamic Python evaluation in args

### Message Definitions

#### msg/TrakstarMsg.msg

- Message definition remains the same (compatible format)
- Generation method changed (handled by `rosidl_generate_interfaces`)

## Build and Run Instructions

### Building

```bash
cd ~/your_ros2_ws
colcon build --packages-select trakstar
source install/setup.bash
```

### Running

```bash
# Launch the trakstar driver
ros2 launch trakstar trakstar.launch.py

# With custom parameters
ros2 launch trakstar trakstar.launch.py publish_tf:=true frequency:=80

# View topics
ros2 topic list
ros2 topic echo /trakstar_msg

# View TF tree
ros2 run tf2_tools view_frames
```

### Testing

```bash
# Check if node is running
ros2 node list

# Check node info
ros2 node info /trakstar_driver

# Monitor TF
ros2 run tf2_ros tf2_echo trakstar_base trakstar0
```

## API Changes Summary

| ROS1                         | ROS2                                    |
| ---------------------------- | --------------------------------------- |
| `ros::init()`                | `rclcpp::init()`                        |
| `ros::NodeHandle`            | `rclcpp::Node`                          |
| `ros::Publisher<T>`          | `rclcpp::Publisher<T>::SharedPtr`       |
| `ros::Subscriber<T>`         | `rclcpp::Subscription<T>::SharedPtr`    |
| `ros::Rate`                  | `rclcpp::Rate` or `create_wall_timer()` |
| `ros::Time::now()`           | `node->now()` or `this->now()`          |
| `ROS_INFO()`                 | `RCLCPP_INFO(logger, ...)`              |
| `tf::TransformBroadcaster`   | `tf2_ros::TransformBroadcaster`         |
| `rospy`                      | `rclpy`                                 |
| `.launch` (XML)              | `.launch.py` (Python)                   |
| `catkin_make`/`catkin build` | `colcon build`                          |

## Dependencies

### System Dependencies

```bash
sudo apt-get install libusb-1.0-0-dev
```

### Python Dependencies

```bash
pip3 install transforms3d numpy scikit-learn
```

### ROS2 Dependencies

All specified in `package.xml`:

- rclcpp
- rclpy
- geometry_msgs
- std_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs

## Known Issues and Notes

1. **Timer-based publishing**: ROS2 uses timer-based callbacks instead of spin-once loops. The frequency parameter controls the timer period.

2. **TF2**: All TF operations now use TF2. The API is mostly compatible but some function names have changed.

3. **Parameter declaration**: ROS2 requires parameters to be declared before use. All parameters are declared in the node constructor.

4. **Python Rate**: ROS2's `create_rate()` works differently than ROS1. Consider using timers for periodic tasks.

5. **Launch file timestamps**: The datetime eval trick from ROS1 launch files has been replaced with Python's `datetime` module directly.

6. **Message generation**: Messages are now generated at build time and installed to the `install/` directory. Include paths have changed to use the ROS2 convention (`package/msg/message.hpp`).

## Testing Checklist

- [ ] Package builds without errors
- [ ] Trakstar node starts and initializes hardware
- [ ] Messages are published on `/trakstar_msg` and `/trakstar_raw_msg`
- [ ] TF transforms are published (when `publish_tf:=true`)
- [ ] Python calibration nodes work
- [ ] Launch files work with parameters
- [ ] Multiple sensors are detected and published

## Future Improvements

- Consider using ROS2 lifecycle nodes for better hardware management
- Add ROS2 parameter callback for dynamic reconfiguration
- Implement proper component-based architecture
- Add ROS2 actions for calibration procedures
- Consider using ros2_control for sensor management

