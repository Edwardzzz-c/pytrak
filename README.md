---
trakstar_ros
---

ROS2 driver for trakstar magnetic tracker

**Note: This package has been migrated from ROS1 to ROS2**

---

### Instructions for installing first time\*:

1. make sure libusb-1.0-0-dev is installed

```
sudo apt-get install libusb-1.0-0-dev
```

2. copy contents of 'config/99-trakstar.rules' to /lib/udev/rules.d/99-libusb.rules:

- 2.1 cd into trakstar_ros/config and run:

```
sudo cp 99-trakstar.rules /lib/udev/rules.d/99-libusb.rules
```

- 2.2 **reboot**.

3. If libusb is still giving you issues, see notes at bottom of https://github.com/ChristophJud/ATC3DGTracker readme.

- 3.1 run command 'rmmod ehci_hcd'.
- 3.2 compile demo script from the installation instructions in https://github.com/ChristophJud/ATC3DGTracker

4. Build the package in your ROS2 workspace:

```
cd ~/your_ros2_ws
colcon build --packages-select trakstar
source install/setup.bash
```

note: usually steps 1 and 2 are enough

---

### Instructions to run (ROS2):

(make sure trakstar power is on and status light is green; blinking is fine)

```
ros2 launch trakstar trakstar.launch.py
```

you're done! Transforms should be published on topic /tf and you can view them in rviz2.
Transforms are trakstar# (sensor number) relative to trakstar_base (transmitter).

### Recording data with ROS2 bag:

```
ros2 bag record /tf /trakstar_msg /trakstar_raw_msg
```

### Replaying a saved bag:

```
ros2 bag play your_bag_name
```

### Converting bag to CSV:

In ROS2, you can use `ros2 bag` command line tools or Python scripts to extract data from bags and convert to CSV.

---

## ROS2 Migration Notes (2025)

This package has been migrated from ROS1 (catkin) to ROS2 (ament_cmake). Key changes:

### Major Changes:

- **Build system**: Migrated from catkin to ament_cmake
- **C++ node**: Converted from `ros::NodeHandle` to `rclcpp::Node`
- **Python nodes**: Converted from `rospy` to `rclpy`
- **Launch files**: Converted from XML to Python launch files
- **Messages**: Message generation now uses `rosidl_default_generators`
- **TF**: Migrated from `tf` to `tf2` and `tf2_ros`

### File Changes:

- `package.xml`: Updated to format 3 with ROS2 dependencies
- `CMakeLists.txt`: Complete rewrite for ament_cmake
- `nodes/trakstar_node.cpp`: Rewritten using rclcpp
- `nodes/*.py`: Updated to use rclpy instead of rospy
- `launch/*.launch`: Converted to Python launch files (`.launch.py`)
- `setup.py`: Removed (not needed for ament_cmake packages)

### Dependencies:

- `rclcpp` (replaces roscpp)
- `rclpy` (replaces rospy)
- `tf2` and `tf2_ros` (replaces tf)
- `geometry_msgs`, `std_msgs` (same names, ROS2 versions)

### Python Dependencies:

Make sure you have these Python packages installed:

```
pip3 install transforms3d numpy scikit-learn
```

---

## Historical Notes (ROS1):

Latest (11/17/2023, KSL): we were able to run on katelyn's machine! Does require configuring libusb to work.

Latest (02/12/2024, JP): we ran it in the bimanual manipulation machine. It required steps 3.1 and 3.2.

Latest (03/30/2025, RW): Setup possible for ROS within virtual env (Robostack Mamba)

```
conda install -c conda-forge libusb-compat
```

When building within virtual environments, install libusb-compat via conda (mamba preferred) and do NOT install any libusb packages via apt. They conflict with one another, and virtual environments don't play nice with apt packages. At the time of installation, the latest version on conda-forge is 0.1.12.

Note: the trakstar driver code was written over 10 years ago. libusb was a much earlier version (0.1.\*) and some methods have been removed/renamed/deprecated. usb_close() was one of them (renamed to libusb_close). The reason why we needed to install libusb-dev was because it contained previous version of the package (this is my educated guess based on reading up related errors).

Note: I did not run into any permission issues. Not sure if related but: when I first set up the current machine, I had to deal with a usb permission issue (ttyACM0 Permission Error). The solution was to add the current user to the dialout group to obtain permission for port access. If you run into permission errors, ask RW for detail on how to create dialout groups

---

## Troubleshooting:

### Build errors with libusb:

Make sure you have libusb-1.0-0-dev installed and the udev rules are properly configured.

### TF not publishing:

Make sure the `publish_tf` parameter is set to `true` in the launch file.

### Python node errors:

Ensure transforms3d, numpy, and scikit-learn are installed in your Python environment.
