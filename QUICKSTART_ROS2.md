# Quick Start Guide - ROS2 Trakstar Package

This guide will help you quickly get started with the ROS2 version of the trakstar package.

## Prerequisites

### 1. System Setup

```bash
# Install libusb
sudo apt-get update
sudo apt-get install libusb-1.0-0-dev

# Install udev rules
cd ~/your_ros2_ws/src/trakstar_ros
sudo cp config/99-trakstar.rules /lib/udev/rules.d/99-libusb.rules

# Reboot to apply udev rules
sudo reboot
```

### 2. Python Dependencies

```bash
# Install Python packages
pip3 install -r requirements.txt

# Or install individually:
pip3 install transforms3d scikit-learn numpy pandas matplotlib
```

### 3. ROS2 Setup

Make sure you have ROS2 installed (tested with Humble/Iron/Jazzy).

```bash
source /opt/ros/<your-distro>/setup.bash
```

## Build

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_ws

# Build the package
colcon build --packages-select trakstar

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

1. **Connect the Trakstar hardware**

   - Power on the Trakstar transmitter
   - Ensure status light is green (blinking is OK)
   - Connect sensors

2. **Launch the driver**

   ```bash
   ros2 launch trakstar trakstar.launch.py
   ```

3. **Verify it's working**

   ```bash
   # Check if node is running
   ros2 node list

   # View published messages
   ros2 topic echo /trakstar_msg

   # View TF transforms (if publish_tf:=true)
   ros2 run tf2_ros tf2_echo trakstar_base trakstar0
   ```

### Launch with Custom Parameters

```bash
# Enable TF publishing
ros2 launch trakstar trakstar.launch.py publish_tf:=true

# Change frequency
ros2 launch trakstar trakstar.launch.py frequency:=100

# Set sensor attachment offsets
ros2 launch trakstar trakstar.launch.py px:=0.01 py:=0.02 pz:=0.03

# Use back hemisphere
ros2 launch trakstar trakstar.launch.py hemisphere_back:=true

# Enable 72 inch range
ros2 launch trakstar trakstar.launch.py range_72inch:=true
```

### Available Launch Files

1. **trakstar.launch.py** - Basic trakstar driver
2. **ascension.launch.py** - Ascension-specific configuration
3. **trakstar_futek.launch.py** - Configuration with Futek sensor integration

### Recording Data

```bash
# Record all trakstar topics
ros2 bag record /trakstar_msg /trakstar_raw_msg /tf

# Record with a specific name
ros2 bag record -o my_trakstar_data /trakstar_msg /trakstar_raw_msg

# Play back recorded data
ros2 bag play my_trakstar_data
```

### Visualization in RViz2

```bash
# Launch RViz2
rviz2

# Add TF display
# Set Fixed Frame to: trakstar_base
# Add -> TF -> OK
```

## Calibration Workflow

### 1. Collect Calibration Data

```bash
# Run the calibration data collection node
ros2 run trakstar calibration_data_collection.py
```

This will:

- Record 10 neutral pose transforms
- Record 40 sweeping motion transforms
- Save to `calibration_poses` file

### 2. Process Calibration Data

```bash
# Run the mounting calibration script
ros2 run trakstar trakstar_mounting_calibration.py --file calibration_poses --outfile calibration_info
```

This will:

- Compute joint axis using PCA
- Save calibration info to `calibration_info` file

### 3. Publish Calibrated Data

```bash
# Run the calibrated data publisher
ros2 run trakstar publish_calibrated_data.py
```

This will:

- Load calibration info
- Compute and publish joint angles to `/calibrated_joint_angle`

## Troubleshooting

### "Can't open trakstar" Error

- Check if device is powered on
- Verify USB connection
- Check udev rules: `ls -l /dev/bus/usb/`
- Try running with sudo (not recommended for production)
- Reboot after installing udev rules

### "At least 2 trackers required" Error

- Ensure sensors are connected to the transmitter
- Check sensor LED indicators
- Try repowering the device

### Build Errors with libusb

```bash
# Make sure libusb-1.0 is installed
sudo apt-get install libusb-1.0-0-dev pkg-config

# Clean and rebuild
cd ~/your_ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select trakstar
```

### Python Import Errors

```bash
# Install missing Python packages
pip3 install transforms3d scikit-learn numpy

# Or use the requirements file
pip3 install -r requirements.txt
```

### TF Not Publishing

- Make sure `publish_tf:=true` is set in launch file
- Check if the trakstar_driver node is running: `ros2 node list`
- Monitor TF topics: `ros2 topic echo /tf`

## Topics and Messages

### Published Topics

- `/trakstar_msg` (trakstar/msg/TrakstarMsg) - Calibrated sensor transforms
- `/trakstar_raw_msg` (trakstar/msg/TrakstarMsg) - Raw sensor transforms
- `/tf` (tf2_msgs/TFMessage) - TF transforms (if publish_tf is enabled)

### Message Format

```
std_msgs/Header header
geometry_msgs/Transform[4] transform
uint8 n_tracker
uint8 LEFT=0
uint8 RIGHT=1
```

## Parameters

| Parameter                  | Type   | Default | Description                 |
| -------------------------- | ------ | ------- | --------------------------- |
| `publish_tf`               | bool   | false   | Publish transforms to /tf   |
| `frequency`                | int    | 255     | Measurement rate in Hz      |
| `hemisphere_back`          | bool   | false   | Use back hemisphere         |
| `range_72inch`             | bool   | false   | Use 72 inch range           |
| `pivot_x/y/z`              | double | 0.0     | Sensor 0 pivot offset       |
| `attach_roll/pitch/yaw`    | double | 0.0     | Sensor 0 orientation offset |
| `pivot_x1/y1/z1`           | double | 0.0     | Sensor 1 pivot offset       |
| `attach_roll1/pitch1/yaw1` | double | 0.0     | Sensor 1 orientation offset |

## Testing

```bash
# Run the standalone test tool (no ROS)
cd ~/your_ros2_ws/install/trakstar/lib/trakstar
./trakstar_print

# This will print sensor data to console
# Press any key to stop
```

## Additional Resources

- [ROS2 Migration Guide](ROS2_MIGRATION_GUIDE.md) - Detailed migration documentation
- [Original Repository](https://github.com/seanyun/trakstar_ros) - ROS1 version
- [Trakstar Hardware Docs](https://github.com/ChristophJud/ATC3DGTracker) - Hardware driver information

## Support

For issues specific to the ROS2 migration, check:

1. All dependencies are installed
2. Udev rules are properly configured
3. Hardware is functioning (test with `trakstar_print`)
4. ROS2 environment is sourced

For hardware-specific issues, refer to the Ascension Trakstar documentation.

