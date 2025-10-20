# Pytrak (Python Trakstar Wrapper)

A Python wrapper for the Trakstar PointATC3DG USB tracker based on:
- https://github.com/ChristophJud/ATC3DGTracker (Original)
- https://github.com/seanyun/trakstar_ros (Version compatible with hardware owned by ROAM lab)
- https://github.com/joaquin-ps/trakstar_ros2 (Same implementation as Pytrak in ROS2)

## Features

- Direct access to Trakstar hardware with Python API
- Support for multiple sensors
- Multiple data formats: quaternion, Euler angles, rotation matrix
- Docker support for isolated builds

## Quick Start

### Option 1: Docker (Recommended)
```bash
# Build and start container
./build_docker.sh

# Access container
docker exec -it pytrak_container /bin/bash

# Run examples
python3 examples/simple_example.py
python3 examples/trakstar_example.py
```

### Option 2: Native Build
```bash
# Build (requires sudo for dependencies)
./build.sh

# Run examples
python3 examples/simple_example.py
python3 examples/trakstar_example.py
```

### Usage
```python
import pytrak

# Initialize device
trakstar = pytrak.Trakstar()
if trakstar.is_ok():
    print(f"Number of sensors: {trakstar.get_number_of_sensors()}")
    
    # Get data from single sensor
    data = trakstar.get_coordinates_quaternion(0)
    if data["success"]:
        x, y, z = data["x"], data["y"], data["z"]
        quat = data["quaternion"]  # [w, x, y, z]
        print(f"Sensor 0: pos=({x:.3f}, {y:.3f}, {z:.3f}) quat=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})")
    
    # Get data from all sensors at once
    all_data = trakstar.get_all_sensors_data()
    if all_data["success"]:
        for sensor_data in all_data["sensors"]:
            sensor_id = sensor_data["sensor_id"]
            x, y, z = sensor_data["x"], sensor_data["y"], sensor_data["z"]
            quat = sensor_data["quaternion"]
            print(f"Sensor {sensor_id}: pos=({x:.3f}, {y:.3f}, {z:.3f}) quat=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})")
```

## Examples

- `examples/simple_example.py` - Basic usage with continuous reading (Ctrl+C to stop)
- `examples/trakstar_example.py` - Comprehensive example with all features

## API Reference

### Trakstar Class

- `is_ok()` - Check if device is initialized
- `get_number_of_sensors()` - Get number of connected sensors
- `get_coordinates_quaternion(sensor_id)` - Get position and quaternion
- `get_coordinates_angles(sensor_id)` - Get position and Euler angles
- `get_coordinates_matrix(sensor_id)` - Get position and rotation matrix
- `get_all_sensors_data()` - Get data from all sensors at once
- `set_measurement_rate(rate)` - Set measurement frequency
- `set_maximum_range(range_72inch)` - Set measurement range

## Troubleshooting

- **Device not found**: Check USB permissions and device connection
- **Import error**: Make sure the module is in your Python path
- **Permission denied**: Add your user to the dialout group
