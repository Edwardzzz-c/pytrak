# Trakstar Python Wrapper

A Python wrapper for the Trakstar PointATC3DG USB tracker without ROS2 dependencies.

## Features

- Direct access to Trakstar hardware without ROS2
- Support for multiple sensors
- Multiple data formats: quaternion, Euler angles, rotation matrix
- Real-time data acquisition
- Easy-to-use Python API
- No ROS2 dependencies
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

trakstar = pytrak.Trakstar()
if trakstar.is_ok():
    data = trakstar.get_coordinates_quaternion(0)
    print(f"Position: {data['x']}, {data['y']}, {data['z']}")
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

## Coordinate System

Right-handed coordinate system:
- X-axis: Forward/backward
- Y-axis: Left/right  
- Z-axis: Up/down

Position units are in meters.
