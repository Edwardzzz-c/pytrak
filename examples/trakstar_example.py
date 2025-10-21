#!/usr/bin/env python3
"""
Trakstar Python Example

This script demonstrates how to use the Python wrapper for the Trakstar PointATC3DG
USB tracker. It provides examples of:
- Initializing the device
- Reading sensor data
- Configuring sensor parameters
- Continuous data acquisition

Usage:
    # Activate mamba environment first:
    mamba activate pytrak
    export PYTHONPATH=$(pwd)
    python examples/trakstar_example.py
    
    # Or use mamba run:
    mamba run -n pytrak bash -c "export PYTHONPATH=\$(pwd) && python examples/trakstar_example.py"

Author: Cheng Zhang <cz2874@columbia.edu>
"""

import sys
import os
import time

# Add current directory to Python path if not already there
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import numpy as np
import pytrak

def print_sensor_data(sensor_id, data, data_type="quaternion"):
    """Print formatted sensor data"""
    print(f"\n--- Sensor {sensor_id} Data ({data_type}) ---")
    if data["success"]:
        print(f"Position: x={data['x']:.6f}, y={data['y']:.6f}, z={data['z']:.6f}")
        
        if data_type == "quaternion" and "quaternion" in data:
            q = data["quaternion"]
            print(f"Quaternion: w={q[0]:.6f}, x={q[1]:.6f}, y={q[2]:.6f}, z={q[3]:.6f}")
            
        elif data_type == "angles" and "azimuth" in data:
            print(f"Angles: azimuth={data['azimuth']:.2f}°, elevation={data['elevation']:.2f}°, roll={data['roll']:.2f}°")
            
        elif data_type == "matrix" and "rotation_matrix" in data:
            matrix = np.array(data["rotation_matrix"]).reshape(3, 3)
            print("Rotation Matrix:")
            print(matrix)
    else:
        print("Failed to read sensor data")

def main():
    """Main example function"""
    print("Trakstar Python Wrapper Example")
    print("===============================")
    
    try:
        # Initialize the Trakstar device
        print("\n1. Initializing Trakstar device...")
        trakstar = pytrak.Trakstar()
        
        # Check if device is properly initialized
        if not trakstar.is_ok():
            print("ERROR: Failed to initialize Trakstar device!")
            print("Make sure the device is connected and you have proper permissions.")
            return 1
        
        print("✓ Device initialized successfully!")
        
        # Get device information
        print(f"\n2. Device Information:")
        print(f"Number of sensors: {trakstar.get_number_of_sensors()}")
        print(f"Transmitter attached: {trakstar.transmitter_attached()}")
        
        # Check each sensor
        num_sensors = trakstar.get_number_of_sensors()
        for i in range(num_sensors):
            attached = trakstar.sensor_attached(i)
            print(f"Sensor {i} attached: {attached}")
        
        if num_sensors == 0:
            print("No sensors detected. Please check your setup.")
            return 1
        
        # Configure sensors
        print(f"\n3. Configuring sensors...")
        
        # Set measurement rate (Hz)
        rate = 100.0
        result = trakstar.set_measurement_rate(rate)
        print(f"Set measurement rate to {rate} Hz: {'Success' if result == 0 else 'Failed'}")
        
        # Set maximum range (false = 36 inch, true = 72 inch)
        range_72inch = False
        result = trakstar.set_maximum_range(range_72inch)
        print(f"Set maximum range to {'72' if range_72inch else '36'} inch: {'Success' if result == 0 else 'Failed'}")
        
        # Configure each sensor
        for i in range(num_sensors):
            # Set hemisphere (false = front, true = back)
            hemisphere_back = False
            result = trakstar.set_sensor_hemisphere(i, hemisphere_back)
            print(f"Set sensor {i} hemisphere to {'rear' if hemisphere_back else 'front'}: {'Success' if result == 0 else 'Failed'}")
            
            # Set output format to quaternion
            result = trakstar.set_sensor_quaternion(i)
            print(f"Set sensor {i} to output quaternion: {'Success' if result == 0 else 'Failed'}")
        
        # Single measurement examples
        print(f"\n4. Single Measurements:")
        
        # Example 1: Get quaternion data from sensor 0
        if num_sensors > 0:
            print("\n--- Single Quaternion Reading ---")
            data = trakstar.get_coordinates_quaternion(0)
            print_sensor_data(0, data, "quaternion")
        
        # Example 2: Get Euler angles from sensor 0
        if num_sensors > 0:
            print("\n--- Single Euler Angles Reading ---")
            data = trakstar.get_coordinates_angles(0)
            print_sensor_data(0, data, "angles")
        
        # Example 3: Get rotation matrix from sensor 0
        if num_sensors > 0:
            print("\n--- Single Rotation Matrix Reading ---")
            data = trakstar.get_coordinates_matrix(0)
            print_sensor_data(0, data, "matrix")
        
        # Example 4: Get data from all sensors at once
        print("\n--- All Sensors Data ---")
        all_data = trakstar.get_all_sensors_data()
        if all_data["success"]:
            print(f"Number of sensors: {all_data['num_sensors']}")
            for sensor_data in all_data["sensors"]:
                sensor_id = sensor_data["sensor_id"]
                print(f"Sensor {sensor_id}: x={sensor_data['x']:.6f}, y={sensor_data['y']:.6f}, z={sensor_data['z']:.6f}")
        
        # Continuous data acquisition example
        print(f"\n5. Continuous Data Acquisition (5 seconds):")
        print("Press Ctrl+C to stop early")
        
        try:
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < 5.0:
                # Get data from all sensors
                all_data = trakstar.get_all_sensors_data()
                
                if all_data["success"]:
                    sample_count += 1
                    current_time = time.time() - start_time
                    
                    print(f"\nSample {sample_count} (t={current_time:.2f}s):")
                    for sensor_data in all_data["sensors"]:
                        sensor_id = sensor_data["sensor_id"]
                        x, y, z = sensor_data["x"], sensor_data["y"], sensor_data["z"]
                        print(f"  Sensor {sensor_id}: pos=({x:8.4f}, {y:8.4f}, {z:8.4f})")
                
                time.sleep(0.1)  # 10 Hz sampling
            
            print(f"\nCollected {sample_count} samples in 5 seconds")
            
        except KeyboardInterrupt:
            print("\nStopped by user")
        
        print("\n✓ Example completed successfully!")
        return 0
        
    except Exception as e:
        print(f"\nERROR: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
