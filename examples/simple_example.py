#!/usr/bin/env python3
"""
Simple Trakstar Python Example

This is a minimal example showing basic usage of the Trakstar Python wrapper.

Usage:
    # Activate mamba environment first:
    mamba activate pytrak
    export PYTHONPATH=$(pwd)
    python examples/simple_example.py
    
    # Or use mamba run:
    mamba run -n pytrak bash -c "export PYTHONPATH=\$(pwd) && python examples/simple_example.py"
"""

import sys
import os

# Add current directory to Python path if not already there
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import pytrak
import time

def main():
    # Initialize the device
    trakstar = pytrak.Trakstar()
    
    if not trakstar.is_ok():
        print("Failed to initialize Trakstar device")
        print("Make sure the device is connected and you have proper permissions.")
        print("On Linux, you may need to add your user to the dialout group:")
        print("  sudo usermod -a -G dialout $USER")
        print("  (then log out and back in)")
        return
    
    print(f"Device initialized. Number of sensors: {trakstar.get_number_of_sensors()}")
    
    # Set up sensors
    num_sensors = trakstar.get_number_of_sensors()
    for i in range(num_sensors):
        trakstar.set_sensor_quaternion(i)
    
    # Read data continuously until Ctrl+C
    print("Reading sensor data continuously. Press Ctrl+C to stop...")
    try:
        while True:
            for sensor_id in range(num_sensors):
                data = trakstar.get_coordinates_quaternion(sensor_id)
                if data["success"]:
                    x, y, z = data["x"], data["y"], data["z"]
                    q = data["quaternion"]
                    print(f"Sensor {sensor_id}: pos=({x:.3f}, {y:.3f}, {z:.3f}) quat=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C)")

if __name__ == "__main__":
    main()
