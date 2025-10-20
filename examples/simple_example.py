#!/usr/bin/env python3
"""
Simple Trakstar Python Example

This is a minimal example showing basic usage of the Trakstar Python wrapper.
"""

import pytrak
import time

def main():
    # Initialize the device
    trakstar = pytrak.Trakstar()
    
    if not trakstar.is_ok():
        print("Failed to initialize Trakstar device")
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
