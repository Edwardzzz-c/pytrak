#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import transforms3d as td
import numpy as np
import pickle
from rclpy.duration import Duration

class CalibrationDataCollection(Node):
    def __init__(self):
        super().__init__('calibration_data_collection')
        
        # Subscribing to TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def to_affine(self, t):
        '''
        Returns the 4x4 homogenous transform from the Transform message. 
        '''
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)
    
    def get_transform(self, sensor):
        '''
        Returns the transform from sensor 0 to the user-defined sensor.
        '''
        try:
            # Calculating the transform from Sensor 0 to Sensor 1
            b_sensor0 = self.to_affine(self.tf_buffer.lookup_transform('trakstar_base', 'trakstar0', Time(), Duration(seconds=1.0)))
            b_sensor1 = self.to_affine(self.tf_buffer.lookup_transform('trakstar_base', 'trakstar1', Time(), Duration(seconds=1.0)))
            b_sensor2 = self.to_affine(self.tf_buffer.lookup_transform('trakstar_base', 'trakstar2', Time(), Duration(seconds=1.0)))
            s0_to_s1 = np.dot(np.linalg.inv(b_sensor0), b_sensor1)
            s0_to_s2 = np.dot(np.linalg.inv(b_sensor0), b_sensor2)
            if sensor == "sensor_1":
                return s0_to_s1
            elif sensor == "sensor_2": 
                return s0_to_s2
        except TransformException as ex:
            self.get_logger().error("Calibration data collection: failed to get transforms: %s" % ex)
            return None

    def record_neutral_angle_transforms(self, num_transforms, file):
        '''
        Records the hand in a neutral pose to get a baseline position.
        '''
        t = 0
        neutral_poses = {"sensor_1" : [], "sensor_2" : []}
        rate = self.create_rate(5)

        while rclpy.ok() and t < num_transforms:
            rate.sleep()
            s0_to_s1 = self.get_transform("sensor_1")
            s0_to_s2 = self.get_transform("sensor_2")
            if s0_to_s1 is None: continue 
            if s0_to_s2 is None: continue
            neutral_poses["sensor_1"].append(s0_to_s1)
            neutral_poses["sensor_2"].append(s0_to_s2)
            self.get_logger().info("Transform %s saved"%(t+1))
            t = t+1    
        if t < num_transforms:
            self.get_logger().error("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(neutral_poses, file)
        self.get_logger().info("All neutral transforms saved")    

    def different_transforms(self, t1, t2):
        '''
        Returns whether the translational difference between two transforms is greater than 5 mm
        '''
        if np.linalg.norm(t1[0:2, 3] - t2[0:2, 3]) >= 0.005:
            return True
        return False

    def record_finger_sweeping_transforms(self, num_transforms, file):
        '''
        Records the transforms that define the finger's full range of motion. 
        Transforms are only recorded when there is a difference in translational position of more than 5 mm. 
        '''
        t = 0
        sweeping_poses = {"sensor_1" : [], "sensor_2" : []}
        rate = self.create_rate(5)
        # Defining the identity transform
        previous_transform = td.affines.compose(np.zeros(3), np.eye(3), np.ones(3))

        while rclpy.ok() and t < num_transforms:
            rate.sleep()
            s0_to_s1 = self.get_transform("sensor_1")
            s0_to_s2 = self.get_transform("sensor_2")
            if s0_to_s1 is None: continue 
            if s0_to_s2 is None: continue

            # Only record transform if the difference in translational position > 2mm
            if self.different_transforms(s0_to_s2, previous_transform):
                sweeping_poses["sensor_1"].append(s0_to_s1)
                sweeping_poses["sensor_2"].append(s0_to_s2)
                t = t+1
                previous_transform = s0_to_s2
                self.get_logger().info("Transform %s saved"%(t))    

        if t < num_transforms:
            self.get_logger().error("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(sweeping_poses, file)
        self.get_logger().info("All %s sweeping transforms saved."%(num_transforms))    

def main(args=None):
    rclpy.init(args=args)
    
    cdc = CalibrationDataCollection()
    file = open('calibration_poses', 'wb')
    cdc.record_neutral_angle_transforms(10, file)
    cdc.record_finger_sweeping_transforms(40, file)
    file.close()
    
    cdc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
