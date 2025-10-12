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
from std_msgs.msg import String
from rclpy.duration import Duration

class CalibratedDataPublisher(Node):
    def __init__(self):
        super().__init__('calibrated_data_publisher')
        
        # Subscribing to TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.calibrated_joint_angle = self.create_publisher(String, 'calibrated_joint_angle', 1)
        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def load_file(self, filename):
        '''
        Loads in the calibration information stored in dictionary `calibration_information`.
        Contains "plane_rotation," "plane_translation," and "neutral_transform".
        '''
        try:
            file = open(filename, 'rb')
            self.calibration_info = pickle.load(file)
            
            self.s1_neutral_pose = self.calibration_info["sensor_1_neutral_pose"]
            self.s2_neutral_pose = self.calibration_info["sensor_2_neutral_pose"]
            
            self.joint_axis = self.calibration_info["joint_axis"]
            self.mcp_axis = np.dot(np.linalg.inv(self.s1_neutral_pose[:3, :3]), self.joint_axis)
            self.pip_axis = np.dot(np.linalg.inv(self.s2_neutral_pose[:3, :3]), self.joint_axis)

            self.mcp_neutral_pose = self.get_transform("sensor_1")
            self.pip_neutral_pose = self.get_transform("sensor_2")

        except IOError:
            self.get_logger().error("Failed to read file")
            return False
        return True

    def timer_callback(self):
        '''
        Gets transforms from Trakstar and publishes the joint angle to `/calibrated_joint_angle.`
        '''
        # Collect the raw sensor 0 to sensor 1 transform and the raw sensor 0 to sensor 1 transform
        s1_raw_transform = self.get_transform("sensor_1")
        s2_raw_transform = self.get_transform("sensor_2")
        
        if s1_raw_transform is None: return
        if s2_raw_transform is None: return

        mcp_angle = self.calculate_relative_angle(self.mcp_neutral_pose, s1_raw_transform, self.mcp_axis)
        pip_angle = self.calculate_relative_angle(self.pip_neutral_pose, s2_raw_transform, self.pip_axis) - mcp_angle

        # Publish the angle to ROS
        msg = String()
        msg.data = "MCP : %s, DIP: %s"%(str(mcp_angle)[:7], str(pip_angle)[:7])
        self.calibrated_joint_angle.publish(msg)
    
    def twist_rotation_about_axis(self, transform, axis):
        '''
        Returns the angle theta in degrees about the twist axis. 
        '''
        # quaternion convention is [w, x, y, z]
        transform_quat = td.quaternions.mat2quat(transform[:3, :3])
        transform_axes = np.array([transform_quat[1], transform_quat[2], transform_quat[3]])
        
        dot_product = np.dot(transform_axes, axis)
        axis_norm = np.linalg.norm(axis)
        projection = (dot_product / axis_norm**2) * axis 
        
        # define twist in quaternion form and normalize
        twist = np.array([transform_quat[0], projection[0], projection[1], projection[2]])
        twist /= td.quaternions.qnorm(twist)
        
        # catching singularities
        threshold = 1e-6
        if td.quaternions.qnorm(twist) < threshold:
            self.get_logger().warning("Singularity in twist")
            return
        elif td.quaternions.qnorm(transform_quat) < threshold:
            self.get_logger().warning("Singularity in rotation")
            return
        
        if dot_product < 0:
            twist = -1*twist
        
        #swing = td.quaternions.qmult(transform_quat, td.quaternions.qconjugate(twist))
        #swing /= td.quaternions.qnorm(twist)
        
        twist_axis, twist_theta = td.quaternions.quat2axangle(twist)
    
        if not np.allclose(axis, twist_axis):
            self.get_logger().warning("Axis of rotation is not the given axis. Something went wrong.")
            self.get_logger().warning("twist axis: %s"%(twist_axis))
            self.get_logger().warning("pca axis  : %s"%(axis))
        
        return twist_theta*(180/np.pi)
        
    def calculate_relative_angle(self, reference_transform, target_transform, axis):
        
        relative_transform = np.dot(np.linalg.inv(reference_transform), target_transform)
        target_theta = self.twist_rotation_about_axis(relative_transform, axis)
        delta = abs(target_theta)
        theta = min(delta, 360-delta)

        return theta
    
    def save_sensor_information(self, outfile):
        '''
        Saves sensor information dictionary, `sensor_information`, into a file. 
        '''
        pickle.dump(self.joint_angle_info, outfile)
        outfile.close()
        self.get_logger().info("Saved joint angle information.")

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

def main(args=None):
    rclpy.init(args=args)
    
    cdp = CalibratedDataPublisher()
    if not cdp.load_file('calibration_info'):
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(cdp)
    except KeyboardInterrupt:
        pass
    finally:
        cdp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
