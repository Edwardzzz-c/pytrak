#!/usr/bin/env python3

import rospy
import tf2_ros
import transforms3d as td
import numpy as np
import pickle
import std_msgs.msg

class CalibratedDataPublisher(object):
    def __init__(self):
        # Subscribing to TF
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.calibrated_joint_angle = rospy.Publisher('calibrated_joint_angle', std_msgs.msg.Float32, queue_size = 1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def load_file(self, filename):
        '''
        Loads in the calibration information stored in dictionary `calibration_information`.
        Contains "plane_rotation," "plane_translation," and "neutral_transform"
        '''
        try:
            file = open(filename, 'rb')
            self.calibration_information = pickle.load(file)
        except IOError:
            print("Failed to read file")
            return False
        return True

    def timer_callback(self, timer):
        '''
        Gets transforms from Trakstar and publishes the joint angle to `calibrated_joint_angle`
        '''
        # Collect the raw sensor 1 to sensor 0 transform 
        t_raw = self.get_transform()
        if t_raw is None: return

        # Rotate the raw transform into the plane 
        t = np.dot(self.calibration_information["plane_rotation"], t_raw)
        # Calculate the angle between the neutral pose and the current pose
        theta = td.euler.mat2euler(np.dot(np.linalg.inv(self.calibration_information["neutral_transform"]), t), axes = 'szxy')[0]*(180/np.pi)
        
        # Publish the angle to ROS
        msg = std_msgs.msg.Float32()
        msg.data = theta
        self.calibrated_joint_angle.publish(msg)
    
    def to_affine(self, t):
        '''
        Returns the 4x4 homogenous transform from the Transform message. 
        '''
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)
    
    def get_transform(self):
        '''
        Returns the transform from sensor 1 to sensor 0.
        '''
        try:
            # Calculating the transform from Sensor 0 to Sensor 1
            b_sensor0 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor1 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(1.0)))
            s1_to_s0 = np.dot(np.linalg.inv(b_sensor1), b_sensor0)
            return s1_to_s0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Calibration data collection: failed to get transforms")
            return None

if __name__ == '__main__':
    rospy.init_node('calibrated_data_publisher', anonymous=True)
    cdp = CalibratedDataPublisher()
    if not cdp.load_file('calibration_information'):
        exit(0)
    rospy.spin()
