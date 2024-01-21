#!/usr/bin/env python

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
        self.calibrated_joint_angle = rospy.Publisher('calibrated_joint_angle', std_msgs.msg.String, queue_size = 1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        self.s1_neutral_pose = self.calibration_info["sensor_1_neutral_pose"]
        self.s2_neutral_pose = self.calibration_info["sensor_2_neutral_pose"]
        self.joint_axis = self.calibration_info["joint_axis"]
        self.mcp_w = rospy.Publisher('mcp_w', std_msgs.msg.Float32, queue_size = 1)
        self.pip_w = rospy.Publisher('pip_w', std_msgs.msg.Float32, queue_size = 1)
    
    def load_file(self, filename):
        '''
        Loads in the calibration information stored in dictionary `calibration_information`.
        Contains "plane_rotation," "plane_translation," and "neutral_transform".
        '''
        try:
            file = open(filename, 'rb')
            self.calibration_info = pickle.load(file)
        except IOError:
            print("Failed to read file")
            return False
        return True

    def timer_callback(self, timer):
        '''
        Gets transforms from Trakstar and publishes the joint angle to `\calibrated_joint_angle.`
        '''
        # Collect the raw sensor 0 to sensor 1 transform and the raw sensor 0 to sensor 1 transform
        s1_raw_transform = self.get_transform("sensor_1")
        s2_raw_transform = self.get_transform("sensor_2")
        
        if s1_raw_transform is None: return
        if s2_raw_transform is None: return

        mcp_angle, mcp_w = self.calculate_relative_angle(self.s1_neutral_pose, s1_raw_transform, self.joint_axis)
        pip_angle, pip_w = self.calculate_relative_angle(self.s2_neutral_pose, s2_raw_transform, self.joint_axis) # - mcp_angle

        # Publish the angle to ROS
        msg = std_msgs.msg.String()
        msg.data = "MCP : %s, DIP: %s"%(str(mcp_angle)[:7], str(pip_angle)[:7])
        self.calibrated_joint_angle.publish(msg)

        # Publish the w to ROS to ensure that there's no quaternion flipping happening
        mcp_msg = std_msgs.msg.Float32()
        pip_msg = std_msgs.msg.Float32()
        mcp_msg.data = mcp_w
        pip_msg.data = pip_w
        self.mcp_w.publish(mcp_msg)
        self.pip_w.publish(pip_msg)
    
    def twist_rotation_about_axis(self, transform_quat, axis):
        # quaternion convention is [w, x, y, z]
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
            print("Singularity in twist")
            return
        elif td.quaternions.qnorm(transform_quat) < threshold:
            print("Singularity in rotation")
            return
    
        twist_axis, twist_theta = td.quaternions.quat2axangle(twist)
        if not np.allclose(twist_axis, axis):
            print("Axis of rotation is not the given axis. Something went wrong.")
            return
        
        if dot_product < 0.0:
            return -1*twist_theta
        else:
            return twist_theta
        
    def calculate_relative_angle(self, reference_transform, target_transform, axis):
        
        relative_transform = np.dot(np.linalg.inv(reference_transform), target_transform)
        relative_transform_quat = td.quaternions.mat2quat(relative_transform[:3, :3])
        theta = self.twist_rotation_about_axis(relative_transform_quat, axis)*(180/np.pi)

        # collecting the w value of quaternion for debugging. 
        return theta, relative_transform_quat[0]
    
    def save_sensor_information(self, outfile):
        '''
        Saves sensor information dictionary, `sensor_information`, into a file. 
        '''
        pickle.dump(self.joint_angle_info, outfile)
        outfile.close()
        print("Saved joint angle information.")

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
            b_sensor0 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor1 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor2 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar2', rospy.Time(0), rospy.Duration(1.0)))
            s0_to_s1 = np.dot(np.linalg.inv(b_sensor0), b_sensor1)
            s0_to_s2 = np.dot(np.linalg.inv(b_sensor0), b_sensor2)
            if sensor == "sensor_1":
                return s0_to_s1
            elif sensor == "sensor_2": 
                return s0_to_s2
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Calibration data collection: failed to get transforms")
            return None

if __name__ == '__main__':
    rospy.init_node('calibrated_data_publisher', anonymous=True)
    outfile = open('sensor_information', 'wb')
    cdp = CalibratedDataPublisher()
    if not cdp.load_file('calibration_info'):
        exit()
    rospy.spin()
    if rospy.is_shutdown():
        cdp.save_sensor_information(outfile)
