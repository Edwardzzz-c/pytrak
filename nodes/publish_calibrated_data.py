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
        self.calibrated_joint_angle = rospy.Publisher('calibrated_joint_angle', std_msgs.msg.String, queue_size = 1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        self.sensor_information = {}

    def load_file(self, filename):
        '''
        Loads in the calibration information stored in dictionary `calibration_information`.
        Contains "plane_rotation," "plane_translation," and "neutral_transform".
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
        Gets transforms from Trakstar and publishes the joint angle to `\calibrated_joint_angle.`
        '''
        theta_mcp = 0
        theta_dip = 0

        # Collect the raw sensor 1 to sensor 0 transform 
        raw_transform_mcp = self.get_transform("mcp")
        raw_transform_dip = self.get_transform("dip")
        if raw_transform_mcp is None: return
        if raw_transform_dip is None: return 
        
        # Rotate the raw transform into the plane 
        plane_transform_mcp = np.dot(self.calibration_information["mcp"]["plane_rotation"], raw_transform_mcp)
        plane_transform_mcp = np.dot(self.calibration_information["mcp"]["plane_translation"], plane_transform_mcp)
        # Find the transform between the neutral transform and the current transform to get the joint angle
        joint_transform_mcp = np.dot(np.linalg.inv(self.calibration_information["mcp"]["neutral_transform"]), plane_transform_mcp)
        # Calculate the angle between the neutral pose and the current pose
        theta_mcp = td.euler.mat2euler(joint_transform_mcp, axes = 'szxy')[0]
        # Define the x and y position of sensor 1
        x = plane_transform_mcp[0, 3]
        y = plane_transform_mcp[1, 3]
        # Saves x, y, theta to dictionary `sensor_information`
        joint_dict_mcp = {"x" : [], "y" : [], "theta" : [], "transforms" : []}
        joint_dict_mcp["x"].append(x)
        joint_dict_mcp["y"].append(y)
        joint_dict_mcp["theta"].append(theta_mcp)
        joint_dict_mcp["transforms"].append(plane_transform_mcp)
        self.sensor_information["mcp"] = joint_dict_mcp

        # Rotate the raw transform into the plane 
        plane_transform_dip = np.dot(self.calibration_information["dip"]["plane_rotation"], raw_transform_dip)
        plane_transform_dip = np.dot(self.calibration_information["dip"]["plane_translation"], plane_transform_dip)
        # Find the transform between the neutral transform and the current transform to get the joint angle
        joint_transform_dip = np.dot(np.linalg.inv(self.calibration_information["dip"]["neutral_transform"]), plane_transform_dip)
        # Calculate the angle between the neutral pose and the current pose
        theta_dip = td.euler.mat2euler(joint_transform_dip, axes = 'szxy')[0]
        # Define the x and y position of sensor 1
        x = plane_transform_dip[0, 3]
        y = plane_transform_dip[1, 3]
        # Saves x, y, theta to dictionary `sensor_information`
        joint_dict_dip = {"x" : [], "y" : [], "theta" : [], "transforms" : []}
        joint_dict_dip["x"].append(x)
        joint_dict_dip["y"].append(y)
        joint_dict_dip["theta"].append(theta_dip)
        joint_dict_dip["transforms"].append(plane_transform_dip)
        self.sensor_information["dip"] = joint_dict_dip

        # Publish the angle to ROS
        msg = std_msgs.msg.String()
        msg.data = "MCP : %s, DIP: %s"%(str(theta_mcp*(180/np.pi))[:7], str(theta_dip*(180/np.pi))[:7])
        self.calibrated_joint_angle.publish(msg)
    
    def save_sensor_information(self, outfile):
        '''
        Saves sensor information dictionary, `sensor_information`, into a file. 
        '''
        pickle.dump(self.sensor_information, outfile)
        pickle.dump(self.calibration_information, outfile)
        outfile.close()
        print("Saved sensor information.")

    def to_affine(self, t):
        '''
        Returns the 4x4 homogenous transform from the Transform message. 
        '''
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)
    
    def get_transform(self, joint):
        '''
        Returns the transform from sensor 1 to sensor 0.
        '''
        try:
            # Calculating the transform from Sensor 0 to Sensor 1
            b_sensor0 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor1 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor2 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar2', rospy.Time(0), rospy.Duration(1.0)))
            s1_to_s0 = np.dot(np.linalg.inv(b_sensor1), b_sensor0)
            s2_to_s1 = np.dot(np.linalg.inv(b_sensor2), b_sensor1)
            if joint == "mcp":
                return s1_to_s0
            elif joint == "dip": 
                return s2_to_s1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Calibration data collection: failed to get transforms.")
            return None

if __name__ == '__main__':
    rospy.init_node('calibrated_data_publisher', anonymous=True)
    outfile = open('sensor_information', 'wb')
    cdp = CalibratedDataPublisher()
    if not cdp.load_file('calibration_information'):
        exit()
    rospy.spin()
    if rospy.is_shutdown():
        cdp.save_sensor_information(outfile)
