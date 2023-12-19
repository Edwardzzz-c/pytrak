#!/usr/bin/env python

import rospy
import tf2_ros
import transforms3d as td
import numpy as np
import pickle

class CalibrationDataCollection(object):
    def __init__(self):
        # Subscribing to TF
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def to_affine(self, t):
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)

    def record_neutral_angle_transforms(self, num_transforms, file):
        '''
        This function is called to record the hand in a neutral pose to get a baseline position.
        '''
        t = 0
        neutral_poses = []
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and t<num_transforms:
            rate.sleep()
            s1_to_s0 = self.get_transform()
            if s1_to_s0 is None: continue 
            neutral_poses.append(s1_to_s0)
            t=t+1    
        if t<num_transforms:
            rospy.logerr("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(neutral_poses, file)
        rospy.loginfo("All neutral transform saved")    

    def different_transforms(self, t1, t2):
        if np.linalg.norm(t1[0:2, 3] - t2[0:2, 3]) >= 0.005:
            return True
        return False
    
    def record_data_transforms(self, num_transforms, file):
        '''
        This function records a specified number of transforms and saves them to a file. 
        '''
        t = 0
        data_poses = []
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and t<num_transforms:
            rate.sleep()
            s1_to_s0 = self.get_transform()
            if s1_to_s0 is None: continue 
            data_poses.append(s1_to_s0)
            t=t+1    
        if t<num_transforms:
            rospy.logerr("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(data_poses, file)
        rospy.loginfo("All data transform saved")  

    def record_test_transforms(self, file):
        test_poses = []
        rate = rospy.Rate(0.2)

        print("Position at 45 degrees")
        rate.sleep()
        s1_to_s0 = self.get_transform()
        if s1_to_s0 is None: return
        test_poses.append(s1_to_s0)

        print("Position at 90 degrees")
        rate.sleep()
        s1_to_s0 = self.get_transform()
        if s1_to_s0 is None: return
        test_poses.append(s1_to_s0)

        print("Position at -45 degrees")
        rate.sleep()
        s1_to_s0 = self.get_transform()
        if s1_to_s0 is None: return
        test_poses.append(s1_to_s0)

        print("Position at -90 degrees")
        rate.sleep()
        s1_to_s0 = self.get_transform()
        if s1_to_s0 is None: return
        test_poses.append(s1_to_s0)

        print("Position at 0 degrees")
        rate.sleep()
        s1_to_s0 = self.get_transform()
        if s1_to_s0 is None: return
        test_poses.append(s1_to_s0)

        pickle.dump(test_poses, file)
        rospy.loginfo("All test transforms saved")    

    def get_transform(self):
        try:
            # Calculating the transform from Sensor 0 to Sensor 1
            b_sensor0 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(1.0)))
            b_sensor1 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(1.0)))
            s1_to_s0 = np.dot(np.linalg.inv(b_sensor1), b_sensor0)
            return s1_to_s0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Calibration data collection: failed to get transforms")
            return None

    def record_finger_sweeping_transforms(self, num_transforms, file):
        '''
        This function records the transforms that define the finger's full range of motion. 
        Transforms are only recorded when there is a difference in translational position of more than 5 mm. 
        '''
        t = 0
        sweeping_poses = []
        rate = rospy.Rate(5)
        # Defining the identity transform
        previous_transform = td.affines.compose(np.zeros(3), np.eye(3), np.ones(3))

        while not rospy.is_shutdown() and t<num_transforms:
            rate.sleep()
            s1_to_s0 = self.get_transform()
            if s1_to_s0 is None: continue 

            # Only record transform if the difference in translational position > 5mm
            if self.different_transforms(s1_to_s0, previous_transform):
                sweeping_poses.append(s1_to_s0)
                t=t+1
                previous_transform = s1_to_s0
                rospy.loginfo("Transform saved")    

        if t<num_transforms:
            rospy.logerr("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(sweeping_poses, file)
        rospy.loginfo("All sweeping transforms saved")    

if __name__ == '__main__':
    rospy.init_node('calibration_data_collection', anonymous=True)
    cdc = CalibrationDataCollection()
    file = open('calibration_poses', 'wb')
    cdc.record_neutral_angle_transforms(10, file)
    cdc.record_finger_sweeping_transforms(40, file)
    cdc.record_data_transforms(1000, file)
    file.close()
