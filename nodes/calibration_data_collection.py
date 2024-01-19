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

    def record_neutral_angle_transforms(self, num_transforms, file):
        '''
        Records the hand in a neutral pose to get a baseline position.
        '''
        t = 0
        neutral_poses = {"sensor_1" : [], "sensor_2" : []}
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and t<num_transforms:
            rate.sleep()
            s0_to_s1 = self.get_transform("sensor_1")
            s0_to_s2 = self.get_transform("sensor_2")
            if s0_to_s1 is None: continue 
            if s0_to_s2 is None: continue
            neutral_poses["sensor_1"].append(s0_to_s1)
            neutral_poses["sensor_2"].append(s0_to_s2)
            rospy.loginfo("Transform %s saved"%(t+1))
            t=t+1    
        if t<num_transforms:
            rospy.logerr("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(neutral_poses, file)
        rospy.loginfo("All neutral transforms saved")    

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
        rate = rospy.Rate(5)
        # Defining the identity transform
        previous_transform = td.affines.compose(np.zeros(3), np.eye(3), np.ones(3))

        while not rospy.is_shutdown() and t<num_transforms:
            rate.sleep()
            s0_to_s1 = self.get_transform("sensor_1")
            s0_to_s2 = self.get_transform("sensor_2")
            if s0_to_s1 is None: continue 
            if s0_to_s2 is None: continue

            # Only record transform if the difference in translational position > 2mm
            if self.different_transforms(s0_to_s2, previous_transform):
                sweeping_poses["sensor_1"].append(s0_to_s1)
                sweeping_poses["sensor_2"].append(s0_to_s2)
                t=t+1
                previous_transform = s0_to_s2
                rospy.loginfo("Transform %s saved"%(t))    

        if t<num_transforms:
            rospy.logerr("Failed to get enough transforms")
            return
        
        # Save transforms stored in a list to a pickle file
        pickle.dump(sweeping_poses, file)
        rospy.loginfo("All %s sweeping transforms saved."%(num_transforms))    

if __name__ == '__main__':
    rospy.init_node('calibration_data_collection', anonymous=True)
    cdc = CalibrationDataCollection()
    file = open('calibration_poses', 'wb')
    cdc.record_neutral_angle_transforms(10, file)
    cdc.record_finger_sweeping_transforms(40, file)
    file.close()
