#!/usr/bin/env python

import rospy
import math
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
import transforms3d as td
import numpy as np

class JointAngle(object):
    def __init__(self):
        # Subscribing to TF
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publishing joint angle
        self.joint_angle = rospy.Publisher('joint_angle', std_msgs.msg.Float32, queue_size = 1)

        b_sensor0_i = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(3.0)))
        b_sensor1_i = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(3.0)))

        self.s0_to_s1_i = np.dot(np.linalg.inv(b_sensor0_i), b_sensor1_i)

    def to_affine(self, t):
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)

    def angle_calc(self):
        while not rospy.is_shutdown():
            try:
                b_sensor0 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0)))
                b_sensor1 = self.to_affine(self.tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0)))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            s0_to_s1 = np.dot(np.linalg.inv(b_sensor0), b_sensor1)
            angle_tf_1 = np.dot(np.linalg.inv(self.s0_to_s1_i), s0_to_s1)

            ax1, angle_1, pt1 = td.axangles.aff2axangle(angle_tf_1)

            msg = std_msgs.msg.Float32()
            msg.data = angle_1 * (180 / np.pi)

            self.joint_angle.publish(msg)


if __name__ == '__main__':
    rospy.init_node('joint_node', anonymous=True)
    jnode = JointAngle()
    jnode.angle_calc()
    rospy.spin()