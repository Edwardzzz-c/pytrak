#!/usr/bin/env python

import rospy
import math
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
import transforms3d as td
import numpy as np
#roslib.load_manifest('trakstar')


def to_affine(t):
    T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
    R = td.quaternions.quat2mat(rotation)
    Z = np.ones(3)

    return td.affines.compose(T, R, Z)


def jnode():
    rospy.init_node('joint_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    joint_angle = rospy.Publisher('joint_angle', std_msgs.msg.Float32, queue_size = 1)

    rate = rospy.Rate(10.0)
    b_sensor0_i = to_affine(tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0), rospy.Duration(3.0)))
    b_sensor1_i = to_affine(tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0), rospy.Duration(3.0)))
    
    s0_to_s1_i = np.dot(np.linalg.inv(b_sensor0_i), b_sensor1_i)
    
    while not rospy.is_shutdown():
        try:
            b_sensor0 = to_affine(tfBuffer.lookup_transform('trakstar_base', 'trakstar0', rospy.Time(0)))
            b_sensor1 = to_affine(tfBuffer.lookup_transform('trakstar_base', 'trakstar1', rospy.Time(0)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        s0_to_s1 = np.dot(np.linalg.inv(b_sensor0), b_sensor1)
        angle_tf = np.dot(np.linalg.inv(s0_to_s1_i), s0_to_s1)
        
        ax, angle, pt = td.axangles.aff2axangle(angle_tf)

        msg = std_msgs.msg.Float32()
        msg.data = angle * (180 / np.pi)

        joint_angle.publish(msg)

if __name__ == '__main__':
    try:
        jnode()
    except rospy.ROSInterruptException:
        pass