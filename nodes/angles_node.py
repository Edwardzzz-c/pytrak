import rospy
import math
from geometry_msgs.msg import Transform
from custom_msg.msg import TrakstarMsg
import numpy as np
import transforms3d as td
# import the message
# import the general tf package
from std_msgs.msg import String
from rclpy.node import init_node



class JointAngles(Node):

	def __init__(self):
		super().__init__('joint_angles')


def listener():
	rospy.init_node('joint_angles', anonymous=True)

	rospy.Subscriber('tf', Transform, callback)

	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("joint_angles")
	joint_angles = JointAngles()
	joint_angles.publish_loop()