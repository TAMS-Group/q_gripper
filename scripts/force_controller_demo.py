#!/usr/bin/env python

from sensor_msgs.msg import JointState, Joy
import sys
import rospy
from tams_diana7_tools.dual_diana_helper import DualDianaHelper

rospy.init_node('force_controller tester')

goal_pub = rospy.Publisher('/arm_l/q_gripper/force_goal', Joy, queue_size=10)
pressure = 0.4
dh = DualDianaHelper(load_moveit=False)
dh.load_twist_controllers()
dh.arm_l.move_arm_with_velocity(vz=0.05, duration=1.0)
input("Press Enter to continue...")
dh.arm_l.move_arm_with_velocity(vz=-0.045, duration=1.0)
c = 0
r = rospy.Rate(15)
while not rospy.is_shutdown() and c < 150:
    js_msg = Joy()
    js_msg.header.stamp = rospy.Time.now()
    js_msg.axes.append(pressure)
    goal_pub.publish(js_msg)
    c += 1
    r.sleep()
dh.arm_l.move_arm_with_velocity(vz=0.05, duration=1.0)
rospy.sleep(1.0)
dh.arm_l.gripper.open()
