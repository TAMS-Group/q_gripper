#!/usr/bin/env python

from sensor_msgs.msg import JointState, Joy
import sys
import rospy

rospy.init_node('force_controller tester')

goal_pub = rospy.Publisher('/arm_l/q_gripper/force_goal', Joy, queue_size=10)
pressure = float(input("Enter pressure: "))

r = rospy.Rate(15)
while not rospy.is_shutdown():
    js_msg = Joy()
    js_msg.header.stamp = rospy.Time.now()
    js_msg.axes.append(pressure)
    goal_pub.publish(js_msg)
    r.sleep()
