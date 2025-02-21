#! /usr/bin/env python
from simple_pid import PID

import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState, Joy
import sys
import rospy



class ForceController:
    def __init__(self):
        self.last_force = None
        self.current_pos_goal = 0.5
        self.sg_pub = rospy.Publisher('/arm_l/q_gripper/simple_goal', JointState, queue_size=10)
        self.goal_sub = rospy.Subscriber('/arm_l/q_gripper/force_goal', Joy, self.goal_callback)
        self.force_sub = rospy.Subscriber('/arm_l/q_gripper/forces', Joy, self.force_callback)
        self.pid = PID(0.0004, 0.00002, 0.0)
        self.last_goal: Joy = None

    def goal_callback(self, msg):
        self.last_goal = msg

    def force_callback(self, msg):
        if self.last_goal is None or len(self.last_goal.axes) == 0 or self.last_goal.header.stamp < rospy.Time.now() - rospy.Duration(0.1):
            self.pid = PID(0.0004, 0.00002, 0.0)
            self.current_pos_goal = 0.5
            return
        self.pid.setpoint = self.last_goal.axes[0]
        self.last_force = msg.axes[0]

        control_signal = self.pid(self.last_force)
        self.current_pos_goal += control_signal
        js_msg = JointState()
        js_msg.position.append(self.current_pos_goal)
        self.sg_pub.publish(js_msg)

    def run(self):
        rospy.spin()
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('force_controller')
    fc = ForceController()
    fc.run()
    sys.exit(0)

