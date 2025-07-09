#!/usr/bin/env python3
# filepath: ~/robot_car_ws/src/robot_car/scripts/motor_control_node.py

import rospy
from std_msgs.msg import String

def command_callback(msg):
    rospy.loginfo(f"Received movement command: {msg.data}")

def main():
    rospy.init_node('motor_control_node')
    rospy.Subscriber('motor_command', String, command_callback)
    rospy.loginfo("Motor control node started, waiting for commands...")
    rospy.spin()

if __name__ == '__main__':
    main()
