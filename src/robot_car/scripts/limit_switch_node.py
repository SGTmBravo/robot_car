#!/usr/bin/env python3
# filepath: ~/robot_car_ws/src/robot_car/scripts/limit_switch_node.py

import rospy
from gpiozero import Button
from std_msgs.msg import Bool

def main():
    rospy.init_node('limit_switch_node')
    pub = rospy.Publisher('/limit_switch_triggered', Bool, queue_size=1)

    switch = Button(22, pull_up=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        triggered = not switch.is_pressed  # Inverted: switch reads opposite
        pub.publish(Bool(data=triggered))
        rospy.loginfo(f"Limit switch: {'PRESSED' if triggered else 'RELEASED'}")
        rate.sleep()

if __name__ == '__main__':
    main()
