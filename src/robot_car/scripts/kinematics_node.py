#!/usr/bin/env python3
# filepath: ~/robot_car_ws/src/robot_car/scripts/kinematics_node.py

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Set your robot's parameters here
WHEEL_RADIUS = 0.02825  # meters (example)
WHEEL_BASE = 0.1025    # meters (distance between wheels, example)

def cmd_vel_callback(msg):
    # Extract linear and angular velocity
    vx = msg.linear.x
    wz = msg.angular.z

    # Differential drive inverse kinematics
    v_right = (vx + (wz * WHEEL_BASE / 2.0)) / WHEEL_RADIUS
    v_left  = (vx - (wz * WHEEL_BASE / 2.0)) / WHEEL_RADIUS

    # Publish wheel velocities
    pub_right.publish(v_right)
    pub_left.publish(v_left)

    rospy.loginfo(f"Left wheel: {v_left:.2f} rad/s, Right wheel: {v_right:.2f} rad/s")

def main():
    global pub_right, pub_left
    rospy.init_node('kinematics_node')
    pub_right = rospy.Publisher('/right_wheel_ang_vel', Float64, queue_size=10)
    pub_left = rospy.Publisher('/left_wheel_ang_vel', Float64, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Kinematics node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
