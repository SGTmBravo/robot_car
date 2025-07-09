#!/usr/bin/env python3
# filepath: ~/robot_car_ws/src/robot_car/scripts/photoresistor_node.py

import rospy
from gpiozero import DigitalInputDevice
from std_msgs.msg import Bool

def main():
    rospy.init_node('photoresistor_node')
    pub = rospy.Publisher('/start_signal', Bool, queue_size=1)

    sensor = DigitalInputDevice(27)  # GPIO27
    rate = rospy.Rate(5)  # Hz

    while not rospy.is_shutdown():
        light_detected = sensor.value  # 1 if light, 0 if dark
        pub.publish(Bool(data=bool(light_detected)))
        rospy.loginfo(f"Start LED: {'ON' if light_detected else 'OFF'}")
        rate.sleep()

if __name__ == '__main__':
    main()

