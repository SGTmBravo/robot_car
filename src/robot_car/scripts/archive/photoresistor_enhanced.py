#!/usr/bin/env python3
"""
Enhanced Photoresistor Node
Detects start signal with edge detection and debouncing
Uses hardware-adjustable threshold for ambient light compensation
"""

import rospy
from gpiozero import DigitalInputDevice
from std_msgs.msg import Bool
import time

class PhotoresistorNode:
    def __init__(self):
        rospy.init_node('photoresistor_node')
        
        # GPIO setup
        self.sensor = DigitalInputDevice(27, pull_up=False)  # GPIO27
        
        # Publishers
        self.start_pub = rospy.Publisher('/start_signal', Bool, queue_size=1)
        self.light_state_pub = rospy.Publisher('/light_detected', Bool, queue_size=1)
        
        # State tracking
        self.last_state = None
        self.last_change_time = 0
        self.debounce_delay = 0.1  # 100ms debounce
        
        # Statistics
        self.trigger_count = 0
        self.start_time = time.time()
        
        rospy.loginfo("🔦 Photoresistor node started on GPIO27")
        rospy.loginfo("📊 Publishing to /start_signal and /light_detected")
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz for responsive detection
        
        while not rospy.is_shutdown():
            current_state = bool(self.sensor.value)  # True if light detected
            current_time = time.time()
            
            # Always publish current light state
            self.light_state_pub.publish(Bool(data=current_state))
            
            # Edge detection with debouncing
            if self.last_state is not None and current_state != self.last_state:
                # State changed - check debounce
                if (current_time - self.last_change_time) > self.debounce_delay:
                    self.last_change_time = current_time
                    
                    if current_state:  # Light detected (rising edge)
                        self.trigger_count += 1
                        rospy.loginfo(f"🟢 START SIGNAL TRIGGERED! (#{self.trigger_count})")
                        self.start_pub.publish(Bool(data=True))
                    else:  # Light lost (falling edge)
                        rospy.loginfo(f"🔴 Light signal lost")
                        self.start_pub.publish(Bool(data=False))
            
            self.last_state = current_state
            
            # Periodic status (every 30 seconds)
            if int(current_time - self.start_time) % 30 == 0 and int(current_time) % 1 == 0:
                uptime = int(current_time - self.start_time)
                rospy.loginfo(f"📊 Status: {self.trigger_count} triggers in {uptime}s, Light: {'ON' if current_state else 'OFF'}")
            
            rate.sleep()
    
    def shutdown(self):
        rospy.loginfo("🛑 Photoresistor node shutting down")
        # Send final false signal
        self.start_pub.publish(Bool(data=False))

def main():
    node = PhotoresistorNode()
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Interrupted by user")
    finally:
        node.shutdown()

if __name__ == '__main__':
    main()
