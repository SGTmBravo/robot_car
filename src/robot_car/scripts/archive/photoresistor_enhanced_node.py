#!/usr/bin/env python3
"""
Enhanced Photoresistor Node
Detects light levels and publishes start signals for robot competition
Supports both digital and analog modes with configurable thresholds
"""

import rospy
import time
from gpiozero import DigitalInputDevice, MCP3008
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Illuminance

class PhotoresistorNode:
    def __init__(self):
        rospy.init_node('photoresistor_node')
        
        # Configuration parameters
        self.digital_pin = rospy.get_param('~digital_pin', 27)  # GPIO27 default
        self.use_analog = rospy.get_param('~use_analog', False)  # Set to True if using ADC
        self.adc_channel = rospy.get_param('~adc_channel', 0)   # MCP3008 channel 0
        self.light_threshold = rospy.get_param('~light_threshold', 0.5)  # 0-1 for analog, 1/0 for digital
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        self.edge_detection = rospy.get_param('~edge_detection', True)  # Only publish on state changes
        self.calibration_mode = rospy.get_param('~calibration_mode', False)  # Continuous readings for setup
        
        # Publishers
        self.start_pub = rospy.Publisher('/start_signal', Bool, queue_size=1)
        self.light_pub = rospy.Publisher('/light_level', Float32, queue_size=1)
        self.status_pub = rospy.Publisher('/photoresistor_status', String, queue_size=1)
        self.illuminance_pub = rospy.Publisher('/illuminance', Illuminance, queue_size=1)
        
        # State tracking for edge detection
        self.last_state = None
        self.light_detected_time = None
        self.dark_detected_time = None
        
        # Initialize sensors
        self.setup_sensors()
        
        # Main loop
        self.rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("🔆 Enhanced Photoresistor Node Started")
        rospy.loginfo(f"   Digital Pin: GPIO{self.digital_pin}")
        rospy.loginfo(f"   Analog Mode: {'Enabled' if self.use_analog else 'Disabled'}")
        rospy.loginfo(f"   Threshold: {self.light_threshold}")
        rospy.loginfo(f"   Edge Detection: {'Enabled' if self.edge_detection else 'Disabled'}")
        rospy.loginfo(f"   Calibration Mode: {'Enabled' if self.calibration_mode else 'Disabled'}")
        
        if self.calibration_mode:
            self.run_calibration()
        else:
            self.run_normal()
    
    def setup_sensors(self):
        """Initialize the photoresistor sensors"""
        try:
            # Digital sensor (always available)
            self.digital_sensor = DigitalInputDevice(self.digital_pin)
            rospy.loginfo(f"✅ Digital sensor initialized on GPIO{self.digital_pin}")
            
            # Analog sensor (optional)
            if self.use_analog:
                try:
                    self.analog_sensor = MCP3008(channel=self.adc_channel)
                    rospy.loginfo(f"✅ Analog sensor initialized on MCP3008 channel {self.adc_channel}")
                except Exception as e:
                    rospy.logwarn(f"⚠️ Analog sensor failed to initialize: {e}")
                    rospy.logwarn("🔄 Falling back to digital mode only")
                    self.use_analog = False
                    
        except Exception as e:
            rospy.logerr(f"❌ Failed to initialize sensors: {e}")
            raise
    
    def read_light_level(self):
        """Read current light level (0.0 = dark, 1.0 = bright)"""
        try:
            if self.use_analog:
                # Read from ADC (MCP3008 returns 0.0 to 1.0)
                analog_value = self.analog_sensor.value
                return analog_value
            else:
                # Read from digital pin (0 or 1)
                digital_value = float(self.digital_sensor.value)
                return digital_value
        except Exception as e:
            rospy.logwarn(f"⚠️ Sensor read error: {e}")
            return 0.0
    
    def is_light_detected(self, light_level):
        """Determine if light is detected based on threshold"""
        return light_level >= self.light_threshold
    
    def publish_readings(self, light_level, light_detected):
        """Publish all sensor readings"""
        
        # Start signal (main competition trigger)
        self.start_pub.publish(Bool(data=light_detected))
        
        # Raw light level
        self.light_pub.publish(Float32(data=light_level))
        
        # Illuminance message (standard ROS sensor message)
        illuminance_msg = Illuminance()
        illuminance_msg.header.stamp = rospy.Time.now()
        illuminance_msg.header.frame_id = "photoresistor"
        illuminance_msg.illuminance = light_level * 1000.0  # Convert to lux (approximate)
        illuminance_msg.variance = 0.01  # Estimate
        self.illuminance_pub.publish(illuminance_msg)
        
        # Status message
        status = f"Light: {'ON' if light_detected else 'OFF'} (Level: {light_level:.3f})"
        self.status_pub.publish(String(data=status))
    
    def run_calibration(self):
        """Calibration mode - continuous readings for threshold setup"""
        rospy.loginfo("🔧 CALIBRATION MODE - Monitor light levels to set threshold")
        rospy.loginfo("   Point a light at the sensor and observe the readings")
        rospy.loginfo("   Press Ctrl+C when done")
        
        try:
            while not rospy.is_shutdown():
                light_level = self.read_light_level()
                light_detected = self.is_light_detected(light_level)
                
                # More verbose output for calibration
                status = f"Level: {light_level:.3f} | Threshold: {self.light_threshold:.3f} | Detection: {'✅ ON' if light_detected else '❌ OFF'}"
                rospy.loginfo(status)
                
                self.publish_readings(light_level, light_detected)
                self.rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🔧 Calibration complete")
    
    def run_normal(self):
        """Normal operation mode"""
        try:
            while not rospy.is_shutdown():
                light_level = self.read_light_level()
                light_detected = self.is_light_detected(light_level)
                
                # Edge detection logic
                if self.edge_detection:
                    if light_detected != self.last_state:
                        if light_detected:
                            self.light_detected_time = rospy.Time.now()
                            rospy.loginfo(f"🔆 LIGHT DETECTED! Level: {light_level:.3f}")
                        else:
                            self.dark_detected_time = rospy.Time.now()
                            rospy.loginfo(f"🌑 Light lost. Level: {light_level:.3f}")
                        
                        # Always publish on state change
                        self.publish_readings(light_level, light_detected)
                        self.last_state = light_detected
                    
                    # Periodic status (less frequent)
                    if rospy.Time.now().to_sec() % 5.0 < (1.0 / self.publish_rate):
                        rospy.logdebug(f"Status: {'Light' if light_detected else 'Dark'} ({light_level:.3f})")
                else:
                    # Always publish if edge detection is disabled
                    self.publish_readings(light_level, light_detected)
                    rospy.loginfo(f"Light: {'ON' if light_detected else 'OFF'} ({light_level:.3f})")
                
                self.rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Photoresistor node stopped")
        except Exception as e:
            rospy.logerr(f"❌ Error in main loop: {e}")

def main():
    try:
        node = PhotoresistorNode()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ Failed to start photoresistor node: {e}")

if __name__ == '__main__':
    main()
