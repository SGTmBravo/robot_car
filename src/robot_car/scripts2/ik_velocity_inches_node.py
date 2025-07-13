#!/usr/bin/env python3
"""
Interactive Velocity Control - INCHES VERSION
Allows manual robot control using inch measurements
"""
import rospy
import math
from std_msgs.msg import Float32MultiArray
from unit_converter import inches_to_meters, format_position_dual

# Robot parameters
WHEEL_RADIUS = 0.02825
L = 0.075

def main():
    rospy.init_node('ik_velocity_inches_node')
    
    # Publisher for wheel velocities
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    
    rospy.loginfo("✅ ROS velocity node ready (INCHES VERSION)")
    print("\n🤖 ROBOT CONTROL - INCHES VERSION")
    print("="*40)
    print("📏 Enter distances in inches")
    print("🔄 Angles still in radians (0 = no rotation)")
    print("❌ Type 'q' to quit")
    
    while not rospy.is_shutdown():
        try:
            print("\n" + "-"*30)
            user_input = input("Enter x distance (inches) or 'q' to quit: ")
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
            
            x_inches = float(user_input)
            y_inches = float(input("Enter y distance (inches): "))
            theta = float(input("Enter θ rotation (rad): "))
            
            # Convert to meters for calculations
            x = inches_to_meters(x_inches)
            y = inches_to_meters(y_inches)
            
            print(f"📍 Command: {format_position_dual(x, y)}, θ={theta:.3f} rad")
            
        except KeyboardInterrupt:
            break
        except ValueError:
            rospy.logwarn("⚠️ Invalid input, try again.")
            continue

        # Distance to move
        linear_distance = math.hypot(x, y)
        angular_distance = abs(theta)

        # Duration based on maximum linear and angular speeds
        max_linear_speed = 0.05  # m/s
        max_angular_speed = 0.2  # rad/s
        
        linear_time = linear_distance / max_linear_speed if linear_distance > 0 else 0
        angular_time = angular_distance / max_angular_speed if angular_distance > 0 else 0
        total_time = max(linear_time, angular_time, 0.5)  # Minimum 0.5 seconds
        
        # Calculate velocities
        if total_time > 0:
            vx = x / total_time
            vy = y / total_time
            omega = theta / total_time
        else:
            vx = vy = omega = 0

        # Calculate wheel velocities using mecanum kinematics
        motor1 = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
        motor2 = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
        motor3 = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
        motor4 = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right

        print(f"⏱️ Duration: {total_time:.2f}s")
        print(f"🔧 Motors: FR={motor1:.2f}, FL={motor2:.2f}, RL={motor3:.2f}, RR={motor4:.2f}")

        # Send command
        msg = Float32MultiArray()
        msg.data = [motor1, motor2, motor3, motor4, total_time]
        wheel_pub.publish(msg)

        print(f"🚀 Moving...")
        # Wait for movement to complete
        rospy.sleep(total_time + 0.5)
        print("✅ Movement complete")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
