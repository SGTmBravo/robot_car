#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

# Robot parameters (same as master_state_machine_node.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.05       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor from linear_calibration.py

def move_robot_inches(x_inches, y_inches, theta, wheel_pub):
    """Move robot using inch measurements - EXACT SAME as master_state_machine_node.py"""
    # Convert inches to meters
    x_meters = x_inches * INCHES_TO_METERS
    y_meters = y_inches * INCHES_TO_METERS
    
    rospy.loginfo(f"📦 Moving: x={x_inches:.1f}\" ({x_meters:.3f}m), y={y_inches:.1f}\" ({y_meters:.3f}m), θ={theta:.3f} rad")
    
    # Do linear movement first (using linear_calibration approach)
    if x_meters != 0 or y_meters != 0:
        distance = math.sqrt(x_meters**2 + y_meters**2)
        linear_time = (distance / MOTOR_SPEED) * DISTANCE_SCALE
        
        vx = (x_meters / distance) * MOTOR_SPEED
        vy = (y_meters / distance) * MOTOR_SPEED
        omega = 0
        
        # Same motor calculations as master_state_machine_node.py
        motor1_fr = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
        motor2_fl = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
        motor3_rl = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
        motor4_rr = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right
        
        msg = Float32MultiArray()
        msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, linear_time]
        wheel_pub.publish(msg)
        rospy.loginfo(f"🏃 Linear movement: {linear_time:.2f}s")
        rospy.sleep(linear_time + 0.5)
    
    # Then do rotation if needed
     if theta != 0:
        angular_speed = 0.2
        angular_time = abs(theta) / angular_speed
        
        # For pure rotation: all wheels should move in coordinated pattern
        # Positive theta = counterclockwise rotation
        omega = angular_speed if theta > 0 else -angular_speed
        
        # Correct mecanum rotation: FR&RR same direction, FL&RL same direction (opposite to FR&RR)
        motor1_fr = -omega * L / WHEEL_RADIUS  # Front Right
        motor2_fl = +omega * L / WHEEL_RADIUS  # Front Left  
        motor3_rl = +omega * L / WHEEL_RADIUS  # Rear Left
        motor4_rr = -omega * L / WHEEL_RADIUS  # Rear Right
        
        msg = Float32MultiArray()
        msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, angular_time]
        wheel_pub.publish(msg)
        rospy.loginfo(f"🔄 Angular movement: {angular_time:.2f}s")
        rospy.sleep(angular_time + 0.5)
    
    # Stop
    stop_robot(wheel_pub)
    rospy.loginfo("✅ Step complete")

def stop_robot(wheel_pub):
    """Send stop command - EXACT SAME as master_state_machine_node.py"""
    rospy.loginfo("🛑 Stopping robot")
    msg = Float32MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.1]  # All motors stop
    wheel_pub.publish(msg)
    rospy.sleep(0.1)

def main():
    rospy.init_node('ik_velocity_node')
    
    # Publisher for wheel velocities
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.sleep(0.5)  # Allow publisher to establish connection
    
    rospy.loginfo("✅ IK velocity node ready - using INCHES like master_state_machine!")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter x distance (INCHES) or 'q' to quit: ")
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
            x_inches = float(user_input)
            y_inches = float(input("Enter y distance (INCHES): "))
            theta = float(input("Enter θ rotation (rad): "))
            
            # Use the EXACT SAME function as master_state_machine_node.py
            move_robot_inches(x_inches, y_inches, theta, wheel_pub)
            
        except KeyboardInterrupt:
            break
        except ValueError:
            rospy.logwarn("⚠️ Invalid input, try again.")
            continue

if __name__ == "__main__":
    main()