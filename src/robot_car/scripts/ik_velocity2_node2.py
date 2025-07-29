#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray, Bool

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

# Robot parameters (same as master_state_machine_node.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.075       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor from linear_calibration.py

# Global publishers (matching master_state_machine_node.py)
wheel_pub = None
claw_pub = None
beacon_pub = None

def move_robot_inches(x_inches, y_inches, theta):
    """Move robot using inch measurements - EXACT SAME as master_state_machine_node.py"""
    # Convert inches to meters
    x_meters = x_inches * INCHES_TO_METERS
    y_meters = y_inches * INCHES_TO_METERS
    
    rospy.loginfo(f"📦 Moving: x={x_inches:.1f}\" ({x_meters:.3f}m), y={y_inches:.1f}\" ({y_meters:.3f}m), θ={theta:.3f} rad")
    
    # Do linear movement first (using linear_calibration approach)
    if x_meters != 0 or y_meters != 0:
        linear_dist = math.hypot(x_meters, y_meters)
        linear_time = (linear_dist / MOTOR_SPEED) * DISTANCE_SCALE
        
        # Calculate direction and motor commands (same as linear_calibration.py)
        vx = (x_meters / linear_dist) * MOTOR_SPEED
        vy = (y_meters / linear_dist) * MOTOR_SPEED
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
            rospy.loginfo(f"   🔄 Rotation: θ={theta:.3f} rad")
            angular_speed = 0.2  # rad/s
            angular_time = abs(theta) / angular_speed
            
            vx = 0
            vy = 0
            omega = angular_speed if theta > 0 else -angular_speed
            
            # CORRECT X-configuration mecanum rotation (wheels at 45° pointing inward)
            # Motors 1&2 correct, fix motors 3&4 to move in same direction
            motor1_fr = omega * L / WHEEL_RADIUS  # Front Right - same direction
            motor2_fl = omega * L / WHEEL_RADIUS  # Front Left - same direction
            motor3_rl = -omega * L / WHEEL_RADIUS  # Rear Left - REVERSED 
            motor4_rr = -omega * L / WHEEL_RADIUS  # Rear Right - REVERSED
            
            msg = Float32MultiArray()
            msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, angular_time]
            wheel_pub.publish(msg)
            rospy.sleep(angular_time + 0.5)
        
    # Stop
    stop_robot()
    rospy.loginfo("✅ Step complete")

def stop_robot():
    """Send stop command - EXACT SAME as master_state_machine_node.py"""
    rospy.loginfo("🛑 Stopping robot")
    msg = Float32MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.1]  # All motors stop
    wheel_pub.publish(msg)
    rospy.sleep(0.1)

def control_claw(close_claw):
    """Control the claw - True to close, False to open"""
    global claw_pub
    rospy.loginfo(f"🤖 {'Closing' if close_claw else 'Opening'} claw...")
    claw_pub.publish(Bool(data=close_claw))
    rospy.sleep(2.0)  # Wait for movement to complete

def control_beacon(drop_beacon):
    """Control the beacon - True to drop, False to retract"""
    global beacon_pub
    rospy.loginfo(f"🟢 {'Dropping' if drop_beacon else 'Retracting'} beacon...")
    beacon_pub.publish(Bool(data=drop_beacon))
    rospy.sleep(2.0)  # Wait for movement to complete

def main():
    global wheel_pub, claw_pub, beacon_pub
    
    rospy.init_node('ik_velocity_node')
    
    # Publishers (same as master_state_machine_node.py)
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    claw_pub = rospy.Publisher('/claw_control', Bool, queue_size=1)
    beacon_pub = rospy.Publisher('/beacon_drop', Bool, queue_size=1)
    rospy.sleep(0.5)  # Allow publishers to establish connection
    
    rospy.loginfo("✅ IK velocity node ready - MOVEMENT + CLAW + BEACON CONTROL!")
    rospy.loginfo("📋 Commands:")
    rospy.loginfo("   - Movement: Enter x, y, theta values")
    rospy.loginfo("   - Claw: Type 'claw open' or 'claw close'")
    rospy.loginfo("   - Beacon: Type 'beacon drop' or 'beacon up'")
    rospy.loginfo("   - Quit: Type 'q' or 'quit'")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("\nEnter command: ").strip().lower()
            
            # Quit commands
            if user_input in ['q', 'quit', 'exit']:
                break
            
            # Claw commands
            elif user_input in ['claw close', 'close claw', 'claw']:
                control_claw(True)
                continue
            elif user_input in ['claw open', 'open claw', 'open']:
                control_claw(False)
                continue
            
            # Beacon commands
            elif user_input in ['beacon drop', 'drop beacon', 'beacon down', 'drop']:
                control_beacon(True)
                continue
            elif user_input in ['beacon up', 'beacon retract', 'retract beacon', 'up']:
                control_beacon(False)
                continue
            
            # Movement commands (try to parse as number)
            else:
                try:
                    x_inches = float(user_input)
                    y_inches = float(input("Enter y distance (INCHES): "))
                    theta = float(input("Enter θ rotation (rad): "))
                    
                    # Use the EXACT SAME function as master_state_machine_node.py
                    move_robot_inches(x_inches, y_inches, theta)
                except ValueError:
                    rospy.logwarn("⚠️ Invalid command. Try:")
                    rospy.logwarn("   - Number for movement (x distance)")
                    rospy.logwarn("   - 'claw open' or 'claw close'")
                    rospy.logwarn("   - 'beacon drop' or 'beacon up'")
                    rospy.logwarn("   - 'q' to quit")
                    continue
                    
        except KeyboardInterrupt:
            break
        except Exception as e:
            rospy.logwarn(f"⚠️ Error: {e}")
            continue

if __name__ == "__main__":
    main()
