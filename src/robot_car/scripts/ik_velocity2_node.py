#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray

# Robot parameters
WHEEL_RADIUS = 0.02825
L = 0.075

# MOTOR SPEED SETTINGS - Adjusted to match working test values
# The working test script uses [-0.5, 0.5, 0.5, -0.5] for smooth rotation
ROBOT_SPEED = 0.05   # m/s - Linear speed
ROBOT_OMEGA = 0.1883  # rad/s - Calculated to match working test rotation values

def main():
    rospy.init_node('ik_velocity_node')
    
    # Publisher for wheel velocities
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    
    rospy.loginfo("✅ ROS velocity node ready")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter x distance (m) or 'q' to quit: ")
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
            x = float(user_input)
            y = float(input("Enter y distance (m): "))
            
            theta_input_str = input("Enter θ rotation (rad) [pi, pi/2, pi/4, 2*pi, -pi, etc.]: ")
            # Handle special cases and mathematical expressions
            if theta_input_str.lower() == 'pi':
                theta_input = math.pi
            elif theta_input_str.lower() == '-pi':
                theta_input = -math.pi
            elif theta_input_str.lower() == 'pi/2':
                theta_input = math.pi / 2
            elif theta_input_str.lower() == '-pi/2':
                theta_input = -math.pi / 2
            elif theta_input_str.lower() == 'pi/4':
                theta_input = math.pi / 4
            elif theta_input_str.lower() == '-pi/4':
                theta_input = -math.pi / 4
            elif theta_input_str.lower() == '2*pi' or theta_input_str.lower() == '2pi':
                theta_input = 2 * math.pi
            elif theta_input_str.lower() == '-2*pi' or theta_input_str.lower() == '-2pi':
                theta_input = -2 * math.pi
            else:
                theta_input = float(theta_input_str)
            
            # Limit theta to -2π to 2π
            theta = max(-2*math.pi, min(2*math.pi, theta_input))
            if theta != theta_input:
                rospy.logwarn(f"⚠️ Theta limited to {theta:.2f} rad (was {theta_input:.2f})")
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Interrupted by user")
            break
        except ValueError:
            rospy.logwarn("⚠️ Invalid input, try again.")
            continue

        # Calculate duration - scale rotation time with theta magnitude
        linear_dist = math.hypot(x, y)
        angular_dist = abs(theta)
        
        linear_time = linear_dist / ROBOT_SPEED if linear_dist > 0 else 0
        
        # Scale rotation time to match actual angle rotation
        # Since theta=1 rad (~57.3°) was rotating ~90°, we need to adjust
        # If 1 rad rotates 90°, then to rotate 1 rad (57.3°) we need: (57.3/90) of the time
        if angular_dist > 0:
            # Adjusted scaling: accounts for the fact that our rotation is faster than expected
            angle_correction = 57.3 / 90.0  # ~0.637 - scales down duration since we rotate faster
            angular_time = 2.0 * (angular_dist / 0.5) * angle_correction
        else:
            angular_time = 0
            
        duration = max(1.0, linear_time, angular_time)  # Minimum 1 second

        # SIMPLIFIED: Use the exact same approach as the working test script
        # instead of complex kinematic calculations that might have errors
        
        if linear_dist > 0:
            vx = (x / linear_dist) * ROBOT_SPEED  # Direction * constant speed
            vy = (y / linear_dist) * ROBOT_SPEED
        else:
            vx = vy = 0
            
        # For rotation, use the proven test script patterns with CONSTANT velocity
        if theta != 0:
            # INCREASED: Faster rotation for better testing (was 0.5)
            rotation_strength = 1.0  # Doubled for faster rotation testing
            
            if theta > 0:  # Counter-clockwise
                # FIXED: Corrected motor directions for proper CCW rotation
                motor1_fr = rotation_strength   # Motor 1: FLIPPED (was negative)
                motor2_fl = rotation_strength   # Motor 2: Same
                motor3_rl = -rotation_strength  # Motor 3: FLIPPED (was positive)
                motor4_rr = -rotation_strength  # Motor 4: Same
            else:  # Clockwise
                # Reverse the pattern for CW rotation
                motor1_fr = -rotation_strength  # Motor 1: Opposite of CCW
                motor2_fl = -rotation_strength  # Motor 2: Opposite of CCW
                motor3_rl = rotation_strength   # Motor 3: Opposite of CCW
                motor4_rr = rotation_strength   # Motor 4: Opposite of CCW
        else:
            # No rotation - use kinematic equations for linear movement
            omega = 0
            motor1_fr = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
            motor2_fl = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
            motor3_rl = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
            motor4_rr = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right

        # Create and publish message
        msg = Float32MultiArray()
        msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, duration]
        
        wheel_pub.publish(msg)
        rospy.loginfo(f"📤 Published: M1(FR)={motor1_fr:.2f} M2(FL)={motor2_fl:.2f} M3(RL)={motor3_rl:.2f} M4(RR)={motor4_rr:.2f}, duration={duration:.2f}s")
        
        # Wait for movement to complete
        rospy.sleep(duration + 0.5)
        
        # Send multiple stop commands to ensure clean stopping
        for _ in range(3):
            stop_msg = Float32MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0, 0.0, 0.1]
            wheel_pub.publish(stop_msg)
            rospy.sleep(0.1)
        rospy.loginfo("🛑 Motors stopped")

if __name__ == "__main__":
    main()