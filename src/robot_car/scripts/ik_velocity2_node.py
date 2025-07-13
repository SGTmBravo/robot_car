#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray

# Robot parameters
WHEEL_RADIUS = 0.02825
L = 0.075

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
            theta = float(input("Enter θ rotation (rad): "))
        except KeyboardInterrupt:
            break
        except ValueError:
            rospy.logwarn("⚠️ Invalid input, try again.")
            continue

        # Compute velocity and duration
        max_speed = 2.0  # m/s
        max_omega = 1.0  # rad/s

        # Distance to move
        xy_dist = math.hypot(x, y)
        theta_dist = abs(theta)

        # Duration based on maximum linear and angular speeds
        duration = max(1.0, xy_dist / max_speed, theta_dist / max_omega)

        # Velocities (m/s and rad/s)
        vx = x / duration
        vy = y / duration
        omega = theta / duration

        # Mecanum wheel inverse kinematics
        v_fl = (vx - vy - L * omega) / WHEEL_RADIUS  # Front Left
        v_fr = (vx + vy + L * omega) / WHEEL_RADIUS  # Front Right  
        v_rl = (vx + vy - L * omega) / WHEEL_RADIUS  # Rear Left
        v_rr = (vx - vy + L * omega) / WHEEL_RADIUS  # Rear Right

        # Create and publish message
        msg = Float32MultiArray()
        msg.data = [v_fl, v_fr, v_rl, v_rr, duration]
        
        wheel_pub.publish(msg)
        rospy.loginfo(f"📤 Published: FL={v_fl:.2f} FR={v_fr:.2f} RR={v_rr:.2f} RL={v_rl:.2f}, duration={duration:.2f}s")
        
        # Wait for movement to complete
        rospy.sleep(duration + 0.5)

if __name__ == "__main__":
    main()