#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray

# Robot parameters (same as linear_calibration.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.05       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor: measured 1.6477m for 1.0m target, so scale = 1.0/1.6477

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

        # Use calibrated movement system (same as linear_calibration.py)
        xy_dist = math.hypot(x, y)
        theta_dist = abs(theta)

        # Linear movement duration using calibrated parameters
        if xy_dist > 0:
            linear_time = (xy_dist / MOTOR_SPEED) * DISTANCE_SCALE
            vx = (x / xy_dist) * MOTOR_SPEED
            vy = (y / xy_dist) * MOTOR_SPEED
        else:
            linear_time = 0
            vx = 0
            vy = 0

        # Angular movement (simple approach for now)
        if theta_dist > 0:
            angular_speed = 0.2  # rad/s
            angular_time = theta_dist / angular_speed
            omega = angular_speed if theta > 0 else -angular_speed
        else:
            angular_time = 0
            omega = 0

        # Total duration (do linear first, then angular)
        total_duration = linear_time + angular_time

        rospy.loginfo(f"📐 Distances: xy={xy_dist:.3f}m, θ={theta_dist:.3f}rad")
        rospy.loginfo(f"⏱️  Times: linear={linear_time:.2f}s, angular={angular_time:.2f}s, total={total_duration:.2f}s")

        # Do linear movement first
        if xy_dist > 0:
            # Mecanum wheel inverse kinematics for linear movement
            omega_linear = 0  # No rotation during linear movement
            v_fl = (vx - vy - L * omega_linear) / WHEEL_RADIUS  # Front Left
            v_fr = (vx + vy + L * omega_linear) / WHEEL_RADIUS  # Front Right  
            v_rl = (vx + vy - L * omega_linear) / WHEEL_RADIUS  # Rear Left
            v_rr = (vx - vy + L * omega_linear) / WHEEL_RADIUS  # Rear Right

            # Create and publish linear movement message
            msg = Float32MultiArray()
            msg.data = [v_fl, v_fr, v_rl, v_rr, linear_time]
            wheel_pub.publish(msg)
            rospy.loginfo(f"📤 Linear: FL={v_fl:.2f} FR={v_fr:.2f} RL={v_rl:.2f} RR={v_rr:.2f}, time={linear_time:.2f}s")
            
            # Wait for linear movement to complete
            rospy.sleep(linear_time + 0.5)

        # Then do angular movement
        if theta_dist > 0:
            # Mecanum wheel inverse kinematics for rotation in place
            vx_rot = 0  # No linear movement during rotation
            vy_rot = 0
            v_fl = (vx_rot - vy_rot - L * omega) / WHEEL_RADIUS  # Front Left
            v_fr = (vx_rot + vy_rot + L * omega) / WHEEL_RADIUS  # Front Right  
            v_rl = (vx_rot + vy_rot - L * omega) / WHEEL_RADIUS  # Rear Left
            v_rr = (vx_rot - vy_rot + L * omega) / WHEEL_RADIUS  # Rear Right

            # Create and publish angular movement message
            msg = Float32MultiArray()
            msg.data = [v_fl, v_fr, v_rl, v_rr, angular_time]
            wheel_pub.publish(msg)
            rospy.loginfo(f"📤 Angular: FL={v_fl:.2f} FR={v_fr:.2f} RL={v_rl:.2f} RR={v_rr:.2f}, time={angular_time:.2f}s")
            
            # Wait for angular movement to complete
            rospy.sleep(angular_time + 0.5)

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()