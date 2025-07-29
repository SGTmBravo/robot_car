#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32MultiArray
import time
import math
import signal
import sys

# === GLOBAL STATE FLAGS ===
start_triggered = False
limit_triggered = False

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

# Robot parameters (same as linear_calibration.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.075   # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor from linear_calibration.py

# Global publisher for wheel velocities (matching linear_calibration system)
wheel_pub = None

# === EMERGENCY SHUTDOWN HANDLER ===
def emergency_shutdown(signum, frame):
    """Handle Ctrl+C and emergency shutdown - reset claw before exit"""
    rospy.logwarn("🚨 EMERGENCY SHUTDOWN DETECTED!")
    rospy.logwarn("🤖 Resetting claw position before exit...")
    
    try:
        # Stop all movement immediately
        if wheel_pub:
            stop_robot()
        
        # Reset claw to open position
        claw_pub = rospy.Publisher('/claw_control', Bool, queue_size=1)
        rospy.sleep(0.5)  # Allow publisher to establish connection
        claw_pub.publish(False)  # Open claw
        rospy.logwarn("🤖 Claw reset to OPEN position")
        rospy.sleep(1.5)  # Wait for claw to open
        
        rospy.logwarn("🛑 Emergency shutdown complete - robot safe")
        
    except Exception as e:
        rospy.logerr(f"❌ Error during emergency shutdown: {e}")
    
    finally:
        sys.exit(0)

# Register the signal handler for Ctrl+C
signal.signal(signal.SIGINT, emergency_shutdown)

# === MOVEMENT FUNCTION USING LINEAR_CALIBRATION SYSTEM ===
def move_robot_inches(x_inches, y_inches, theta):
    """Move robot using inch measurements - same system as linear_calibration.py"""
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
        
        omega = 0  # No rotation during linear movement
        motor1_fr = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
        motor2_fl = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
        motor3_rl = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
        motor4_rr = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right
        
        rospy.loginfo(f"   🏃 Linear: FR={motor1_fr:.2f}, FL={motor2_fl:.2f}, RL={motor3_rl:.2f}, RR={motor4_rr:.2f}, time={linear_time:.2f}s")
        
        # Execute movement
        msg = Float32MultiArray()
        msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, linear_time]
        wheel_pub.publish(msg)
        rospy.sleep(linear_time + 0.5)
    
    # Then do rotation if needed
    if theta != 0:
        rospy.loginfo(f"   🔄 Rotation: θ={theta:.3f} rad")
        angular_speed = 0.2  # rad/s
        angular_time = abs(theta) / angular_speed
        
        vx = 0
        vy = 0
        omega = angular_speed if theta > 0 else -angular_speed
        
        # CORRECT X-configuration mecanum rotation (same as ik_velocity2_node2.py)
        # Motors 1&2 same direction, motors 3&4 opposite direction
        motor1_fr = omega * L / WHEEL_RADIUS  # Front Right
        motor2_fl = omega * L / WHEEL_RADIUS  # Front Left
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
    """Send stop command"""
    rospy.loginfo("🛑 Stopping robot")
    msg = Float32MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.1]  # All motors stop
    wheel_pub.publish(msg)
    rospy.sleep(0.1)

# === CALLBACKS ===
def start_led_callback(msg):
    global start_triggered
    if msg.data and not start_triggered:  # Only trigger once - edge detection
        rospy.loginfo("🟢 Start LED detected - TRIGGERING SEQUENCE!")
        start_triggered = True

def limit_switch_callback(msg):
    global limit_triggered
    if msg.data and not limit_triggered:  # Only trigger once - edge detection
        rospy.loginfo("🔴 Limit switch triggered - CONTINUING SEQUENCE!")
        limit_triggered = True

# === MAIN STATE MACHINE ===
def main():
    global start_triggered, limit_triggered, wheel_pub
    
    rospy.init_node("master_node")
    
    # Use same system as linear_calibration.py - direct wheel velocities
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    
    rospy.loginfo("🤖 Master State Machine Ready (INCHES + /wheel_velocities)")
    rospy.loginfo("📏 Using linear_calibration motor system with inch measurements")
    rospy.loginfo("🚨 Press Ctrl+C at any time to emergency stop and reset claw")

    rospy.Subscriber("/start_signal", Bool, start_led_callback)
    rospy.Subscriber("/limit_switch_triggered", Bool, limit_switch_callback)

    rate = rospy.Rate(10)

    # === Wait for start LED ===
    rospy.loginfo("⏳ Waiting for start LED...")
    while not start_triggered and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("🚗 Starting autonomous sequence...")

    # === STEP 1: Localize at White 2 ===
    move_robot_inches(0.0, 28.0, 0.0)
    move_robot_inches(0.0, -1.2, 0.0)
    move_robot_inches(15.0, 0.0, 0.0)
    move_robot_inches(0.0, 0.5, 0.0)
    move_robot_inches(0.0, 0.0, -0.11)

    # === STEP 2: Start moving along wall until limit switch triggers ===
    rospy.loginfo("🚗 Moving along wall - limit switch will stop movement...")
    
    # SLOWER SPEED for wall following - adjustable parameter
    WALL_FOLLOW_SPEED = 0.03  # m/s - Much slower than normal MOTOR_SPEED (0.075)
    rospy.loginfo(f"🐌 Using slower wall-following speed: {WALL_FOLLOW_SPEED:.3f} m/s")
    
    # Start continuous movement along the wall (using wheel velocities)
    # Move forward along wall in +X direction at slower speed
    vx = WALL_FOLLOW_SPEED  # Slower forward movement along wall
    vy = 0.0               # No sideways movement
    omega = 0.0
    
    motor1_fr = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
    motor2_fl = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
    motor3_rl = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
    motor4_rr = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right
    
    msg = Float32MultiArray()
    msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, 0.1]  # Short duration for continuous
    
    # Keep moving until limit switch triggers
    while not limit_triggered and not rospy.is_shutdown():
        wheel_pub.publish(msg)
        rospy.sleep(0.1)
    
    # Limit switch triggered - stop immediately
    stop_robot()
    rospy.loginfo("🔴 Limit switch hit - adjusting position...")
    
    # Small adjustment move to align properly for beacon drop
    move_robot_inches(-1.0, 0.0, 0.0)  # Small 0.5" adjustment

    # === STEP 3: Drop beacon ===
    rospy.loginfo("🟢 Dropping beacon...")
    beacon_pub = rospy.Publisher('/beacon_drop', Bool, queue_size=1)
    rospy.sleep(0.5)  # Allow publisher to establish connection
    beacon_pub.publish(True)
    rospy.loginfo("🟢 Beacon drop command sent, waiting for completion...")
    rospy.sleep(2.0)
    
    # Optional: Retract beacon arm (uncomment if desired)
    # rospy.loginfo("🔵 Retracting beacon arm...")
    # beacon_pub.publish(False)
    # rospy.sleep(1.5)

    # === STEP 4: Into the cave and back to White 4 ===
    # Original: 0.23m → Precise: 9.055"
    move_robot_inches(0.0, 0.0, -0.08)
    move_robot_inches(0.0, -65.0, 0)
    move_robot_inches(0.0, 21.5, 0)

    # === STEP 5: Localize at White 5 ===
    # Original: -0.15m, -0.08m → Precise: -5.906", -3.15"
    move_robot_inches(21.5, 0.0, 0.0)
    move_robot_inches(-1.0, 0.0, 0.07)

    # === STEP 5.5: Retract beacon arm ===
    rospy.loginfo("🔵 Retracting beacon arm...")
    beacon_pub = rospy.Publisher('/beacon_drop', Bool, queue_size=1)
    rospy.sleep(0.5)  # Allow publisher to establish connection
    beacon_pub.publish(False)  # Retract beacon arm
    rospy.sleep(1.5)

    # === STEP 6: Push container to left wall (White 6) ===
    # Original: 0.52m → Precise: 20.472"
    move_robot_inches(0.0, 37.0, 0.12)
    move_robot_inches(1.5, 0.0, 0)
    move_robot_inches(-1.0, 0.0, 0)

    # === STEP 7: Return to White 5 ===
    move_robot_inches(0.0, -39.0, 0.0)
    move_robot_inches(4.5, 0.0, 0.0)
    move_robot_inches(0.0, 1.0, 0)

    # === STEP 8: Move to second container ===
    move_robot_inches(-27.1, 0.0, 0)

    # === STEP 9: Grab container at White 7 and drag back ===
    rospy.loginfo("🤖 Closing claw to grab container...")
    claw_pub = rospy.Publisher('/claw_control', Bool, queue_size=1)
    rospy.sleep(0.5)  # Allow publisher to establish connection
    claw_pub.publish(True)  # Close claw to grab
    rospy.sleep(2.0)
    #  === STEP 10: Move back with container
    #move_robot_inches(0.0, 0.0, 0.0)
    move_robot_inches(0.0, 8.0, 0.0)
    #move_robot_inches(0.0, 0.0, 0.45)
    move_robot_inches(40, 40, 0)
    #move_robot_inches(0.0, 4.0, 0.0)
    move_robot_inches(0.0, 0.0, -1.9)
    #move_robot_inches(-12.598, 0.0, 0)

    # Reset claw position for next run
    rospy.loginfo("🤖 Opening claw to reset position for next run...")
    claw_pub.publish(False)  # Open claw to reset
    rospy.sleep(2.0)

    rospy.loginfo("🏁 Task sequence complete - Claw reset for next run")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("🛑 ROS shutdown detected")
        # Emergency shutdown will handle claw reset
        emergency_shutdown(None, None)
    except KeyboardInterrupt:
        rospy.logwarn("🛑 Keyboard interrupt detected")
        # Emergency shutdown will handle claw reset
        emergency_shutdown(None, None)
    except Exception as e:
        rospy.logerr(f"❌ Unexpected error: {e}")
        # Reset claw even on unexpected errors
        try:
            claw_pub = rospy.Publisher('/claw_control', Bool, queue_size=1)
            rospy.sleep(0.5)
            claw_pub.publish(False)
            rospy.logwarn("🤖 Claw reset due to error")
            rospy.sleep(1.5)
        except:
            pass
