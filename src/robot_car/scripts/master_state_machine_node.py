#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32MultiArray
import time
import math

# === GLOBAL STATE FLAGS ===
start_triggered = False
limit_triggered = False

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

# Robot parameters (same as linear_calibration.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.05       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor from linear_calibration.py

# Global publisher for wheel velocities (matching linear_calibration system)
wheel_pub = None

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
        # Simple rotation - could be improved later
        angular_speed = 0.2  # rad/s
        angular_time = abs(theta) / angular_speed
        
        # All wheels rotate in same direction for turning in place
        wheel_speed = (angular_speed * L) / WHEEL_RADIUS
        if theta > 0:  # Counter-clockwise
            motor_speeds = [-wheel_speed, wheel_speed, -wheel_speed, wheel_speed]
        else:  # Clockwise
            motor_speeds = [wheel_speed, -wheel_speed, wheel_speed, -wheel_speed]
        
        msg = Float32MultiArray()
        msg.data = motor_speeds + [angular_time]
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

def drop_beacon():
    rospy.loginfo("🟢 Dropping beacon with motor ID 6...")
    # Replace with real call to claw motor control node
    # For example: publish to /claw_command topic or call service
    time.sleep(1.0)
    rospy.loginfo("✅ Beacon dropped")

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

    rospy.Subscriber("/start_signal", Bool, start_led_callback)
    rospy.Subscriber("/limit_switch_triggered", Bool, limit_switch_callback)

    rate = rospy.Rate(10)

    # === Wait for start LED ===
    rospy.loginfo("⏳ Waiting for start LED...")
    while not start_triggered and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("🚗 Starting autonomous sequence...")

    # === STEP 1: Localize at White 2 ===
    # 
    move_robot_inches(-1.0, 25.5, 0)

    # === STEP 2: Start moving along wall until limit switch triggers ===
    rospy.loginfo("🚗 Moving along wall - limit switch will stop movement...")
    
    # Start continuous movement along the wall (using wheel velocities)
    # Move forward along wall in +X direction at MOTOR_SPEED
    vx = MOTOR_SPEED  # Forward movement along wall
    vy = 0.0          # No sideways movement
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
    move_robot_inches(0.0, 0.5, 0)  # Small 0.5" adjustment

    # === STEP 3: Drop beacon ===
    rospy.loginfo("🟢 Dropping beacon...")
    beacon_pub = rospy.Publisher('/beacon_drop', Bool, queue_size=1)
    beacon_pub.publish(True)
    rospy.sleep(2.0)

    # === STEP 4: Into the cave and back to White 4 ===
    # Original: 0.23m → Precise: 9.055"
    move_robot_inches(0.0, -73.0, 0)
    move_robot_inches(0.0, 17.0, 0)

    # === STEP 5: Localize at White 5 ===
    # Original: -0.15m, -0.08m → Precise: -5.906", -3.15"
    move_robot_inches(15, -0.5, 0)

    # === STEP 6: Push container to left wall (White 6) ===
    # Original: 0.52m → Precise: 20.472"
    move_robot_inches(0.0, 40.0, 0)

    # === STEP 7: Return to White 5 ===
    move_robot_inches(0.0, -40.0, 0)

    # === STEP 8: Move to second container ===
    move_robot_inches(-27.0, 0.0, 0)

    # === STEP 9: Grab container at White 7 and drag back ===
    rospy.loginfo("🤖 Closing claw to grab container...")
    claw_pub = rospy.Publisher('/claw_control', Bool, queue_size=1)
    claw_pub.publish(True)  # Close claw to grab
    rospy.sleep(2.0)
    #  === STEP 10: Move back with container
    move_robot_inches(43, 0.0, 0)
    #move_robot_inches(-12.598, 0.0, 0)

    #rospy.loginfo("🤖 Opening claw to release container...")
    #claw_pub.publish(False)  # Open claw to release
    #rospy.sleep(2.0)

    rospy.loginfo("🏁 Task sequence complete")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
