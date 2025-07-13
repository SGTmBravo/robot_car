#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

# === GLOBAL STATE FLAGS ===
start_triggered = False
limit_triggered = False

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

# Global publisher for cmd_vel (current system)
cmd_vel_pub = None

# === SIMPLE MOVEMENT FUNCTION USING EXISTING SYSTEM ===
def move_robot_inches(x_inches, y_inches, theta):
    """Move robot using inch measurements - works with current /cmd_vel system"""
    # Convert inches to meters
    x_meters = x_inches * INCHES_TO_METERS
    y_meters = y_inches * INCHES_TO_METERS
    
    rospy.loginfo(f"📦 Moving: x={x_inches:.1f}\" ({x_meters:.3f}m), y={y_inches:.1f}\" ({y_meters:.3f}m), θ={theta:.3f} rad")
    
    # Create velocity command for current system
    cmd = Twist()
    
    # Simple approach: move at fixed speed for calculated time
    speed = 0.05  # m/s - conservative speed
    angular_speed = 0.2  # rad/s
    
    # Calculate time needed
    linear_dist = (x_meters**2 + y_meters**2)**0.5
    angular_dist = abs(theta)
    
    linear_time = linear_dist / speed if linear_dist > 0 else 0
    angular_time = angular_dist / angular_speed if angular_dist > 0 else 0
    
    # Do linear movement first, then rotation
    if linear_dist > 0:
        cmd.linear.x = (x_meters / linear_dist) * speed if linear_dist > 0 else 0
        cmd.linear.y = (y_meters / linear_dist) * speed if linear_dist > 0 else 0
        cmd.angular.z = 0
        
        rospy.loginfo(f"   🏃 Linear: vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f} for {linear_time:.2f}s")
        cmd_vel_pub.publish(cmd)
        rospy.sleep(linear_time)
    
    # Then do rotation
    if angular_dist > 0:
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.angular.z = angular_speed if theta > 0 else -angular_speed
        
        rospy.loginfo(f"   🔄 Angular: ω={cmd.angular.z:.3f} for {angular_time:.2f}s")
        cmd_vel_pub.publish(cmd)
        rospy.sleep(angular_time)
    
    # Stop
    stop_robot()
    rospy.loginfo("✅ Step complete")

def stop_robot():
    """Send stop command"""
    rospy.loginfo("🛑 Stopping robot")
    cmd = Twist()  # All zeros
    cmd_vel_pub.publish(cmd)
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
    global start_triggered, limit_triggered, cmd_vel_pub
    
    rospy.init_node("master_node")
    
    # Use existing /cmd_vel system (don't break what's working!)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("🤖 Master State Machine Ready (INCHES + /cmd_vel)")
    rospy.loginfo("📏 Using existing motor system with inch measurements")

    rospy.Subscriber("/start_signal", Bool, start_led_callback)
    rospy.Subscriber("/limit_switch_triggered", Bool, limit_switch_callback)

    rate = rospy.Rate(10)

    # === Wait for start LED ===
    rospy.loginfo("⏳ Waiting for start LED...")
    while not start_triggered and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("🚗 Starting autonomous sequence...")

    # === STEP 1: Localize at White 2 ===
    # Original: -0.08m, 0.08m → Precise: -3.15", 3.15"
    move_robot_inches(-3.15, 3.15, 0)

    # === STEP 2: Start moving along wall until limit switch triggers ===
    rospy.loginfo("🚗 Moving along wall - limit switch will stop movement...")
    
    # Start continuous movement along the wall
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.linear.y = 0.05  # Move left along wall at steady speed
    cmd.angular.z = 0.0
    cmd_vel_pub.publish(cmd)
    
    # Keep moving until limit switch triggers
    while not limit_triggered and not rospy.is_shutdown():
        rate.sleep()
    
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
    move_robot_inches(0.0, 9.055, 0)
    move_robot_inches(0.0, -9.055, 0)

    # === STEP 5: Localize at White 5 ===
    # Original: -0.15m, -0.08m → Precise: -5.906", -3.15"
    move_robot_inches(-5.906, -3.15, 0)

    # === STEP 6: Push container to left wall (White 6) ===
    # Original: 0.52m → Precise: 20.472"
    move_robot_inches(0.0, 20.472, 0)

    # === STEP 7: Return to White 5 ===
    move_robot_inches(0.0, -20.472, 0)

    # === STEP 8: Grab container at White 7 and drag back ===
    rospy.loginfo("🤖 Lowering claw to grab container at White 7...")
    # Original: 0.32m → Precise: 12.598"
    move_robot_inches(12.598, 0.0, 0)
    move_robot_inches(-12.598, 0.0, 0)

    rospy.loginfo("🏁 Task sequence complete")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
