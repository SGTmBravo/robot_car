#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import time

# === GLOBAL STATE FLAGS ===
start_triggered = False
limit_triggered = False

# === MOVEMENT FUNCTION STUB ===
def move_robot(x, y, theta):
    # Replace this with your actual velocity-based movement interface
    rospy.loginfo(f"📦 Moving: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
    time.sleep(0.4)  # Small movement step
    rospy.loginfo("✅ Step complete")

def stop_robot():
    rospy.loginfo("🛑 Stopping robot")
    move_robot(0, 0, 0)

def drop_beacon():
    rospy.loginfo("🟢 Dropping beacon with motor ID 6...")
    # Replace with real call to claw motor control node
    # For example: publish to /claw_command topic or call service
    time.sleep(1.0)
    rospy.loginfo("✅ Beacon dropped")

# === CALLBACKS ===
def start_led_callback(msg):
    global start_triggered
    if msg.data:
        rospy.loginfo("🟢 Start LED detected")
        start_triggered = True

def limit_switch_callback(msg):
    global limit_triggered
    if msg.data:
        rospy.loginfo("🔴 Limit switch triggered")
        limit_triggered = True

# === MAIN STATE MACHINE ===
def main():
    global start_triggered, limit_triggered
    rospy.init_node("master_node")

    rospy.Subscriber("/start_signal", Bool, start_led_callback)
    rospy.Subscriber("/limit_switch_triggered", Bool, limit_switch_callback)

    rate = rospy.Rate(10)

    # === Wait for start LED ===
    rospy.loginfo("⏳ Waiting for start LED...")
    while not start_triggered and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("🚗 Starting autonomous sequence...")

    # === STEP 1: Localize at White 2 ===
    move_robot(-0.08, 0.08, 0)

    # === STEP 2: Move along wall until limit switch ===
    move_robot(0.0, 0.5, 0)
    rospy.loginfo("⏳ Waiting for limit switch near White 3...")
    while not limit_triggered and not rospy.is_shutdown():
        rate.sleep()

    stop_robot()

    # === STEP 3: Drop beacon ===
    rospy.loginfo("🟢 Dropping beacon...")
    beacon_pub = rospy.Publisher('/beacon_drop', Bool, queue_size=1)
    beacon_pub.publish(True)
    rospy.sleep(2.0)

    # === STEP 4: Into the cave and back to White 4 ===
    move_robot(0.0, 0.23, 0)
    move_robot(0.0, -0.23, 0)

    # === STEP 5: Localize at White 5 ===
    move_robot(-0.15, -0.08, 0)

    # === STEP 6: Push container to left wall (White 6) ===
    move_robot(0.0, 0.52, 0)

    # === STEP 7: Return to White 5 ===
    move_robot(0.0, -0.52, 0)

    # === STEP 8: Grab container at White 7 and drag back ===
    rospy.loginfo("🤖 Lowering claw to grab container at White 7...")
    move_robot(0.32, 0.0, 0)
    move_robot(-0.32, 0.0, 0)

    rospy.loginfo("🏁 Task sequence complete")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
