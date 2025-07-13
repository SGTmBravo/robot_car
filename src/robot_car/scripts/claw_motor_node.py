#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from dynamixel_sdk import *

PORT_NAME = '/dev/ttyACM0'
BAUDRATE = 57600
MOTOR_ID = 5  # Claw motor
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
PROTOCOL_VERSION = 2.0

POSITION_CLOSED = 2048   # Claw closed to grab container (adjust as needed)
POSITION_OPEN = 1024     # Claw open to release container (adjust as needed)

def init_port():
    port = PortHandler(PORT_NAME)
    if not port.openPort():
        rospy.logerr("❌ Failed to open port")
        return None
    if not port.setBaudRate(BAUDRATE):
        rospy.logerr("❌ Failed to set baudrate")
        return None
    return port

def write_position(packet, dxl_id, pos):
    packet.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos)

def claw_callback(msg):
    if msg.data:
        rospy.loginfo("🤖 Closing claw to grab container...")
        write_position(packetHandler, MOTOR_ID, POSITION_CLOSED)
        rospy.sleep(1.5)
        rospy.loginfo("✅ Claw closed - container grabbed")
    else:
        rospy.loginfo("🤖 Opening claw to release container...")
        write_position(packetHandler, MOTOR_ID, POSITION_OPEN)
        rospy.sleep(1.5)
        rospy.loginfo("✅ Claw opened - container released")

if __name__ == "__main__":
    rospy.init_node('claw_motor_node')

    portHandler = init_port()
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if portHandler is None:
        exit()

    # Enable torque
    packetHandler.write1ByteTxRx(portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, 1)

    rospy.Subscriber("/claw_control", Bool, claw_callback)
    rospy.loginfo("🤖 Ready to control claw via /claw_control (True=close, False=open)")

    rospy.spin()
    portHandler.closePort()
