#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from dynamixel_sdk import *

PORT_NAME = '/dev/ttyACM0'
BAUDRATE = 57600
MOTOR_ID = 6
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
PROTOCOL_VERSION = 2.0

POSITION_DOWN = 2048   # Adjust as needed
POSITION_UP = 1024     # Adjust as needed

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

def beacon_callback(msg):
    if msg.data:
        rospy.loginfo("🟢 Dropping beacon...")
        write_position(packetHandler, MOTOR_ID, POSITION_DOWN)
        rospy.sleep(1.5)
        write_position(packetHandler, MOTOR_ID, POSITION_UP)

if __name__ == "__main__":
    rospy.init_node('beacon_motor_node')

    portHandler = init_port()
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if portHandler is None:
        exit()

    # Enable torque
    packetHandler.write1ByteTxRx(portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, 1)

    rospy.Subscriber("/beacon_drop", Bool, beacon_callback)
    rospy.loginfo("🔧 Ready to drop beacon via /beacon_drop")

    rospy.spin()
    portHandler.closePort()
