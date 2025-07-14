#!/bin/bash

# Robot Car Launch Script - Auto-start compatible
echo "🚗 Starting Robot Car System..."

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /home/ubuntu/robot_car_ws/devel/setup.bash

# Set ROS environment variables
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# Kill any existing processes
echo "Stopping existing processes..."
pkill -f rosserial_python
pkill -f robot_monitor
sleep 2

# Start roscore if not running
if ! pgrep -x "roscore" > /dev/null; then
    echo "Starting roscore..."
    roscore &
    sleep 3
fi

# Start rosserial connection to OpenCR
echo "Starting ROS Serial connection..."
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600 &
sleep 3

echo "🎉 All systems started!"
echo "Motors 1-4: Wheel control via /wheel_velocities"
echo "Motor 5: Beacon control via /beacon_drop" 
echo "Motor 6: Claw control via /claw_control"
echo "Ready to run master_state_machine_node.py for autonomous sequence"
echo "To stop everything, run: pkill -f 'rosrun|roscore|rosserial|python3'"
