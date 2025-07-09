#!/bin/bash

# Robot Car Launch Script
echo "🚗 Starting Robot Car System..."

# Kill any existing processes
echo "Stopping existing processes..."
pkill -f rosserial_python
pkill -f ik_velocity2_node
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

# Start robot monitor
echo "Starting robot monitor..."
rosrun robot_car robot_monitor.py &
sleep 2

# Start velocity control node
echo "Starting velocity control node..."
rosrun robot_car ik_velocity2_node.py &

echo "🎉 All systems started!"
echo "Check the terminal outputs for status messages."
echo "To stop everything, run: pkill -f 'rosrun|roscore|rosserial'"
