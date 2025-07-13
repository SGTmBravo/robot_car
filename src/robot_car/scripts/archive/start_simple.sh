#!/bin/bash

# Simple robot startup script
echo "🤖 Starting robot communication..."

# Start roscore if not running
if ! pgrep -x "roscore" > /dev/null; then
    echo "Starting roscore..."
    roscore &
    sleep 3
fi

# Start rosserial for Arduino communication
echo "Starting rosserial communication..."
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

echo "✅ Robot ready! Run your ik_velocity2_node.py in another terminal"
