# OpenCR 6-Motor Firmware Solution

## Problem Identified
Your current OpenCR firmware (`improved_seiral_comms_ros2.ino`) only supports motors 1-4 for wheel control. Motors 5 (claw) and 6 (beacon) were trying to connect directly via `/dev/ttyACM0`, but this caused conflicts since the OpenCR board was already using that port.

## Solution
Since motors 5 and 6 are connected to the same TTL Dynamixel bus as the wheel motors (just not daisy-chained), we can control all 6 motors through the OpenCR board using a unified firmware approach.

## New Architecture

### OpenCR Firmware (`opencr_6motor_firmware.ino`)
- **Motors 1-4**: Velocity control via `/wheel_velocities` topic (unchanged)
- **Motor 5 (Beacon)**: Position control via `/beacon_drop` topic  
- **Motor 6 (Claw)**: Position control via `/claw_control` topic

### Updated Python Nodes
- **`claw_motor_node.py`**: Now publishes to `/claw_control` instead of direct Dynamixel communication
- **`beacon_motor_node.py`**: Now publishes to `/beacon_drop` instead of direct Dynamixel communication
- **`ik_velocity2_node.py`**: Unchanged, still publishes to `/wheel_velocities`

## Upload Instructions

### 1. Upload New Firmware to OpenCR
```bash
# In Arduino IDE:
# 1. Open opencr_6motor_firmware.ino
# 2. Select Tools > Board > OpenCR Board
# 3. Select correct Port (usually /dev/ttyACM0)
# 4. Click Upload
```

### 2. Verify Upload Success
After upload, open Serial Monitor (115200 baud). You should see:
```
Starting 6-motor OpenCR control...
Wheel Motor 1 OK  (or FAILED if not connected)
Wheel Motor 2 OK
Wheel Motor 3 OK  
Wheel Motor 4 OK
Beacon Motor 5 OK
Claw Motor 6 OK
Ready for commands!
```

### 3. Test the System
```bash
# Start all nodes
./start_robot.sh

# Test claw (True=close, False=open)
rostopic pub /claw_control std_msgs/Bool "data: true"
rostopic pub /claw_control std_msgs/Bool "data: false"

# Test beacon drop
rostopic pub /beacon_drop std_msgs/Bool "data: true"

# Test movement (still works the same)
rostopic pub /wheel_velocities std_msgs/Float32MultiArray "data: [1.0, -1.0, 1.0, -1.0, 2.0]"
```

## Benefits
1. **Single communication channel**: All motors controlled through OpenCR
2. **No port conflicts**: Only OpenCR uses `/dev/ttyACM0`
3. **Reliable communication**: Unified ROS-based control
4. **Backwards compatible**: Wheel movement commands unchanged

## Position Values (Adjust as needed)
- **Claw Open**: 1024
- **Claw Closed**: 2048  
- **Beacon Up**: 1024
- **Beacon Down**: 2048

You can tune these values in the firmware based on your actual motor positions.
