# Robot Car Project

## 🤖 Mecanum Wheel Robot Control

This repository contains the working code for a mecanum-wheeled robot with smooth movement control.

### ✅ **WORKING FEATURES:**
- **Perfect rotation control** - CCW and CW rotation with accurate angle control
- **Smooth linear movement** - x, y movement with constant velocity
- **Combined movement** - Can handle simultaneous linear + rotational movement
- **Mathematical input parsing** - Accepts `pi`, `pi/2`, `pi/4`, `2*pi`, etc.

### 🚀 **Quick Start:**

**Terminal 1 (Start ROS communication):**
```bash
cd /home/ubuntu/robot_car_ws/src/robot_car/scripts
./start_simple.sh
```

**Terminal 2 (Control the robot):**
```bash
cd /home/ubuntu/robot_car_ws/src/robot_car/scripts
python3 ik_velocity2_node.py
```

### 📁 **Key Files:**
- `ik_velocity2_node.py` - Main control node (WORKING!)
- `smooth_motor_control.ino` - Arduino code for motor control
- `start_simple.sh` - Simple startup script for development
- `start_robot.sh` - Full system startup (for autonomous mode later)

### 🎯 **Rotation Patterns:**
- **CCW (positive theta):** `[+1.0, +1.0, -1.0, -1.0]` (Motors: FR, FL, RL, RR)
- **CW (negative theta):** `[-1.0, -1.0, +1.0, +1.0]`
- **Constant velocity:** Theta magnitude affects TIME, not SPEED

### 📝 **Development Notes:**
- Fixed choppy rotation by correcting motor direction patterns
- Implemented angle correction factor (57.3°/90° ≈ 0.637) for accurate rotation
- Uses simplified rotation logic instead of complex kinematics for reliability
- All debug/test files cleaned up, workspace organized

### 🔧 **Hardware Setup:**
- OpenCR board with 4 Dynamixel motors
- Mecanum wheels in standard configuration
- ROS communication via rosserial

---
*Last updated: [Current date] - Robot working perfectly with smooth rotation!*
