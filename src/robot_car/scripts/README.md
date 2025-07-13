# Robot Car Scripts Organization

## Main Scripts Folder (`/scripts/`)
**Essential, actively used nodes:**

### Core Movement & Control:
- `ik_velocity2_node.py` - Main movement control system
- `master_state_machine_node.py` - Autonomous sequence coordinator (uses inches internally)

### Sensors:
- `limit_switch_node.py` - Limit switch sensor node
- `photoresistor_node.py` - Light sensor node (publishes `/start_signal`)

### Calibration & Testing:
- `linear_calibration.py` - Movement accuracy calibration tool (meters)

### Support:
- `beacon_motor_node.py` - Beacon drop motor control
- `power_monitor.py` - System monitoring
- `start_robot.sh` - Main system startup script

---

## Scripts2 Folder (`../scripts2/`)
**Inch-based versions and utilities:**

- `ik_velocity_inches_node.py` - Inch-based movement control (alternative)
- `linear_calibration_cmd_vel.py` - Calibration using /cmd_vel with inches
- `linear_calibration_inches.py` - Calibration using /wheel_velocities with inches
- `inch_conversion_guide.py` - Conversion reference guide
- `unit_converter.py` - Unit conversion utilities

---

## Archive Folder (`archive/`)
**Unused/old files:**

- `start_simple.sh` - Basic startup (not used)
- `photoresistor_enhanced.py` & `photoresistor_enhanced_node.py` - Enhanced versions (not used)
- `kinematics_node.py` - Kinematics calculations (not referenced)

---

## Hardware Folder (`hardware/`)
**Arduino sketches:**

- `smooth_motor_control.ino` - Motor control firmware
- `smooth_motor_control_with_battery.ino` - Motor control with battery monitoring

---

## System Dependencies

**Master State Machine Uses:**
- `/start_signal` topic ← `photoresistor_node.py`
- `/limit_switch_triggered` topic ← `limit_switch_node.py`
- `/cmd_vel` topic → `ik_velocity2_node.py`

**Current Working System:**
1. `start_robot.sh` - launches basic communication
2. `photoresistor_node.py` - detects start LED
3. `limit_switch_node.py` - detects wall contact
4. `ik_velocity2_node.py` - handles movement commands
5. `master_state_machine_node.py` - coordinates autonomous sequence
