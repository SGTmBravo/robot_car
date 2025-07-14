# Robot Car Scripts Organization

## Main Scripts Folder (`/scripts/`)
**Essential, actively used nodes:**

### Core Movement & Control:
- `master_state_machine_node.py` - Autonomous sequence coordinator (uses calibrated wheel velocities directly)

### Hardware Control:
- `beacon_motor_node.py` - Beacon drop motor control (Motor ID 6)
- `claw_motor_node.py` - Container claw motor control (Motor ID 5)

### Sensors:
- `limit_switch_node.py` - Limit switch sensor node
- `photoresistor_node.py` - Light sensor node (publishes `/start_signal`)

### Calibration & Testing (Active):
- `linear_calibration.py` - Movement accuracy calibration tool (meters) - **Currently in use**

### Support:
- `power_monitor.py` - System monitoring
- `start_robot.sh` - Main system startup script

---

## Calibration Folder (`calibration/`)
**Movement testing and calibration tools:**

- `ik_velocity2_node.py` - Manual movement testing tool
- `ik_velocity_inches_node.py` - Inch-based movement control (alternative)
- `linear_calibration_cmd_vel.py` - Calibration using /cmd_vel with inches
- `linear_calibration_inches.py` - Calibration using /wheel_velocities with inches

---

## Utils Folder (`utils/`)
**Utility scripts and references:**

- `inch_conversion_guide.py` - Conversion reference guide
- `unit_converter.py` - Unit conversion utilities

---

## Archive Folder (`archive/`)
**Unused/old files:**

- `start_simple.sh` - Basic startup script (not used)
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
- `/wheel_velocities` topic → **Direct to OpenCR** (self-contained movement)
- `/beacon_drop` topic → `beacon_motor_node.py`
- `/claw_control` topic → `claw_motor_node.py`

**Current Working System:**
1. `start_robot.sh` - launches basic communication
2. `photoresistor_node.py` - detects start LED
3. `limit_switch_node.py` - detects wall contact
4. `beacon_motor_node.py` - controls beacon dropping (Motor ID 6)
5. `claw_motor_node.py` - controls container claw (Motor ID 5)
6. `master_state_machine_node.py` - coordinates autonomous sequence with direct wheel control
