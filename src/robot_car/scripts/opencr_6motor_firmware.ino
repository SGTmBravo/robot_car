#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84;
const float PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
ros::NodeHandle nh;

// All motor IDs - wheels (1-4) + beacon (5) + claw (6)
const uint8_t WHEEL_MOTOR_IDS[4] = {1, 2, 3, 4};
const uint8_t BEACON_MOTOR_ID = 5;
const uint8_t CLAW_MOTOR_ID = 6;
const uint8_t ALL_MOTOR_IDS[6] = {1, 2, 3, 4, 5, 6};

// Position control for claw and beacon
const uint32_t CLAW_OPEN = 1024;
const uint32_t CLAW_CLOSED = 2048;
const uint32_t BEACON_UP = 2048;
const uint32_t BEACON_DOWN = 1024;

// Movement control
unsigned long movement_start = 0;
unsigned long movement_duration = 0;
bool is_moving = false;

void stopWheelMotors() {
  for (int i = 0; i < 4; i++) {
    dxl.setGoalVelocity(WHEEL_MOTOR_IDS[i], 0);
  }
  is_moving = false;
  DEBUG_SERIAL.println("Wheel motors stopped!");
}

// Wheel velocity callback (unchanged from original)
void wheelVelocitiesCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length != 5) return;
  
  // Increase conversion factor for faster movement
  const float CONVERSION = 50.0;
  
  DEBUG_SERIAL.print("Setting wheel velocities: ");
  for (int i = 0; i < 4; i++) {
    int32_t vel = (int32_t)(msg.data[i] * CONVERSION);
    dxl.setGoalVelocity(WHEEL_MOTOR_IDS[i], vel);
    DEBUG_SERIAL.print(vel);
    DEBUG_SERIAL.print(" ");
  }
  DEBUG_SERIAL.println();
  
  // Set movement timer
  movement_start = millis();
  movement_duration = (unsigned long)(msg.data[4] * 1000); // Convert to ms
  is_moving = true;
  
  DEBUG_SERIAL.print("Moving for ");
  DEBUG_SERIAL.print(movement_duration);
  DEBUG_SERIAL.println(" ms");
}

// Claw control callback
void clawControlCallback(const std_msgs::Bool& msg) {
  uint32_t target_pos = msg.data ? CLAW_CLOSED : CLAW_OPEN;
  
  DEBUG_SERIAL.print("Setting claw to position: ");
  DEBUG_SERIAL.println(target_pos);
  
  dxl.setGoalPosition(CLAW_MOTOR_ID, target_pos);
}

// Beacon control callback  
void beaconControlCallback(const std_msgs::Bool& msg) {
  uint32_t target_pos = msg.data ? BEACON_DOWN : BEACON_UP;
  
  DEBUG_SERIAL.print("Setting beacon to position: ");
  DEBUG_SERIAL.println(target_pos);
  
  dxl.setGoalPosition(BEACON_MOTOR_ID, target_pos);
}

// ROS subscribers
ros::Subscriber<std_msgs::Float32MultiArray> wheel_sub("/wheel_velocities", wheelVelocitiesCallback);
ros::Subscriber<std_msgs::Bool> claw_sub("/claw_control", clawControlCallback);
ros::Subscriber<std_msgs::Bool> beacon_sub("/beacon_drop", beaconControlCallback);

void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Starting 6-motor OpenCR control...");
  
  // Initialize Dynamixel
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);
  dxl.begin(57600);
  
  // Setup wheel motors (1-4) for velocity control
  for (int i = 0; i < 4; i++) {
    uint8_t id = WHEEL_MOTOR_IDS[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id);
      dxl.setGoalVelocity(id, 0); // Start stopped
      DEBUG_SERIAL.print("Wheel Motor ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" OK");
    } else {
      DEBUG_SERIAL.print("Wheel Motor ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" FAILED");
    }
  }
  
  // Setup beacon motor (5) for position control
  if (dxl.ping(BEACON_MOTOR_ID)) {
    dxl.torqueOff(BEACON_MOTOR_ID);
    dxl.setOperatingMode(BEACON_MOTOR_ID, OP_POSITION);
    dxl.torqueOn(BEACON_MOTOR_ID);
    dxl.setGoalPosition(BEACON_MOTOR_ID, BEACON_UP); // Start up
    DEBUG_SERIAL.print("Beacon Motor ");
    DEBUG_SERIAL.print(BEACON_MOTOR_ID);
    DEBUG_SERIAL.println(" OK");
  } else {
    DEBUG_SERIAL.print("Beacon Motor ");
    DEBUG_SERIAL.print(BEACON_MOTOR_ID);
    DEBUG_SERIAL.println(" FAILED");
  }
  
  // Setup claw motor (6) for position control
  if (dxl.ping(CLAW_MOTOR_ID)) {
    dxl.torqueOff(CLAW_MOTOR_ID);
    dxl.setOperatingMode(CLAW_MOTOR_ID, OP_POSITION);
    dxl.torqueOn(CLAW_MOTOR_ID);
    dxl.setGoalPosition(CLAW_MOTOR_ID, CLAW_OPEN); // Start open
    DEBUG_SERIAL.print("Claw Motor ");
    DEBUG_SERIAL.print(CLAW_MOTOR_ID);
    DEBUG_SERIAL.println(" OK");
  } else {
    DEBUG_SERIAL.print("Claw Motor ");
    DEBUG_SERIAL.print(CLAW_MOTOR_ID);
    DEBUG_SERIAL.println(" FAILED");
  }
  
  // Initialize ROS
  nh.initNode();
  nh.subscribe(wheel_sub);
  nh.subscribe(claw_sub);
  nh.subscribe(beacon_sub);
  
  DEBUG_SERIAL.println("Ready for commands!");
}

void loop() {
  // Check if wheel movement should stop
  if (is_moving && (millis() - movement_start >= movement_duration)) {
    stopWheelMotors();
  }
  
  nh.spinOnce();
  delay(10);
}
