#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84;
const float PROTOCOL_VERSION = 2.0;

// Battery monitoring setup
#define BATTERY_PIN A0  // Analog pin to read battery voltage
const float VOLTAGE_DIVIDER_RATIO = 3.0;  // Adjust based on your voltage divider (R1+R2)/R2
const float ADC_REFERENCE = 3.3;  // OpenCR ADC reference voltage
const int ADC_RESOLUTION = 1024;  // 10-bit ADC

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
ros::NodeHandle nh;

// Motor IDs
const uint8_t MOTOR_IDS[4] = {1, 2, 3, 4};

// Movement control
unsigned long movement_start = 0;
unsigned long movement_duration = 0;
bool is_moving = false;

// Battery monitoring
unsigned long last_battery_check = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 5000;  // Check every 5 seconds
std_msgs::Float32 battery_msg;
ros::Publisher battery_pub("/opencr_battery_voltage", &battery_msg);

float readBatteryVoltage() {
  int adc_reading = analogRead(BATTERY_PIN);
  float voltage = (adc_reading * ADC_REFERENCE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

void publishBatteryStatus() {
  float battery_voltage = readBatteryVoltage();
  battery_msg.data = battery_voltage;
  battery_pub.publish(&battery_msg);
  
  DEBUG_SERIAL.print("Battery voltage: ");
  DEBUG_SERIAL.print(battery_voltage);
  DEBUG_SERIAL.println("V");
  
  // Battery status warnings
  if (battery_voltage < 6.0) {  // Adjust threshold based on your battery
    DEBUG_SERIAL.println("⚠️ LOW BATTERY WARNING!");
  } else if (battery_voltage < 5.5) {
    DEBUG_SERIAL.println("❌ CRITICAL BATTERY - STOP OPERATION!");
  }
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    dxl.setGoalVelocity(MOTOR_IDS[i], 0);
  }
  is_moving = false;
  DEBUG_SERIAL.println("Motors stopped!");
}

void wheelVelocitiesCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length != 5) return;
  
  // Check battery before movement
  float battery_voltage = readBatteryVoltage();
  if (battery_voltage < 5.5) {  // Emergency stop threshold
    DEBUG_SERIAL.println("❌ BATTERY TOO LOW - MOVEMENT BLOCKED!");
    stopAllMotors();
    return;
  }
  
  // SMOOTHER conversion factor for less choppy movement
  const float CONVERSION = 25.0;  // Reduced from 50.0 for smoother motion
  
  DEBUG_SERIAL.print("Setting smooth velocities: ");
  for (int i = 0; i < 4; i++) {
    int32_t vel = (int32_t)(msg.data[i] * CONVERSION);
    dxl.setGoalVelocity(MOTOR_IDS[i], vel);
    DEBUG_SERIAL.print(vel);
    DEBUG_SERIAL.print(" ");
  }
  DEBUG_SERIAL.println();
  
  // Set movement timer
  movement_start = millis();
  movement_duration = (unsigned long)(msg.data[4] * 1000);
  is_moving = true;
  
  DEBUG_SERIAL.print("Moving for ");
  DEBUG_SERIAL.print(movement_duration);
  DEBUG_SERIAL.println(" ms");
}

ros::Subscriber<std_msgs::Float32MultiArray> wheel_sub("/wheel_velocities", wheelVelocitiesCallback);

void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Starting SMOOTH motor control with BATTERY MONITORING...");
  
  // Initialize battery monitoring pin
  pinMode(BATTERY_PIN, INPUT);
  
  // Initialize Dynamixel
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);
  dxl.begin(57600);
  
  // Setup motors with smooth acceleration
  for (int i = 0; i < 4; i++) {
    uint8_t id = MOTOR_IDS[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_VELOCITY);
      
      // Set smoother acceleration/deceleration profiles
      dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 50);  // Slower acceleration
      dxl.writeControlTableItem(PROFILE_VELOCITY, id, 100);     // Limit max velocity
      
      dxl.torqueOn(id);
      dxl.setGoalVelocity(id, 0); // Start stopped
      DEBUG_SERIAL.print("Motor ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" OK (smooth profile)");
    } else {
      DEBUG_SERIAL.print("Motor ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" FAILED");
    }
  }
  
  // Initialize ROS
  nh.initNode();
  nh.subscribe(wheel_sub);
  nh.advertise(battery_pub);
  
  // Initial battery reading
  publishBatteryStatus();
  
  DEBUG_SERIAL.println("Ready for SMOOTH commands with BATTERY MONITORING!");
}

void loop() {
  // Check if movement should stop
  if (is_moving && (millis() - movement_start >= movement_duration)) {
    stopAllMotors();
  }
  
  // Periodic battery monitoring
  if (millis() - last_battery_check >= BATTERY_CHECK_INTERVAL) {
    publishBatteryStatus();
    last_battery_check = millis();
  }
  
  nh.spinOnce();
  delay(10);
}
