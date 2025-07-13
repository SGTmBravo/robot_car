#!/usr/bin/env python3
"""
Linear Movement Calibration Tool - INCHES VERSION
Tests different distances and directions to calibrate robot movement accuracy
All inputs and outputs in inches for easier measurement with standard tape measures
"""
import rospy
import math
from std_msgs.msg import Float32MultiArray
from unit_converter import inches_to_meters, meters_to_inches, format_distance_dual, format_position_dual

# Robot parameters (same as ik_velocity2_node.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.05       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor: measured 1.6477m for 1.0m target, so scale = 1.0/1.6477

def test_linear_movement_inches(wheel_pub, x_inches, y_inches, label=""):
    """Test a single linear movement using inch inputs"""
    
    # Convert inches to meters for internal calculations
    x = inches_to_meters(x_inches)
    y = inches_to_meters(y_inches)
    
    linear_dist = math.hypot(x, y)
    linear_dist_inches = meters_to_inches(linear_dist)
    
    if linear_dist == 0:
        print(f"❌ No movement for {label}")
        return
        
    linear_time = (linear_dist / MOTOR_SPEED) * DISTANCE_SCALE
    
    # Calculate direction and motor commands
    vx = (x / linear_dist) * MOTOR_SPEED
    vy = (y / linear_dist) * MOTOR_SPEED
    
    omega = 0  # No rotation during linear movement
    motor1_fr = (vx - vy - omega * L) / WHEEL_RADIUS  # Front Right
    motor2_fl = (vx + vy + omega * L) / WHEEL_RADIUS  # Front Left  
    motor3_rl = (vx - vy + omega * L) / WHEEL_RADIUS  # Rear Left
    motor4_rr = (vx + vy - omega * L) / WHEEL_RADIUS  # Rear Right
    
    print(f"\n🎯 TEST: {label}")
    print(f"   Target: x={x_inches:.1f}\", y={y_inches:.1f}\" (distance={linear_dist_inches:.1f}\")")
    print(f"   Internal: {format_position_dual(x, y)}, distance={format_distance_dual(linear_dist)}")
    print(f"   Motor Speed: {MOTOR_SPEED:.3f} m/s, Duration: {linear_time:.2f}s, Scale: {DISTANCE_SCALE:.3f}")
    print(f"   Motors: FR={motor1_fr:.2f}, FL={motor2_fl:.2f}, RL={motor3_rl:.2f}, RR={motor4_rr:.2f}")
    
    # Execute movement
    msg = Float32MultiArray()
    msg.data = [motor1_fr, motor2_fl, motor3_rl, motor4_rr, linear_time]
    wheel_pub.publish(msg)
    
    print(f"   ⏳ Moving for {linear_time:.2f} seconds...")
    rospy.sleep(linear_time + 0.5)
    
    # Stop commands
    for _ in range(3):
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0, 0.0, 0.1]
        wheel_pub.publish(stop_msg)
        rospy.sleep(0.1)
    
    print(f"   ✅ Completed {label}")
    return {
        'x_inches': x_inches, 'y_inches': y_inches, 'x_meters': x, 'y_meters': y,
        'distance_inches': linear_dist_inches, 'distance_meters': linear_dist,
        'duration': linear_time, 'motors': [motor1_fr, motor2_fl, motor3_rl, motor4_rr]
    }

def main():
    rospy.init_node('linear_calibration_inches')
    
    # Publisher for wheel velocities
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    
    rospy.loginfo("🔧 Linear Calibration Tool (INCHES VERSION) Ready")
    rospy.loginfo("📏 All measurements in inches for easier tape measure use")
    rospy.sleep(1)
    
    print("\n" + "="*60)
    print("🤖 ROBOT LINEAR MOVEMENT CALIBRATION - INCHES VERSION")
    print("="*60)
    print("📏 Use your inch tape measure to verify actual distances")
    print("⚠️  Place robot in open area before starting")
    print("🎯 Test various distances to check accuracy")
    
    # Test sequence in inches (easier to measure)
    test_commands = [
        (3.0, 0.0, "3\" Forward"),          # 3 inches forward
        (-3.0, 0.0, "3\" Backward"),        # 3 inches back
        (0.0, 3.0, "3\" Left"),             # 3 inches left
        (0.0, -3.0, "3\" Right"),           # 3 inches right
        (6.0, 0.0, "6\" Forward"),          # 6 inches forward
        (-6.0, 0.0, "6\" Backward"),        # 6 inches back
        (0.0, 6.0, "6\" Left"),             # 6 inches left
        (0.0, -6.0, "6\" Right"),           # 6 inches right
        (4.0, 4.0, "4\" Diagonal NE"),      # 4" x 4" diagonal
        (-4.0, -4.0, "4\" Diagonal SW"),    # opposite diagonal
    ]
    
    input("Press Enter to start calibration sequence...")
    
    results = []
    for x_inches, y_inches, label in test_commands:
        input(f"\n📐 Next: {label}. Mark robot position and press Enter...")
        result = test_linear_movement_inches(wheel_pub, x_inches, y_inches, label)
        if result:
            results.append(result)
        
        # Ask for measurement
        try:
            actual = input(f"📏 Measure actual distance moved (inches): ")
            if actual:
                actual_inches = float(actual)
                expected_inches = result['distance_inches']
                error_inches = actual_inches - expected_inches
                error_percent = (error_inches / expected_inches) * 100
                print(f"   📊 Expected: {expected_inches:.1f}\", Actual: {actual_inches:.1f}\", Error: {error_inches:+.1f}\" ({error_percent:+.1f}%)")
                result['actual_inches'] = actual_inches
                result['error_inches'] = error_inches
                result['error_percent'] = error_percent
        except ValueError:
            print("   ⚠️ Invalid measurement, skipping")
    
    # Summary
    print("\n" + "="*60)
    print("📊 CALIBRATION SUMMARY")
    print("="*60)
    
    total_error = 0
    valid_tests = 0
    
    for result in results:
        if 'actual_inches' in result:
            print(f"Target: {result['distance_inches']:.1f}\", Actual: {result['actual_inches']:.1f}\", Error: {result['error_inches']:+.1f}\" ({result['error_percent']:+.1f}%)")
            total_error += abs(result['error_percent'])
            valid_tests += 1
    
    if valid_tests > 0:
        avg_error = total_error / valid_tests
        print(f"\n📈 Average absolute error: {avg_error:.1f}%")
        
        if avg_error > 10:
            current_scale = DISTANCE_SCALE
            suggested_scale = current_scale * (100 / (100 + avg_error))
            print(f"💡 Consider updating DISTANCE_SCALE from {current_scale:.3f} to {suggested_scale:.3f}")
    
    print("\n🏁 Calibration complete!")
    print("📏 Remember: All future commands can use inches with the conversion tools")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n⏹️ Calibration interrupted")
