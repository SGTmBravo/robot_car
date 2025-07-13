#!/usr/bin/env python3
"""
Linear Movement Calibration Tool
Tests different distances and directions to calibrate robot movement accuracy
"""
import rospy
import math
from std_msgs.msg import Float32MultiArray

# Robot parameters (same as ik_velocity2_node.py)
WHEEL_RADIUS = 0.02825
L = 0.075
MOTOR_SPEED = 0.05       # m/s - Actual motor speed (physical wheel speed)
DISTANCE_SCALE = 0.607   # Calibration factor: measured 1.6477m for 1.0m target, so scale = 1.0/1.6477

def test_linear_movement(wheel_pub, x, y, label=""):
    """Test a single linear movement and return the command details"""
    
    linear_dist = math.hypot(x, y)
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
    print(f"   Target: x={x:.2f}m, y={y:.2f}m (distance={linear_dist:.2f}m)")
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
    
    print(f"   ✅ Movement complete! Please measure actual distance traveled.")
    return {
        'target_distance': linear_dist,
        'target_x': x,
        'target_y': y,
        'duration': linear_time,
        'motors': [motor1_fr, motor2_fl, motor3_rl, motor4_rr]
    }

def main():
    rospy.init_node('linear_calibration_node')
    wheel_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    
    print("🔧 LINEAR MOVEMENT CALIBRATION TOOL")
    print("=" * 50)
    print("This tool will test various linear movements to calibrate accuracy.")
    print("After each test, measure the ACTUAL distance traveled and note any deviation.")
    print()
    
    # Test cases - start with 1 meter in cardinal directions
    test_cases = [
        # 1 meter tests in cardinal directions
        (1.0, 0.0, "1m Forward (X+)"),
        (0.0, 1.0, "1m Left (Y+)"),
        (-1.0, 0.0, "1m Backward (X-)"),
        (0.0, -1.0, "1m Right (Y-)"),
        
        # Diagonal tests
        (0.707, 0.707, "1m Diagonal (45° forward-left)"),
        (0.707, -0.707, "1m Diagonal (45° forward-right)"),
        
        # Different distances
        (0.5, 0.0, "0.5m Forward (X+)"),
        (2.0, 0.0, "2m Forward (X+)"),
    ]
    
    results = []
    
    while not rospy.is_shutdown():
        print("\n📋 CALIBRATION TEST MENU:")
        print("0. Run all predefined tests")
        for i, (x, y, label) in enumerate(test_cases, 1):
            print(f"{i}. {label}")
        print("88. Custom test (enter your own x, y)")
        print("99. Quit")
        
        try:
            choice = input("\nSelect test (0-99): ").strip()
            
            if choice == '99':
                break
            elif choice == '0':
                # Run all tests
                print("\n🚀 RUNNING ALL CALIBRATION TESTS")
                print("Please have a tape measure ready!")
                input("Press Enter when ready to start...")
                
                for i, (x, y, label) in enumerate(test_cases, 1):
                    print(f"\n{'='*20} TEST {i}/{len(test_cases)} {'='*20}")
                    result = test_linear_movement(wheel_pub, x, y, label)
                    if result:
                        actual = input(f"What was the ACTUAL distance traveled? (m): ")
                        try:
                            actual_dist = float(actual)
                            error = actual_dist - result['target_distance']
                            error_pct = (error / result['target_distance']) * 100
                            result['actual_distance'] = actual_dist
                            result['error'] = error
                            result['error_percent'] = error_pct
                            results.append(result)
                            print(f"   📊 Error: {error:+.3f}m ({error_pct:+.1f}%)")
                        except ValueError:
                            print("   ⚠️ Invalid measurement, skipping error calculation")
                    
                    if i < len(test_cases):
                        input("Press Enter for next test...")
                
                # Summary
                print(f"\n{'='*20} CALIBRATION SUMMARY {'='*20}")
                if results:
                    total_error = sum(r['error'] for r in results if 'error' in r)
                    avg_error = total_error / len([r for r in results if 'error' in r])
                    print(f"Average error: {avg_error:+.3f}m")
                    
                    if abs(avg_error) > 0.05:  # 5cm threshold
                        new_scale = DISTANCE_SCALE * (1 + avg_error/1.0)  # Adjust scale factor
                        print(f"🔧 RECOMMENDATION: Adjust DISTANCE_SCALE from {DISTANCE_SCALE:.3f} to {new_scale:.3f}")
                    else:
                        print("✅ Current calibration looks good!")
                        
            elif choice == '88':
                # Custom test
                try:
                    x = float(input("Enter x distance (m): "))
                    y = float(input("Enter y distance (m): "))
                    label = input("Enter test label (optional): ") or f"Custom ({x:.2f}, {y:.2f})"
                    test_linear_movement(wheel_pub, x, y, label)
                except ValueError:
                    print("❌ Invalid input")
                    
            else:
                # Individual test
                try:
                    test_idx = int(choice) - 1
                    if 0 <= test_idx < len(test_cases):
                        x, y, label = test_cases[test_idx]
                        result = test_linear_movement(wheel_pub, x, y, label)
                        actual = input(f"What was the ACTUAL distance traveled? (m): ")
                        try:
                            actual_dist = float(actual)
                            error = actual_dist - result['target_distance']
                            error_pct = (error / result['target_distance']) * 100
                            print(f"📊 Error: {error:+.3f}m ({error_pct:+.1f}%)")
                        except ValueError:
                            print("⚠️ Invalid measurement")
                    else:
                        print("❌ Invalid choice")
                except ValueError:
                    print("❌ Invalid choice")
                    
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print("\n🏁 Calibration session complete!")

if __name__ == "__main__":
    main()
