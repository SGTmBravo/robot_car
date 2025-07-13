#!/usr/bin/env python3
"""
Linear Movement Calibration Tool - INCHES + CMD_VEL VERSION
Tests different distances and directions to calibrate robot movement accuracy
Uses /cmd_vel (current system) with inch measurements for easy tape measure verification
"""
import rospy
import math
from geometry_msgs.msg import Twist

# Conversion: 1 inch = 0.0254 meters
INCHES_TO_METERS = 0.0254

def test_linear_movement_inches(cmd_vel_pub, x_inches, y_inches, label=""):
    """Test a single linear movement using inch inputs with /cmd_vel"""
    
    # Convert inches to meters for internal calculations
    x_meters = x_inches * INCHES_TO_METERS
    y_meters = y_inches * INCHES_TO_METERS
    
    linear_dist_meters = math.hypot(x_meters, y_meters)
    linear_dist_inches = math.hypot(x_inches, y_inches)
    
    if linear_dist_meters == 0:
        print(f"❌ No movement for {label}")
        return
    
    # Calculate movement timing (same as master state machine)
    speed = 0.05  # m/s - conservative speed
    movement_time = linear_dist_meters / speed
    
    print(f"\n🎯 TEST: {label}")
    print(f"   Target: x={x_inches:.1f}\", y={y_inches:.1f}\" (distance={linear_dist_inches:.1f}\")")
    print(f"   Internal: x={x_meters:.3f}m, y={y_meters:.3f}m (distance={linear_dist_meters:.3f}m)")
    print(f"   Speed: {speed:.3f} m/s, Duration: {movement_time:.2f}s")
    
    # Create velocity command
    cmd = Twist()
    cmd.linear.x = (x_meters / linear_dist_meters) * speed if linear_dist_meters > 0 else 0
    cmd.linear.y = (y_meters / linear_dist_meters) * speed if linear_dist_meters > 0 else 0
    cmd.angular.z = 0
    
    print(f"   Velocities: vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f}")
    
    # Execute movement
    print(f"   🚀 Moving for {movement_time:.2f} seconds...")
    cmd_vel_pub.publish(cmd)
    
    rospy.sleep(movement_time)
    
    # Stop
    stop_cmd = Twist()  # All zeros
    cmd_vel_pub.publish(stop_cmd)
    rospy.sleep(0.2)
    
    print(f"   ✅ Completed {label}")
    return {
        'x_inches': x_inches, 'y_inches': y_inches, 
        'x_meters': x_meters, 'y_meters': y_meters,
        'distance_inches': linear_dist_inches, 'distance_meters': linear_dist_meters,
        'duration': movement_time, 'speed': speed
    }

def main():
    rospy.init_node('linear_calibration_cmd_vel')
    
    # Publisher for cmd_vel (current system)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    print("🔧 LINEAR MOVEMENT CALIBRATION TOOL - INCHES VERSION")
    print("=" * 60)
    print("This tool will test various linear movements to calibrate accuracy.")
    print("After each test, measure the ACTUAL distance traveled with your tape measure.")
    print("All measurements in INCHES for easy verification.")
    print()
    
    # Test cases - same as original but converted to inches (CORRECTED)
    test_cases = [
        # 1 meter tests in cardinal directions (39.37 inches)
        (39.37, 0.0, "39.37\" Forward (1m X+)"),
        (0.0, 39.37, "39.37\" Left (1m Y+)"),
        (-39.37, 0.0, "39.37\" Backward (1m X-)"),
        (0.0, -39.37, "39.37\" Right (1m Y-)"),
        
        # Diagonal tests (0.707m = 27.84")
        (27.84, 27.84, "27.84\" Diagonal (1m 45° forward-left)"),
        (27.84, -27.84, "27.84\" Diagonal (1m 45° forward-right)"),
        
        # Different distances
        (19.69, 0.0, "19.69\" Forward (0.5m X+)"),
        (78.74, 0.0, "78.74\" Forward (2m X+)"),
    ]
    
    results = []
    
    while not rospy.is_shutdown():
        print("\n📋 CALIBRATION TEST MENU:")
        print("0. Run all predefined tests")
        for i, (x, y, label) in enumerate(test_cases, 1):
            print(f"{i}. {label}")
        print("88. Custom test (enter your own x, y in inches)")
        print("99. Quit")
        
        try:
            choice = input("\nSelect test (0-99): ").strip()
            
            if choice == '99':
                break
            elif choice == '0':
                # Run all tests
                print("\n🚀 RUNNING ALL CALIBRATION TESTS")
                print("📏 Please have your inch tape measure ready!")
                input("Press Enter when ready to start...")
                
                for i, (x, y, label) in enumerate(test_cases, 1):
                    print(f"\n{'='*20} TEST {i}/{len(test_cases)} {'='*20}")
                    result = test_linear_movement_inches(cmd_vel_pub, x, y, label)
                    if result:
                        actual = input(f"📏 What was the ACTUAL distance traveled? (inches): ")
                        try:
                            actual_dist = float(actual)
                            error = actual_dist - result['distance_inches']
                            error_pct = (error / result['distance_inches']) * 100 if result['distance_inches'] > 0 else 0
                            result['actual_distance'] = actual_dist
                            result['error'] = error
                            result['error_percent'] = error_pct
                            results.append(result)
                            print(f"   📊 Error: {error:+.1f}\" ({error_pct:+.1f}%)")
                        except ValueError:
                            print("   ⚠️ Invalid measurement, skipping error calculation")
                    
                    if i < len(test_cases):
                        input("Press Enter for next test...")
                
                # Summary
                print(f"\n{'='*20} CALIBRATION SUMMARY {'='*20}")
                if results:
                    errors = [r['error'] for r in results if 'error' in r]
                    if errors:
                        avg_error = sum(errors) / len(errors)
                        avg_error_pct = sum(r['error_percent'] for r in results if 'error_percent' in r) / len(errors)
                        print(f"Average error: {avg_error:+.1f}\" ({avg_error_pct:+.1f}%)")
                        
                        if abs(avg_error_pct) > 5:  # 5% threshold
                            print(f"🔧 RECOMMENDATION: Movement accuracy needs adjustment")
                            if avg_error_pct > 0:
                                print(f"   Robot overshoots by {avg_error_pct:.1f}% - consider reducing speed or timing")
                            else:
                                print(f"   Robot undershoots by {abs(avg_error_pct):.1f}% - consider increasing speed or timing")
                        else:
                            print("✅ Movement accuracy looks good!")
                        
                        # Show individual results
                        print("\nDetailed results:")
                        for r in results:
                            if 'error' in r:
                                print(f"  {r['distance_inches']:.1f}\" → {r['actual_distance']:.1f}\" ({r['error']:+.1f}\", {r['error_percent']:+.1f}%)")
                        
            elif choice == '88':
                # Custom test
                try:
                    x = float(input("Enter x distance (inches): "))
                    y = float(input("Enter y distance (inches): "))
                    label = input("Enter test label (optional): ") or f"Custom ({x:.1f}\", {y:.1f}\")"
                    result = test_linear_movement_inches(cmd_vel_pub, x, y, label)
                    if result:
                        actual = input(f"📏 What was the ACTUAL distance traveled? (inches): ")
                        try:
                            actual_dist = float(actual)
                            error = actual_dist - result['distance_inches']
                            error_pct = (error / result['distance_inches']) * 100 if result['distance_inches'] > 0 else 0
                            print(f"� Error: {error:+.1f}\" ({error_pct:+.1f}%)")
                        except ValueError:
                            print("⚠️ Invalid measurement")
                except ValueError:
                    print("❌ Invalid input")
                    
            else:
                # Individual test
                try:
                    test_idx = int(choice) - 1
                    if 0 <= test_idx < len(test_cases):
                        x, y, label = test_cases[test_idx]
                        result = test_linear_movement_inches(cmd_vel_pub, x, y, label)
                        if result:
                            actual = input(f"📏 What was the ACTUAL distance traveled? (inches): ")
                            try:
                                actual_dist = float(actual)
                                error = actual_dist - result['distance_inches']
                                error_pct = (error / result['distance_inches']) * 100 if result['distance_inches'] > 0 else 0
                                print(f"📊 Error: {error:+.1f}\" ({error_pct:+.1f}%)")
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
    print("📏 Use these results to adjust your master state machine movement distances")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n⏹️ Calibration interrupted")
