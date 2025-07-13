#!/usr/bin/env python3
"""
Quick Reference Guide - Inches to Meters Conversion
For easy tape measure use with robot commands
"""

print("📏 INCHES TO METERS CONVERSION CHEAT SHEET")
print("=" * 50)
print("For robot movement commands:")
print()

# Common measurements
conversions = [
    (1, 0.0254),
    (2, 0.0508),
    (3, 0.0762),
    (4, 0.1016),
    (5, 0.1270),
    (6, 0.1524),
    (8, 0.2032),
    (9, 0.2286),
    (10, 0.2540),
    (12, 0.3048),
    (15, 0.3810),
    (18, 0.4572),
    (20, 0.5080),
    (24, 0.6096),
]

for inches, meters in conversions:
    print(f"{inches:2d}\" = {meters:.4f}m")

print()
print("🔄 Quick conversion formula:")
print("   Inches × 0.0254 = Meters")
print("   Meters × 39.37 = Inches")
print()
print("🤖 Use these with move_robot_inches() function:")
print("   move_robot_inches(6.0, 0.0, 0)  # Move 6 inches forward")
print("   move_robot_inches(0.0, 12.0, 0) # Move 12 inches left")
print()
print("📐 Tools available:")
print("   • linear_calibration_inches.py - Calibrate using inches")
print("   • ik_velocity_inches_node.py - Manual control with inches")
print("   • unit_converter.py - Conversion utilities")
