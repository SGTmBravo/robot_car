#!/usr/bin/env python3
"""
Unit Conversion Utilities
Converts between inches and meters for robot movement commands
"""

# Conversion constants
INCHES_TO_METERS = 0.0254  # 1 inch = 0.0254 meters
METERS_TO_INCHES = 39.3701  # 1 meter = 39.3701 inches

def inches_to_meters(inches):
    """Convert inches to meters"""
    return inches * INCHES_TO_METERS

def meters_to_inches(meters):
    """Convert meters to inches"""
    return meters * METERS_TO_INCHES

def convert_position(x_inches, y_inches):
    """Convert x,y position from inches to meters"""
    return (inches_to_meters(x_inches), inches_to_meters(y_inches))

def format_distance_dual(meters):
    """Format distance showing both inches and meters"""
    inches = meters_to_inches(meters)
    return f"{inches:.2f}\" ({meters:.3f}m)"

def format_position_dual(x_meters, y_meters):
    """Format x,y position showing both inches and meters"""
    x_inches = meters_to_inches(x_meters)
    y_inches = meters_to_inches(y_meters)
    return f"x={x_inches:.2f}\", y={y_inches:.2f}\" (x={x_meters:.3f}m, y={y_meters:.3f}m)"

# Common distance presets in inches (converted to meters)
DISTANCES_INCHES = {
    "1_inch": inches_to_meters(1.0),
    "2_inch": inches_to_meters(2.0),
    "3_inch": inches_to_meters(3.0),
    "4_inch": inches_to_meters(4.0),
    "6_inch": inches_to_meters(6.0),
    "12_inch": inches_to_meters(12.0),  # 1 foot
    "18_inch": inches_to_meters(18.0),
    "24_inch": inches_to_meters(24.0),  # 2 feet
}

if __name__ == "__main__":
    # Test the conversions
    print("🔄 Unit Conversion Test:")
    print(f"1 inch = {inches_to_meters(1):.4f} meters")
    print(f"1 meter = {meters_to_inches(1):.2f} inches")
    print(f"12 inches = {inches_to_meters(12):.4f} meters")
    print(f"6 inches = {inches_to_meters(6):.4f} meters")
    
    print("\n📏 Common distances:")
    for name, meters in DISTANCES_INCHES.items():
        inches = meters_to_inches(meters)
        print(f"{name}: {inches:.1f}\" = {meters:.4f}m")
