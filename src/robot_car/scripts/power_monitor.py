#!/usr/bin/env python3
"""
Raspberry Pi Power and Health Monitor
Checks voltage, temperature, throttling, and power issues
"""

import subprocess
import time
import os
import re

def get_voltage():
    """Get voltage readings - NOTE: Pi does not expose main 5V supply voltage via software"""
    voltages = {}
    
    try:
        # Read internal voltage rails (these are NOT the main 5V supply)
        result = subprocess.run(['vcgencmd', 'measure_volts'], capture_output=True, text=True)
        if result.returncode == 0:
            voltage_str = result.stdout.strip()
            if '=' in voltage_str:
                voltage = float(re.findall(r'[\d.]+', voltage_str)[0])
                voltages['core_voltage'] = voltage
        
        # Try to get other internal rails for reference
        for param in ['core', 'sdram_c', 'sdram_i', 'sdram_p']:
            try:
                result = subprocess.run(['vcgencmd', 'measure_volts', param], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    voltage_str = result.stdout.strip()
                    if '=' in voltage_str:
                        voltage = float(re.findall(r'[\d.]+', voltage_str)[0])
                        voltages[f'{param}_voltage'] = voltage
            except:
                continue
                
        return voltages
    except Exception as e:
        return {'error': str(e)}

def get_temperature():
    """Get CPU temperature"""
    try:
        result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
        if result.returncode == 0:
            temp_str = result.stdout.strip()
            temp = float(re.findall(r'[\d.]+', temp_str)[0])
            return temp
        return None
    except:
        return None

def get_throttle_status():
    """Check for throttling/power issues"""
    try:
        result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True)
        if result.returncode == 0:
            throttle_hex = result.stdout.strip().split('=')[1]
            throttle_int = int(throttle_hex, 16)
            
            status = {
                'under_voltage_now': bool(throttle_int & 0x1),
                'arm_frequency_capped_now': bool(throttle_int & 0x2),
                'currently_throttled': bool(throttle_int & 0x4),
                'soft_temp_limit_active': bool(throttle_int & 0x8),
                'under_voltage_occurred': bool(throttle_int & 0x10000),
                'arm_frequency_capped_occurred': bool(throttle_int & 0x20000),
                'throttling_occurred': bool(throttle_int & 0x40000),
                'soft_temp_limit_occurred': bool(throttle_int & 0x80000)
            }
            return status, throttle_hex
        return None, None
    except:
        return None, None

def get_memory_usage():
    """Get memory usage info"""
    try:
        result = subprocess.run(['free', '-h'], capture_output=True, text=True)
        return result.stdout
    except:
        return "Memory info unavailable"

def get_cpu_usage():
    """Get CPU usage"""
    try:
        # Simple CPU usage check
        result = subprocess.run(['top', '-bn1'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        for line in lines:
            if 'Cpu(s):' in line:
                return line.strip()
        return "CPU info unavailable"
    except:
        return "CPU info unavailable"

def check_power_health():
    """Main power health check"""
    print("🔋 RASPBERRY PI POWER & HEALTH MONITOR")
    print("=" * 50)
    
    # Voltage check - internal rails only
    voltages = get_voltage()
    if voltages and not voltages.get('error'):
        print("⚡ Internal Voltage Rails (reference only):")
        
        for label, voltage in voltages.items():
            print(f"  {label}: {voltage:.3f}V")
        
        print(f"\n📊 Main 5V Supply Voltage:")
        print("   ❓ NOT available via software on Raspberry Pi")
        print("   📏 Use multimeter on GPIO pins 2(+5V) & 6(GND) for accurate reading")
        
    elif voltages and voltages.get('error'):
        print(f"⚡ Voltage: Error reading internal rails - {voltages['error']}")
        print("   📏 Use multimeter on GPIO pins 2(+5V) & 6(GND) for main 5V reading")
    else:
        print("⚡ Voltage: Unable to read internal voltage rails")
        print("   📏 Use multimeter on GPIO pins 2(+5V) & 6(GND) for main 5V reading")
    
    # Temperature check
    temp = get_temperature()
    if temp:
        print(f"🌡️  Temperature: {temp:.1f}°C", end="")
        if temp < 60:
            print(" ✅ GOOD")
        elif temp < 70:
            print(" ⚠️  WARM")
        elif temp < 80:
            print(" 🔥 HOT")
        else:
            print(" ❌ CRITICAL (throttling likely)")
    else:
        print("🌡️  Temperature: Unable to read")
    
    # Throttling status
    throttle_status, throttle_hex = get_throttle_status()
    if throttle_status:
        print(f"🚦 Throttle Status: {throttle_hex}")
        
        current_issues = []
        past_issues = []
        
        if throttle_status['under_voltage_now']:
            current_issues.append("Under-voltage NOW")
        if throttle_status['currently_throttled']:
            current_issues.append("Throttled NOW")
        if throttle_status['arm_frequency_capped_now']:
            current_issues.append("Frequency capped NOW")
        if throttle_status['soft_temp_limit_active']:
            current_issues.append("Temp limit active NOW")
            
        if throttle_status['under_voltage_occurred']:
            past_issues.append("Under-voltage occurred")
        if throttle_status['throttling_occurred']:
            past_issues.append("Throttling occurred")
        if throttle_status['arm_frequency_capped_occurred']:
            past_issues.append("Frequency capping occurred")
        if throttle_status['soft_temp_limit_occurred']:
            past_issues.append("Temp limit occurred")
        
        if current_issues:
            print(f"❌ CURRENT ISSUES: {', '.join(current_issues)}")
        else:
            print("✅ No current power/thermal issues")
            
        if past_issues:
            print(f"⚠️  PAST ISSUES: {', '.join(past_issues)}")
    else:
        print("🚦 Throttle Status: Unable to read")
    
    print("\n💾 SYSTEM RESOURCES:")
    print("-" * 30)
    
    # Memory usage
    memory = get_memory_usage()
    print("Memory Usage:")
    for line in memory.split('\n')[:3]:  # First 3 lines
        if line.strip():
            print(f"  {line}")
    
    # CPU usage
    cpu = get_cpu_usage()
    print(f"CPU: {cpu}")
    
    print("\n📋 POWER ASSESSMENT:")
    print("-" * 30)
    
    # Dynamic assessment based on throttling status
    if throttle_status:
        if not any([throttle_status['under_voltage_now'], 
                   throttle_status['currently_throttled'],
                   throttle_status['arm_frequency_capped_now']]):
            print("✅ POWER STATUS: GOOD!")
            print("  - No current power issues detected")
            print("  - System running without throttling")
        else:
            print("⚠️  POWER STATUS: ISSUES DETECTED!")
            
        if not any([throttle_status['under_voltage_occurred'],
                   throttle_status['throttling_occurred']]):
            print("  - No past power issues")
        else:
            print("  - Past power issues detected (see throttle status above)")
    
    print("\n🔮 MONITORING RECOMMENDATIONS:")
    print("  - Use multimeter on GPIO pins 2(+5V) & 6(GND) for accurate 5V measurement")
    print("  - Monitor power when adding servos/sensors") 
    print("  - Keep USB cables short and high quality")
    print("  - Target: 5.0V ±0.25V (4.75V - 5.25V) for stable operation")
    
    # Only show warnings if there are real issues
    if temp and temp > 70:
        print("\n⚠️  THERMAL RECOMMENDATIONS:")
        print("❗ Consider adding cooling (heatsinks, fan)")
        print("❗ Check case ventilation")

def continuous_monitor(interval=5):
    """Continuous monitoring mode"""
    print(f"🔄 Starting continuous monitoring (every {interval} seconds)")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            os.system('clear')  # Clear screen
            check_power_health()
            print(f"\n⏱️  Next update in {interval} seconds...")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n🛑 Monitoring stopped")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--monitor":
        continuous_monitor()
    else:
        check_power_health()
        print("\n💡 Run with '--monitor' for continuous monitoring")
