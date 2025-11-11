#!/usr/bin/env python3
"""
Comprehensive IMU diagnostic script
Tests all components of the IMU data pipeline
"""

import sys
import os
import subprocess

def print_header(title):
    """Print a formatted header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)

def print_result(test_name, passed, message=""):
    """Print test result"""
    status = "✓ PASS" if passed else "✗ FAIL"
    color = "\033[92m" if passed else "\033[91m"
    reset = "\033[0m"
    print(f"{color}{status}{reset} - {test_name}")
    if message:
        print(f"      {message}")

def check_i2c_permissions():
    """Check if user has I2C permissions"""
    print_header("1. I2C Permissions Check")
    
    try:
        # Check if user is in i2c group
        result = subprocess.run(['groups'], capture_output=True, text=True)
        groups = result.stdout
        
        has_i2c = 'i2c' in groups
        print_result("User in i2c group", has_i2c, 
                    f"Groups: {groups.strip()}" if has_i2c else "User not in i2c group - add with: sudo usermod -a -G i2c $USER")
        
        # Check if /dev/i2c-1 exists and is accessible
        i2c_dev = "/dev/i2c-1"
        i2c_exists = os.path.exists(i2c_dev)
        print_result("I2C device exists", i2c_exists, 
                    f"{i2c_dev} found" if i2c_exists else f"{i2c_dev} not found")
        
        if i2c_exists:
            i2c_readable = os.access(i2c_dev, os.R_OK | os.W_OK)
            print_result("I2C device accessible", i2c_readable,
                        "Can read/write" if i2c_readable else "No read/write permission")
        
        return has_i2c and i2c_exists
    except Exception as e:
        print_result("I2C permissions check", False, str(e))
        return False

def check_i2c_device():
    """Check if MPU6050 is detected on I2C bus"""
    print_header("2. I2C Device Detection")
    
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            output = result.stdout
            has_0x68 = '68' in output
            print_result("MPU6050 detected at 0x68", has_0x68,
                        "Device found on I2C bus" if has_0x68 else "Device NOT found - check wiring")
            
            if has_0x68:
                print("\nI2C scan output:")
                print(output)
            
            return has_0x68
        else:
            print_result("i2cdetect command", False, "Command failed")
            return False
    except FileNotFoundError:
        print_result("i2cdetect tool", False, "Install with: sudo apt-get install i2c-tools")
        return False
    except Exception as e:
        print_result("I2C detection", False, str(e))
        return False

def check_mpu6050_library():
    """Check if mpu6050 Python library is installed"""
    print_header("3. MPU6050 Library Check")
    
    try:
        import mpu6050
        print_result("mpu6050 library installed", True, f"Version: {mpu6050.__version__ if hasattr(mpu6050, '__version__') else 'unknown'}")
        return True
    except ImportError as e:
        print_result("mpu6050 library installed", False, "Install with: pip3 install mpu6050-raspberrypi")
        return False

def test_direct_sensor_access():
    """Test direct MPU6050 sensor access"""
    print_header("4. Direct Sensor Access Test")
    
    try:
        from mpu6050 import mpu6050
        sensor = mpu6050(0x68)
        
        print_result("MPU6050 initialization", True, "Sensor initialized at 0x68")
        
        # Try to read data
        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()
        temp = sensor.get_temp()
        
        print_result("Read accelerometer", True, f"X={accel['x']:.2f}, Y={accel['y']:.2f}, Z={accel['z']:.2f} m/s²")
        print_result("Read gyroscope", True, f"X={gyro['x']:.2f}, Y={gyro['y']:.2f}, Z={gyro['z']:.2f} deg/s")
        print_result("Read temperature", True, f"{temp:.1f}°C")
        
        return True
    except Exception as e:
        print_result("Direct sensor access", False, str(e))
        return False

def test_mpu6050_reader():
    """Test the MPU6050Reader wrapper class"""
    print_header("5. MPU6050Reader Class Test")
    
    try:
        # Find project root dynamically
        script_dir = os.path.dirname(os.path.abspath(__file__))
        scripts_path = os.path.join(script_dir, 'ros2_ws', 'src', 'my_robot_automation', 'scripts')
        
        if not os.path.exists(scripts_path):
            # Try to find it relative to current directory
            scripts_path = os.path.join(os.getcwd(), 'ros2_ws', 'src', 'my_robot_automation', 'scripts')
        
        if os.path.exists(scripts_path):
            sys.path.insert(0, scripts_path)
            print(f"      Using scripts path: {scripts_path}")
        else:
            print_result("Locate scripts directory", False, f"Cannot find ros2_ws/src/my_robot_automation/scripts")
            return False
        
        from mpu6050_reader import MPU6050Reader
        
        print_result("Import MPU6050Reader", True, "Class loaded successfully")
        
        imu = MPU6050Reader(address=0x68)
        
        if imu.initialized:
            print_result("MPU6050Reader initialization", True, "Reader initialized")
            
            # Test read_all()
            data = imu.read_all()
            if data:
                print_result("read_all() method", True, "Data retrieved successfully")
                print(f"      Accel: X={data['accelerometer']['x']:.2f}, Y={data['accelerometer']['y']:.2f}, Z={data['accelerometer']['z']:.2f}")
                print(f"      Gyro:  X={data['gyroscope']['x']:.2f}, Y={data['gyroscope']['y']:.2f}, Z={data['gyroscope']['z']:.2f}")
                print(f"      Temp:  {data['temperature']:.1f}°C")
            else:
                print_result("read_all() method", False, "Returned None")
            
            # Test get_orientation()
            orient = imu.get_orientation()
            if orient:
                print_result("get_orientation() method", True, "Orientation calculated")
                print(f"      Pitch={orient['pitch']:.1f}°, Roll={orient['roll']:.1f}°, Yaw={orient['yaw']:.1f}°")
                return True
            else:
                print_result("get_orientation() method", False, "Returned None")
                return False
        else:
            print_result("MPU6050Reader initialization", False, "Reader failed to initialize")
            return False
            
    except ImportError as e:
        print_result("Import MPU6050Reader", False, f"Import error: {e}")
        return False
    except Exception as e:
        print_result("MPU6050Reader test", False, str(e))
        return False

def check_web_interface_running():
    """Check if web interface is running"""
    print_header("6. Web Interface Status")
    
    try:
        import requests
        
        # Test health endpoint
        try:
            response = requests.get('http://localhost:8000/health', timeout=2)
            if response.status_code == 200:
                print_result("Web interface running", True, "Server responding on port 8000")
                return True
            else:
                print_result("Web interface running", False, f"Server returned status {response.status_code}")
                return False
        except requests.exceptions.ConnectionError:
            print_result("Web interface running", False, "No response on port 8000 - server not running")
            return False
    except ImportError:
        print_result("requests library", False, "Install with: pip3 install requests")
        return False

def test_imu_api_endpoint():
    """Test the IMU API endpoint"""
    print_header("7. IMU API Endpoint Test")
    
    try:
        import requests
        
        response = requests.get('http://localhost:8000/api/robot/imu/position', timeout=5)
        
        if response.status_code == 200:
            data = response.json()
            
            if data.get('success'):
                print_result("API endpoint responding", True, "Endpoint returned success=true")
                
                imu_data = data.get('data', {})
                
                # Check if data is simulated or real
                orientation = imu_data.get('orientation', {})
                accel = imu_data.get('linear_acceleration', {})
                
                print("\n      Returned IMU Data:")
                print(f"      Orientation: X={orientation.get('x', 0):.2f}°, Y={orientation.get('y', 0):.2f}°, Z={orientation.get('z', 0):.2f}°")
                print(f"      Accel: X={accel.get('x', 0):.2f}, Y={accel.get('y', 0):.2f}, Z={accel.get('z', 0):.2f} m/s²")
                print(f"      Temp: {imu_data.get('temperature', 0):.1f}°C")
                
                # Check if it looks like simulated data (smooth sine wave values)
                if abs(accel.get('z', 0) - 9.81) < 0.5:
                    print_result("Data source", True, "Appears to be SIMULATED data")
                    print("      ⚠ Web interface is running in simulation mode")
                    print("      ⚠ IMU may not be initialized in web interface")
                else:
                    print_result("Data source", True, "Appears to be REAL sensor data")
                
                return True
            else:
                print_result("API endpoint", False, f"Error: {data.get('error')}")
                return False
        else:
            print_result("API endpoint", False, f"HTTP {response.status_code}")
            return False
            
    except Exception as e:
        print_result("IMU API test", False, str(e))
        return False

def check_web_interface_logs():
    """Provide instructions for checking web interface logs"""
    print_header("8. Web Interface Logs")
    print("To check web interface initialization:")
    print("  1. Stop current web interface (Ctrl+C)")
    print("  2. Start it again and watch for 'MPU6050 initialized successfully'")
    print("  3. If you see 'Running in SIMULATION mode', the IMU isn't initializing")
    print("\nCommand to start web interface (from project root):")
    print("  cd ros2_ws")
    print("  python3 src/my_robot_automation/scripts/web_robot_interface.py")

def main():
    """Run all diagnostic tests"""
    print("\n" + "╔" + "=" * 68 + "╗")
    print("║" + " " * 20 + "IMU DIAGNOSTIC TOOL" + " " * 29 + "║")
    print("╚" + "=" * 68 + "╝")
    
    # Show current directory
    current_dir = os.getcwd()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"\nCurrent directory: {current_dir}")
    print(f"Script location: {script_dir}")
    
    if current_dir != script_dir:
        print("\n⚠ WARNING: You are not in the project root directory!")
        print(f"⚠ Please run this from: {script_dir}")
        print("\nTo fix:")
        print(f"  cd {script_dir}")
        print("  python3 diagnose_imu.py")
        print("\nContinuing anyway...\n")
    
    results = {}
    
    # Run all tests
    results['i2c_permissions'] = check_i2c_permissions()
    results['i2c_device'] = check_i2c_device()
    results['mpu6050_library'] = check_mpu6050_library()
    results['direct_access'] = test_direct_sensor_access()
    results['reader_class'] = test_mpu6050_reader()
    results['web_interface'] = check_web_interface_running()
    
    if results['web_interface']:
        results['api_endpoint'] = test_imu_api_endpoint()
    else:
        print_header("7. IMU API Endpoint Test")
        print("⊘ SKIPPED - Web interface not running")
        results['api_endpoint'] = False
    
    check_web_interface_logs()
    
    # Summary
    print_header("DIAGNOSTIC SUMMARY")
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    print(f"\nTests passed: {passed}/{total}")
    print("\nStatus:")
    for test, result in results.items():
        status = "✓" if result else "✗"
        print(f"  {status} {test.replace('_', ' ').title()}")
    
    # Recommendations
    print("\n" + "─" * 70)
    print("RECOMMENDATIONS:")
    print("─" * 70)
    
    if not results['i2c_permissions']:
        print("\n1. Fix I2C permissions:")
        print("   sudo usermod -a -G i2c $USER")
        print("   Then logout and login again")
    
    if not results['i2c_device']:
        print("\n2. Check MPU6050 wiring:")
        print("   VCC → 3.3V (Pin 1)")
        print("   GND → GND (Pin 6)")
        print("   SDA → GPIO2 (Pin 3)")
        print("   SCL → GPIO3 (Pin 5)")
    
    if not results['mpu6050_library']:
        print("\n3. Install mpu6050 library:")
        print("   pip3 install mpu6050-raspberrypi")
    
    if results['direct_access'] and results['reader_class'] and not results['api_endpoint']:
        print("\n4. Web interface needs restart:")
        print("   The sensor works but web interface shows wrong data.")
        print("   Stop the web interface and start it again:")
        print("   cd ros2_ws  # from project root")
        print("   python3 src/my_robot_automation/scripts/web_robot_interface.py")
    
    if not results['web_interface']:
        print("\n5. Start web interface:")
        print("   cd ros2_ws  # from project root")
        print("   python3 src/my_robot_automation/scripts/web_robot_interface.py")
    
    print("\n" + "=" * 70 + "\n")

if __name__ == '__main__':
    main()

