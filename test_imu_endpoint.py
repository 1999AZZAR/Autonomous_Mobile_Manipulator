#!/usr/bin/env python3
"""
Test script to verify IMU data endpoint on web interface
"""

import requests
import json
import time

def test_imu_endpoint(host='localhost', port=8000):
    """Test the IMU endpoint"""
    url = f'http://{host}:{port}/api/robot/imu/position'
    
    print(f'Testing IMU endpoint: {url}')
    print('-' * 60)
    
    try:
        response = requests.get(url, timeout=5)
        print(f'Status Code: {response.status_code}')
        print(f'Response Headers: {dict(response.headers)}')
        print()
        
        if response.status_code == 200:
            data = response.json()
            print('Response Data:')
            print(json.dumps(data, indent=2))
            print()
            
            if data.get('success'):
                imu_data = data.get('data', {})
                print('IMU Data Summary:')
                print(f"  Orientation (Roll, Pitch, Yaw):")
                print(f"    X (Roll):  {imu_data.get('orientation', {}).get('x', 'N/A')}°")
                print(f"    Y (Pitch): {imu_data.get('orientation', {}).get('y', 'N/A')}°")
                print(f"    Z (Yaw):   {imu_data.get('orientation', {}).get('z', 'N/A')}°")
                print(f"  Angular Velocity:")
                print(f"    X: {imu_data.get('angular_velocity', {}).get('x', 'N/A')} deg/s")
                print(f"    Y: {imu_data.get('angular_velocity', {}).get('y', 'N/A')} deg/s")
                print(f"    Z: {imu_data.get('angular_velocity', {}).get('z', 'N/A')} deg/s")
                print(f"  Linear Acceleration:")
                print(f"    X: {imu_data.get('linear_acceleration', {}).get('x', 'N/A')} m/s²")
                print(f"    Y: {imu_data.get('linear_acceleration', {}).get('y', 'N/A')} m/s²")
                print(f"    Z: {imu_data.get('linear_acceleration', {}).get('z', 'N/A')} m/s²")
                print(f"  Temperature: {imu_data.get('temperature', 'N/A')}°C")
            else:
                print(f'Error: {data.get("error", "Unknown error")}')
        else:
            print(f'Request failed with status code: {response.status_code}')
            print(f'Response: {response.text}')
            
    except requests.exceptions.ConnectionError:
        print('ERROR: Could not connect to web interface')
        print(f'Make sure the web interface is running on {host}:{port}')
    except requests.exceptions.Timeout:
        print('ERROR: Request timed out')
    except Exception as e:
        print(f'ERROR: {type(e).__name__}: {str(e)}')
    
    print('-' * 60)

def continuous_test(host='localhost', port=8000, interval=2, count=5):
    """Continuously test IMU endpoint"""
    print(f'Reading IMU data {count} times with {interval}s interval...\n')
    
    for i in range(count):
        print(f'=== Reading {i+1}/{count} ===')
        test_imu_endpoint(host, port)
        if i < count - 1:
            time.sleep(interval)

if __name__ == '__main__':
    import sys
    
    host = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
    
    print('IMU Endpoint Test Script')
    print('=' * 60)
    print(f'Target: {host}:{port}')
    print()
    
    # Single test
    test_imu_endpoint(host, port)
    
    # Ask for continuous test
    print()
    response = input('Run continuous test? (y/n): ')
    if response.lower() == 'y':
        continuous_test(host, port)

