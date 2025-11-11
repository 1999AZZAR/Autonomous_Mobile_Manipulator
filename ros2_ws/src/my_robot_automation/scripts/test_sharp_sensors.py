#!/usr/bin/env python3

import spidev
import time
import math

class SharpSensorTester:
    def __init__(self):
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000
        
        # Configuration
        self.adc_vref = 3.3
        self.adc_resolution = 1024
        
        # Sensor names for display
        self.sensor_names = {
            0: 'Left Front',
            1: 'Left Back',
            2: 'Right Front',
            3: 'Right Back',
            4: 'Back Left',
            5: 'Back Right'
        }
    
    def read_adc_channel(self, channel):
        if channel < 0 or channel > 7:
            raise ValueError("ADC channel must be between 0 and 7")
        
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data
    
    def adc_to_voltage(self, adc_value):
        return (adc_value * self.adc_vref) / self.adc_resolution
    
    def sharp_gp2y0a02_voltage_to_distance(self, voltage):
        try:
            if voltage < 0.3:
                return None
            if voltage > 2.8:
                return 200
            
            distance_cm = 60 * (voltage ** -1.1) - 1
            
            if distance_cm < 20:
                distance_cm = 20
            elif distance_cm > 150:
                distance_cm = 150
            
            distance_mm = round(distance_cm * 10)
            return distance_mm
            
        except (ValueError, ZeroDivisionError):
            return None
    
    def read_sharp_sensor(self, channel):
        try:
            readings = []
            for _ in range(5):
                adc_value = self.read_adc_channel(channel)
                voltage = self.adc_to_voltage(adc_value)
                distance = self.sharp_gp2y0a02_voltage_to_distance(voltage)
                if distance is not None:
                    readings.append(distance)
                time.sleep(0.001)
            
            if readings:
                readings.sort()
                return readings[len(readings) // 2]
            else:
                return None
                
        except Exception as e:
            print(f'Error reading sensor on channel {channel}: {str(e)}')
            return None
    
    def test_single_sensor(self, channel):
        print(f"\nTesting {self.sensor_names.get(channel, f'Channel {channel}')}...")
        print("-" * 50)
        
        for i in range(10):
            adc_value = self.read_adc_channel(channel)
            voltage = self.adc_to_voltage(adc_value)
            distance = self.sharp_gp2y0a02_voltage_to_distance(voltage)
            
            print(f"Sample {i+1}: ADC={adc_value:4d}, Voltage={voltage:5.3f}V, Distance={distance if distance else 'N/A':>4} mm")
            time.sleep(0.1)
        
        print("\nFiltered reading:")
        distance = self.read_sharp_sensor(channel)
        print(f"Distance: {distance if distance else 'N/A'} mm")
    
    def test_all_sensors(self):
        print("\n" + "=" * 60)
        print("Sharp GP2Y0A02YK0F Sensor Array Test")
        print("=" * 60)
        
        while True:
            print("\n" + "-" * 60)
            print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            print("-" * 60)
            
            for channel in range(6):
                sensor_name = self.sensor_names.get(channel)
                distance = self.read_sharp_sensor(channel)
                
                if distance:
                    distance_cm = distance / 10
                    bar_length = int((distance / 1500) * 30)
                    bar = "█" * bar_length + "░" * (30 - bar_length)
                    print(f"{sensor_name:12} | {distance:4d}mm ({distance_cm:5.1f}cm) | {bar}")
                else:
                    print(f"{sensor_name:12} | No signal or out of range")
            
            time.sleep(0.5)
    
    def calibration_mode(self, channel):
        print(f"\nCalibration Mode - {self.sensor_names.get(channel, f'Channel {channel}')}")
        print("=" * 60)
        print("Place object at known distances and record voltage values")
        print("Press Ctrl+C to exit")
        print("-" * 60)
        
        try:
            while True:
                distance_input = input("\nEnter actual distance in cm (or 'q' to quit): ")
                if distance_input.lower() == 'q':
                    break
                
                try:
                    actual_distance = float(distance_input)
                    
                    # Take multiple samples
                    voltages = []
                    for i in range(10):
                        adc_value = self.read_adc_channel(channel)
                        voltage = self.adc_to_voltage(adc_value)
                        voltages.append(voltage)
                        time.sleep(0.05)
                    
                    avg_voltage = sum(voltages) / len(voltages)
                    print(f"Actual distance: {actual_distance} cm")
                    print(f"Average voltage: {avg_voltage:.4f} V")
                    print(f"Calibration point: ({actual_distance}, {avg_voltage:.4f})")
                    
                except ValueError:
                    print("Invalid input. Please enter a number.")
                    
        except KeyboardInterrupt:
            print("\nCalibration mode exited")
    
    def cleanup(self):
        self.spi.close()

def main():
    print("\nSharp GP2Y0A02YK0F Sensor Test Utility")
    print("=====================================\n")
    
    tester = SharpSensorTester()
    
    try:
        while True:
            print("\nSelect test mode:")
            print("1. Test single sensor")
            print("2. Monitor all sensors (continuous)")
            print("3. Calibration mode")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == '1':
                print("\nSelect sensor:")
                for ch, name in tester.sensor_names.items():
                    print(f"{ch}. {name}")
                
                try:
                    channel = int(input("\nEnter channel (0-5): "))
                    if 0 <= channel <= 5:
                        tester.test_single_sensor(channel)
                    else:
                        print("Invalid channel number")
                except ValueError:
                    print("Invalid input")
            
            elif choice == '2':
                print("\nStarting continuous monitoring (Press Ctrl+C to stop)...")
                try:
                    tester.test_all_sensors()
                except KeyboardInterrupt:
                    print("\n\nMonitoring stopped")
            
            elif choice == '3':
                print("\nSelect sensor for calibration:")
                for ch, name in tester.sensor_names.items():
                    print(f"{ch}. {name}")
                
                try:
                    channel = int(input("\nEnter channel (0-5): "))
                    if 0 <= channel <= 5:
                        tester.calibration_mode(channel)
                    else:
                        print("Invalid channel number")
                except ValueError:
                    print("Invalid input")
            
            elif choice == '4':
                print("\nExiting...")
                break
            
            else:
                print("Invalid choice. Please enter 1-4.")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        tester.cleanup()
        print("Cleanup complete. Goodbye!")

if __name__ == '__main__':
    main()

