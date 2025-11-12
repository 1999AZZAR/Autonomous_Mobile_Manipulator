#!/usr/bin/env python3
"""
PG23 Motor Controller
Implements serial communication for PG23 built-in encoder motors
"""

import time
import struct

try:
    import lgpio
    LGPIO_AVAILABLE = True
except ImportError:
    LGPIO_AVAILABLE = False
    print("WARNING: lgpio not available - motor control will not work")

class PG23MotorController:
    """
    Controller for PG23 motors with built-in driver and encoder
    Uses serial communication via DATA(A) and DATA(B) pins
    """
    
    # Default serial parameters (adjust based on motor datasheet)
    DEFAULT_BAUD_RATE = 9600
    DEFAULT_DATA_BITS = 8
    DEFAULT_STOP_BITS = 1
    DEFAULT_PARITY = 'N'  # None
    
    # Motor commands (these may need adjustment based on actual protocol)
    CMD_STOP = b'\x00'      # Stop motor
    CMD_ENABLE = b'\x01'    # Enable motor
    CMD_DISABLE = b'\x02'   # Disable motor
    CMD_FORWARD = b'\x03'   # Forward direction
    CMD_REVERSE = b'\x04'   # Reverse direction
    
    def __init__(self, gpio_handle, tx_pin, rx_pin, motor_name="Motor", baud_rate=None):
        """
        Initialize PG23 motor controller
        
        Args:
            gpio_handle: lgpio handle
            tx_pin: GPIO pin for DATA(A) - Serial TX
            rx_pin: GPIO pin for DATA(B) - Serial RX
            motor_name: Name for logging
            baud_rate: Serial baud rate (default: 9600)
        """
        self.gpio_handle = gpio_handle
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.motor_name = motor_name
        self.baud_rate = baud_rate or self.DEFAULT_BAUD_RATE
        
        # Motor state
        self.is_enabled = False
        self.current_speed = 0.0
        self.current_direction = 0  # 0 = stop, 1 = forward, -1 = reverse
        
        # Initialize serial communication
        self._init_serial()
        
        # Send STOP command immediately to ensure motor is stopped
        self.stop()
        
    def _init_serial(self):
        """Initialize serial communication on GPIO pins"""
        if not LGPIO_AVAILABLE:
            print(f"WARNING [{self.motor_name}]: lgpio not available - serial init skipped")
            return
            
        try:
            # Configure TX pin as output
            lgpio.gpio_claim_output(self.gpio_handle, self.tx_pin)
            
            # Configure RX pin as input
            lgpio.gpio_claim_input(self.gpio_handle, self.rx_pin)
            
            # Set TX pin to idle state (HIGH for UART)
            lgpio.gpio_write(self.gpio_handle, self.tx_pin, 1)
            
            print(f"✓ [{self.motor_name}] Serial initialized: TX=GPIO{self.tx_pin}, RX=GPIO{self.rx_pin}, Baud={self.baud_rate}")
            
        except Exception as e:
            print(f"ERROR [{self.motor_name}]: Failed to initialize serial: {e}")
    
    def _send_byte(self, byte_data):
        """
        Send a single byte via software serial
        Implements UART protocol: Start bit (0) + 8 data bits + Stop bit (1)
        """
        if not LGPIO_AVAILABLE or self.gpio_handle is None:
            return False
            
        try:
            bit_time = 1.0 / self.baud_rate
            
            # Ensure we have a byte
            if isinstance(byte_data, int):
                byte_value = byte_data
            elif isinstance(byte_data, bytes):
                byte_value = byte_data[0]
            else:
                byte_value = ord(byte_data)
            
            # Start bit (LOW)
            lgpio.gpio_write(self.gpio_handle, self.tx_pin, 0)
            time.sleep(bit_time)
            
            # Data bits (LSB first)
            for i in range(8):
                bit = (byte_value >> i) & 1
                lgpio.gpio_write(self.gpio_handle, self.tx_pin, bit)
                time.sleep(bit_time)
            
            # Stop bit (HIGH)
            lgpio.gpio_write(self.gpio_handle, self.tx_pin, 1)
            time.sleep(bit_time)
            
            return True
            
        except Exception as e:
            print(f"ERROR [{self.motor_name}]: Failed to send byte: {e}")
            return False
    
    def _send_command(self, command, speed_byte=None):
        """
        Send command to motor
        
        Args:
            command: Command byte
            speed_byte: Optional speed value (0-255)
        """
        if not self._send_byte(command):
            return False
            
        # If speed is provided, send it as second byte
        if speed_byte is not None:
            self._send_byte(speed_byte)
            
        return True
    
    def enable(self):
        """Enable motor controller"""
        if self._send_command(self.CMD_ENABLE):
            self.is_enabled = True
            print(f"✓ [{self.motor_name}] Motor enabled")
            return True
        return False
    
    def disable(self):
        """Disable motor controller (motor will free spin)"""
        if self._send_command(self.CMD_DISABLE):
            self.is_enabled = False
            self.current_speed = 0.0
            self.current_direction = 0
            print(f"✓ [{self.motor_name}] Motor disabled")
            return True
        return False
    
    def stop(self):
        """Stop motor immediately (brake mode)"""
        if self._send_command(self.CMD_STOP):
            self.current_speed = 0.0
            self.current_direction = 0
            print(f"✓ [{self.motor_name}] Motor stopped")
            return True
        return False
    
    def set_speed(self, speed, direction=1):
        """
        Set motor speed and direction
        
        Args:
            speed: Speed value 0.0 to 1.0 (0 = stop, 1.0 = max speed)
            direction: 1 for forward, -1 for reverse
        """
        # Clamp speed
        speed = max(0.0, min(1.0, abs(speed)))
        
        # If speed is 0, stop motor
        if speed == 0.0:
            return self.stop()
        
        # Convert speed to byte (0-255)
        speed_byte = int(speed * 255)
        
        # Send direction command
        if direction > 0:
            cmd = self.CMD_FORWARD
            self.current_direction = 1
        else:
            cmd = self.CMD_REVERSE
            self.current_direction = -1
        
        if self._send_command(cmd, speed_byte):
            self.current_speed = speed
            print(f"✓ [{self.motor_name}] Speed set: {speed:.2f}, Direction: {'Forward' if direction > 0 else 'Reverse'}")
            return True
        
        return False
    
    def forward(self, speed=0.5):
        """Move motor forward"""
        return self.set_speed(speed, direction=1)
    
    def reverse(self, speed=0.5):
        """Move motor reverse"""
        return self.set_speed(speed, direction=-1)
    
    def get_status(self):
        """Get current motor status"""
        return {
            'enabled': self.is_enabled,
            'speed': self.current_speed,
            'direction': self.current_direction,
            'tx_pin': self.tx_pin,
            'rx_pin': self.rx_pin,
            'baud_rate': self.baud_rate
        }


class PG23MotorManager:
    """
    Manager for multiple PG23 motors
    Ensures all motors are stopped by default
    """
    
    def __init__(self, gpio_handle):
        """
        Initialize motor manager
        
        Args:
            gpio_handle: lgpio handle
        """
        self.gpio_handle = gpio_handle
        self.motors = {}
        self.initialized = False
    
    def add_motor(self, name, tx_pin, rx_pin, baud_rate=None):
        """
        Add a motor to the manager
        
        Args:
            name: Motor name (e.g., 'front_left', 'front_right')
            tx_pin: GPIO pin for DATA(A) - Serial TX
            rx_pin: GPIO pin for DATA(B) - Serial RX
            baud_rate: Serial baud rate
        """
        motor = PG23MotorController(
            self.gpio_handle,
            tx_pin,
            rx_pin,
            motor_name=name,
            baud_rate=baud_rate
        )
        self.motors[name] = motor
        return motor
    
    def stop_all(self):
        """Stop all motors"""
        for name, motor in self.motors.items():
            motor.stop()
        print("✓ All motors stopped")
    
    def disable_all(self):
        """Disable all motors"""
        for name, motor in self.motors.items():
            motor.disable()
        print("✓ All motors disabled")
    
    def get_motor(self, name):
        """Get motor by name"""
        return self.motors.get(name)
    
    def __getitem__(self, name):
        """Allow dictionary-like access"""
        return self.motors[name]
    
    def __contains__(self, name):
        """Check if motor exists"""
        return name in self.motors

