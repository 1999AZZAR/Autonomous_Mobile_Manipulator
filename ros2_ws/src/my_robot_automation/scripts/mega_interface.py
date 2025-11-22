"""
Arduino Mega Serial Interface
Handles communication with Arduino Mega for motor control and sensor reading
"""

import serial
import time
import logging
from config import MEGA_BAUDRATE, MEGA_TIMEOUT, MEGA_WRITE_TIMEOUT, MEGA_PORTS

logger = logging.getLogger(__name__)

class MegaInterface:
    """Serial communication interface with Arduino Mega"""

    def __init__(self):
        self.mega_serial = None
        self.mega_connected = False
        self.connect_to_mega()

    def connect_to_mega(self):
        """Connect to Arduino Mega via serial"""
        possible_ports = MEGA_PORTS

        for port in possible_ports:
            try:
                logger.info(f'Attempting to connect to Mega on {port}')
                self.mega_serial = serial.Serial(
                    port=port,
                    baudrate=MEGA_BAUDRATE,
                    timeout=MEGA_TIMEOUT,
                    write_timeout=MEGA_WRITE_TIMEOUT
                )
                time.sleep(2)  # Wait for connection

                # Test the connection by sending a status request
                self.mega_serial.write(b'p\n')  # Status command
                self.mega_serial.flush()

                # Try to read response
                response = self.mega_serial.readline().decode().strip()
                if response:
                    self.mega_connected = True
                    logger.info(f'Successfully connected to Arduino Mega on {port}')
                    return

            except Exception as e:
                logger.warn(f'Failed to connect on {port}: {e}')
                if self.mega_serial:
                    try:
                        self.mega_serial.close()
                    except:
                        pass
                self.mega_serial = None
                continue

        self.mega_connected = False
        self.mega_serial = None
        logger.error("Failed to connect to Arduino Mega on any available port")
        logger.info('Available serial ports: ' + ', '.join(possible_ports))

    def send_command_to_mega(self, command):
        """Send command to Arduino Mega"""
        if not self.mega_connected or not self.mega_serial:
            logger.warning(f"Cannot send command '{command}' - Mega not connected")
            return False

        try:
            logger.debug(f"Sending command to Mega: {command}")
            self.mega_serial.write(f"{command}\n".encode())
            self.mega_serial.flush()
            return True
        except Exception as e:
            logger.error(f"Failed to send command '{command}' to Mega: {str(e)}")
            # Try to reconnect
            self.connect_to_mega()
            return False

    def control_gripper(self, command):
        """Send gripper control command to Mega"""
        commands = {
            'open_full': 'no',
            'open_half': 'nh',
            'close': 'nc'
        }

        mega_command = commands.get(command)
        if mega_command:
            return self.send_command_to_mega(mega_command)
        else:
            logger.error(f"Unknown gripper command: {command}")
            return False

    def set_gripper_tilt(self, angle):
        """Set gripper tilt angle (0-180 degrees)"""
        if not (0 <= angle <= 180):
            logger.error(f"Invalid gripper tilt angle: {angle} (must be 0-180)")
            return False

        mega_command = f"ta{int(angle)}"
        return self.send_command_to_mega(mega_command)

    def set_gripper_neck(self, position):
        """Set gripper neck position (-1 to 1, continuous servo)"""
        # Note: Mega may not support gripper neck control yet
        logger.warning(f"[NOT IMPLEMENTED] Gripper neck control not available on Mega: {position}")
        return False

    def set_speed(self, speed_percent):
        """Set robot speed (0-100%)"""
        if not (0 <= speed_percent <= 100):
            logger.error(f"Invalid speed percentage: {speed_percent} (must be 0-100)")
            return False

        # Convert UI percentage (0-100%) to Mega speed command (50-100%)
        # Mega supports: '5'(50%) '6'(60%) '7'(70%) '8'(80%) '9'(90%) '0'(100%)
        if speed_percent >= 83:      # 83-100% UI → 100% Mega
            mega_command = '0'
            mega_percent = 100
        elif speed_percent >= 67:    # 67-82% UI → 90% Mega
            mega_command = '9'
            mega_percent = 90
        elif speed_percent >= 50:    # 50-66% UI → 80% Mega
            mega_command = '8'
            mega_percent = 80
        elif speed_percent >= 33:    # 33-49% UI → 70% Mega
            mega_command = '7'
            mega_percent = 70
        elif speed_percent >= 17:    # 17-32% UI → 60% Mega
            mega_command = '6'
            mega_percent = 60
        else:                       # 0-16% UI → 50% Mega
            mega_command = '5'
            mega_percent = 50

        logger.info(f'UI Speed: {speed_percent}% → Mega Speed: {mega_percent}% (cmd: {mega_command})')
        return self.send_command_to_mega(mega_command)

    def toggle_turbo(self):
        """Toggle turbo mode"""
        return self.send_command_to_mega('o')

    def control_wheel(self, wheel_id, speed):
        """Control individual wheel speed (-100 to 100)"""
        if not (0 <= wheel_id <= 3):
            logger.error(f"Invalid wheel ID: {wheel_id} (must be 0-3)")
            return False

        if not (-100 <= speed <= 100):
            logger.error(f"Invalid wheel speed: {speed} (must be -100 to 100)")
            return False

        mega_command = f"w{wheel_id}{speed}"
        return self.send_command_to_mega(mega_command)

    def stop_all_wheels(self):
        """Stop all wheels"""
        return self.send_command_to_mega('wstop')

    def move_robot(self, direction, speed=0.5):
        """Move robot in specified direction"""
        commands = {
            'forward': 'f',
            'backward': 'b',
            'left': 'l',
            'right': 'r'
        }

        mega_command = commands.get(direction.lower())
        if mega_command:
            # Send direction command
            success = self.send_command_to_mega(mega_command)
            if success and speed != 0.5:
                # Set speed if different from default
                speed_percent = int(speed * 100)
                self.set_speed(speed_percent)
            return success
        else:
            logger.error(f"Unknown direction: {direction}")
            return False

    def stop_robot(self):
        """Stop robot movement"""
        return self.send_command_to_mega('s')

    def turn_robot(self, direction, speed=0.5):
        """Turn robot in specified direction"""
        commands = {
            'left': 'q',
            'right': 'e'
        }

        mega_command = commands.get(direction.lower())
        if mega_command:
            return self.send_command_to_mega(mega_command)
        else:
            logger.error(f"Unknown turn direction: {direction}")
            return False

    def get_status(self):
        """Get Mega status"""
        return self.send_command_to_mega('p')

    def enable_perimeter_safety(self):
        """Enable perimeter safety"""
        return self.send_command_to_mega('se')

    def disable_perimeter_safety(self):
        """Disable perimeter safety"""
        return self.send_command_to_mega('sd')

    def test_limit_switches(self):
        """Test limit switches"""
        return self.send_command_to_mega('ls')

    def cleanup(self):
        """Clean up serial connection"""
        if self.mega_serial and self.mega_connected:
            try:
                self.mega_serial.close()
                logger.info('Mega serial connection closed')
                self.mega_connected = False
            except Exception as e:
                logger.error(f'Error closing Mega serial: {str(e)}')
