"""
Arduino Mega Serial Interface
Handles communication with Arduino Mega for motor control and sensor reading
Enhanced with automatic reconnection, connection monitoring, and robust error handling
"""

import serial
import time
import threading
import logging
from typing import Optional, Callable, Dict, Any
from config import MEGA_BAUDRATE, MEGA_TIMEOUT, MEGA_WRITE_TIMEOUT, MEGA_PORTS

logger = logging.getLogger(__name__)

class MegaInterface:
    """Enhanced serial communication interface with Arduino Mega"""

    def __init__(self, auto_reconnect=True, max_reconnect_attempts=5, reconnect_interval=5.0):
        # Connection state
        self.mega_serial: Optional[serial.Serial] = None
        self.mega_connected = False
        self.current_port = None

        # Reconnection settings
        self.auto_reconnect = auto_reconnect
        self.max_reconnect_attempts = max_reconnect_attempts
        self.reconnect_interval = reconnect_interval
        self.reconnect_attempts = 0
        self.last_reconnect_time = 0

        # Monitoring and callbacks
        self.connection_callbacks: list[Callable[[bool], None]] = []
        self.last_activity_time = time.time()
        self.connection_health_check_interval = 30.0  # Check every 30 seconds

        # Threading
        self._lock = threading.RLock()
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_monitor = False

        # Statistics
        self.stats = {
            'commands_sent': 0,
            'commands_failed': 0,
            'bytes_sent': 0,
            'bytes_received': 0,
            'connection_drops': 0,
            'reconnections': 0,
            'uptime_start': time.time()
        }

        # Initial connection attempt
        self.connect_to_mega()

        # Start monitoring thread if auto-reconnect is enabled
        if self.auto_reconnect:
            self._start_monitoring()

    def _start_monitoring(self):
        """Start background monitoring thread"""
        if self._monitor_thread and self._monitor_thread.is_alive():
            return

        self._stop_monitor = False
        self._monitor_thread = threading.Thread(target=self._monitor_connection, daemon=True)
        self._monitor_thread.start()
        logger.info("Mega connection monitoring started")

    def _stop_monitoring(self):
        """Stop background monitoring thread"""
        self._stop_monitor = True
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)

    def _monitor_connection(self):
        """Background thread to monitor connection health"""
        while not self._stop_monitor:
            try:
                time.sleep(self.connection_health_check_interval)

                if self._stop_monitor:
                    break

                with self._lock:
                    if self.mega_connected and self.mega_serial:
                        # Check if connection is still alive
                        try:
                            # Try a simple health check
                            if self.mega_serial.in_waiting == 0:
                                # Send a minimal status check (non-blocking)
                                self.mega_serial.write(b'p\n')
                                self.mega_serial.flush()

                                # Check if we can still write
                                current_time = time.time()
                                if current_time - self.last_activity_time > 60.0:  # No activity for 1 minute
                                    logger.warning("Mega connection appears stale, attempting health check")
                                    # This will trigger reconnection if needed
                        except (serial.SerialException, OSError) as e:
                            logger.warning(f"Mega connection health check failed: {e}")
                            self._handle_connection_loss()

                    elif self.auto_reconnect and not self.mega_connected:
                        # Attempt reconnection
                        self._attempt_reconnection()

            except Exception as e:
                logger.error(f"Connection monitoring error: {e}")
                time.sleep(5.0)  # Back off on errors

    def add_connection_callback(self, callback: Callable[[bool], None]):
        """Add callback for connection status changes"""
        self.connection_callbacks.append(callback)

    def _notify_connection_callbacks(self, connected: bool):
        """Notify all connection callbacks"""
        for callback in self.connection_callbacks:
            try:
                callback(connected)
            except Exception as e:
                logger.error(f"Connection callback error: {e}")

    def _handle_connection_loss(self):
        """Handle unexpected connection loss"""
        with self._lock:
            if self.mega_connected:
                self.mega_connected = False
                self.stats['connection_drops'] += 1
                logger.warning("Mega connection lost")

                # Close the connection
                if self.mega_serial:
                    try:
                        self.mega_serial.close()
                    except Exception as e:
                        logger.error(f"Error closing lost connection: {e}")
                    self.mega_serial = None

                self._notify_connection_callbacks(False)

                # Trigger reconnection if enabled
                if self.auto_reconnect:
                    threading.Thread(target=self._attempt_reconnection, daemon=True).start()

    def _attempt_reconnection(self):
        """Attempt to reconnect to Mega with exponential backoff"""
        current_time = time.time()

        # Check if we're attempting too frequently
        if current_time - self.last_reconnect_time < self.reconnect_interval:
            return

        if self.reconnect_attempts >= self.max_reconnect_attempts:
            logger.error(f"Max reconnection attempts ({self.max_reconnect_attempts}) reached")
            return

        self.reconnect_attempts += 1
        self.last_reconnect_time = current_time

        logger.info(f"Attempting reconnection ({self.reconnect_attempts}/{self.max_reconnect_attempts})")

        # Try to reconnect
        if self.connect_to_mega():
            self.reconnect_attempts = 0
            self.stats['reconnections'] += 1
            logger.info("Successfully reconnected to Mega")
        else:
            # Exponential backoff
            next_attempt = min(self.reconnect_interval * (2 ** (self.reconnect_attempts - 1)), 300)  # Max 5 minutes
            logger.warning(f"Reconnection failed, next attempt in {next_attempt:.1f} seconds")

    def connect_to_mega(self) -> bool:
        """Connect to Arduino Mega via serial with enhanced error handling"""
        possible_ports = MEGA_PORTS

        with self._lock:
            # Close existing connection if any
            if self.mega_serial:
                try:
                    self.mega_serial.close()
                except Exception as e:
                    logger.debug(f"Error closing existing connection: {e}")
                self.mega_serial = None

            for port in possible_ports:
                try:
                    logger.info(f'Attempting to connect to Mega on {port}')

                    # Create serial connection with additional error handling
                    self.mega_serial = serial.Serial(
                        port=port,
                        baudrate=MEGA_BAUDRATE,
                        timeout=MEGA_TIMEOUT,
                        write_timeout=MEGA_WRITE_TIMEOUT,
                        exclusive=True  # Prevent other processes from accessing
                    )

                    # Wait for connection to stabilize
                    time.sleep(2.5)

                    # Clear any pending data
                    self.mega_serial.reset_input_buffer()
                    self.mega_serial.reset_output_buffer()

                    # Test the connection multiple times for reliability
                    connection_tested = False
                    for test_attempt in range(3):
                        try:
                            logger.debug(f'Connection test {test_attempt + 1}/3')

                            # Send status command
                            self.mega_serial.write(b'p\n')
                            self.mega_serial.flush()

                            # Wait for response with timeout
                            start_time = time.time()
                            while time.time() - start_time < 3.0:  # 3 second timeout
                                if self.mega_serial.in_waiting > 0:
                                    response = self.mega_serial.readline().decode('utf-8', errors='ignore').strip()
                                    if response and len(response) > 0:
                                        connection_tested = True
                                        break
                                time.sleep(0.1)

                            if connection_tested:
                                break

                        except Exception as e:
                            logger.debug(f'Connection test {test_attempt + 1} failed: {e}')
                            continue

                    if connection_tested:
                        self.mega_connected = True
                        self.current_port = port
                        self.last_activity_time = time.time()
                        logger.info(f'Successfully connected to Arduino Mega on {port}')

                        # Notify callbacks
                        self._notify_connection_callbacks(True)
                        return True

                    # If we get here, connection test failed
                    logger.warning(f'Connection test failed on {port}')
                    self.mega_serial.close()
                    self.mega_serial = None

                except serial.SerialException as e:
                    logger.warning(f'Serial exception on {port}: {e}')
                    if self.mega_serial:
                        try:
                            self.mega_serial.close()
                        except:
                            pass
                        self.mega_serial = None
                    continue

                except Exception as e:
                    logger.warning(f'Unexpected error connecting to {port}: {e}')
                    if self.mega_serial:
                        try:
                            self.mega_serial.close()
                        except:
                            pass
                        self.mega_serial = None
                    continue

            # If we get here, no connection succeeded
            self.mega_connected = False
            self.mega_serial = None
            self.current_port = None
            logger.error("Failed to connect to Arduino Mega on any available port")
            logger.info(f'Available serial ports: {", ".join(possible_ports)}')
            return False

    def send_command_to_mega(self, command: str) -> bool:
        """Send command to Arduino Mega with enhanced error handling"""
        with self._lock:
            if not self.mega_connected or not self.mega_serial:
                logger.warning(f"Cannot send command '{command}' - Mega not connected")
                self.stats['commands_failed'] += 1
                return False

            try:
                logger.debug(f"Sending command to Mega: {command}")

                # Encode command
                command_bytes = f"{command}\n".encode('utf-8')
                self.stats['bytes_sent'] += len(command_bytes)

                # Send command
                self.mega_serial.write(command_bytes)
                self.mega_serial.flush()

                # Update activity timestamp
                self.last_activity_time = time.time()

                self.stats['commands_sent'] += 1
                logger.debug(f"Command sent successfully: {command}")
                return True

            except serial.SerialTimeoutException as e:
                logger.error(f"Timeout sending command '{command}' to Mega: {str(e)}")
                self.stats['commands_failed'] += 1
                self._handle_connection_loss()
                return False

            except serial.SerialException as e:
                logger.error(f"Serial error sending command '{command}' to Mega: {str(e)}")
                self.stats['commands_failed'] += 1
                self._handle_connection_loss()
                return False

            except Exception as e:
                logger.error(f"Unexpected error sending command '{command}' to Mega: {str(e)}")
                self.stats['commands_failed'] += 1
                self._handle_connection_loss()
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

    def move_robot(self, direction, speed=0.5, duration=0.0):
        """Move robot in specified direction.
        If duration > 0, auto-stops after that many seconds.
        
        Args:
            direction: 'forward', 'backward', 'forward-left', 'forward-right',
                       'backward-left', 'backward-right'
            speed: 0.0-1.0 (maps to Mega speed 50-100%)
            duration: seconds before auto-stop (0 = no auto-stop)
        """
        commands = {
            'forward': 'f',
            'backward': 'b',
            'forward-left': 'q',
            'forward-right': 'e',
            'backward-left': 'z',
            'backward-right': 'x'
        }

        mega_command = commands.get(direction.lower())
        if mega_command:
            # Send direction command
            success = self.send_command_to_mega(mega_command)
            if success and speed != 0.5:
                # Set speed if different from default
                speed_percent = int(speed * 100)
                self.set_speed(speed_percent)
            # Auto-stop after duration
            if success and duration > 0:
                def _stop_after():
                    time.sleep(duration)
                    self.send_command_to_mega('s')
                threading.Thread(target=_stop_after, daemon=True).start()
            return success
        else:
            logger.error(f"Unknown direction: {direction}")
            return False

    def stop_robot(self):
        """Stop robot movement"""
        return self.send_command_to_mega('s')

    def turn_robot(self, direction, speed=0.5, angle=90.0, get_heading_fn=None):
        """Turn robot using IMU. Blocks until target angle reached.
        
        Args:
            direction: 'left' or 'right'
            speed: 0.0-1.0 (maps to Mega speed 50-100%)
            angle: target angle in degrees (default 90)
            get_heading_fn: callable returning current heading in degrees.
                           If None, falls back to time-based stop.
        Returns:
            True if completed, False on error
        """
        commands = {'left': 't', 'right': 'y'}
        mega_command = commands.get(direction.lower())
        if not mega_command:
            logger.error(f"Unknown turn direction: {direction}")
            return False

        # Set speed
        speed_percent = int(speed * 100)
        self.set_speed(speed_percent)

        # Start rotation
        success = self.send_command_to_mega(mega_command)
        if not success:
            return False

        # If no IMU heading function, use time-based fallback
        if get_heading_fn is None:
            duration = angle / 90.0 * 1.5  # rough estimate: 1.5s per 90 degrees
            time.sleep(duration)
            return self.stop_robot()

        # IMU-based: monitor heading until target reached
        import math
        start_heading = get_heading_fn()
        if start_heading is None:
            logger.warning("IMU heading unavailable, falling back to time-based")
            time.sleep(angle / 90.0 * 1.5)
            return self.stop_robot()

        # Calculate target heading
        if direction.lower() == 'left':
            target_heading = start_heading + angle
        else:
            target_heading = start_heading - angle

        # Normalize to -180..180
        while target_heading > 180:
            target_heading -= 360
        while target_heading < -180:
            target_heading += 360

        logger.info(f"Turn {direction}: start={start_heading:.1f} target={target_heading:.1f} angle={angle:.1f}")

        timeout = angle / 90.0 * 5.0  # 5s max per 90 degrees
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = get_heading_fn()
            if current is None:
                time.sleep(0.05)
                continue

            # Calculate error
            error = target_heading - current
            while error > 180:
                error -= 360
            while error < -180:
                error += 360

            # Check if close enough (threshold)
            if abs(error) < 3.0:  # 3 degree tolerance
                logger.info(f"Turn complete: heading={current:.1f}")
                return self.stop_robot()

            time.sleep(0.02)  # 50Hz check

        # Timeout - stop anyway
        logger.warning(f"Turn timeout after {timeout:.1f}s")
        return self.stop_robot()

    def rotate_robot(self, direction, speed=0.8, angle=180.0, get_heading_fn=None):
        """Rotate robot (same as turn, just higher default speed for bigger angles).
        
        Args:
            direction: 'left' or 'right'
            speed: 0.0-1.0 (default 0.8, higher than turn)
            angle: target angle in degrees (default 180 for half rotation)
            get_heading_fn: callable returning current heading in degrees.
        """
        return self.turn_robot(direction, speed=speed, angle=angle, get_heading_fn=get_heading_fn)

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

    def enable_sensor_publishing(self):
        """Enable sensor publishing"""
        return self.send_command_to_mega('spe')

    def disable_sensor_publishing(self):
        """Disable sensor publishing"""
        return self.send_command_to_mega('spd')

    def read_sensor_data(self):
        """Read sensor data from Mega using 'sr' command"""
        try:
            with self._lock:
                if not self.mega_connected or not self.mega_serial:
                    return None

                # Clear any pending data
                self.mega_serial.reset_input_buffer()

                # Send sensor reading command
                logger.debug("Sending 'sr' command to Mega")
                self.mega_serial.write(b'sr\n')
                self.mega_serial.flush()
                self.stats['commands_sent'] += 1
                self.stats['bytes_sent'] += 3

                # Wait for response with timeout
                start_time = time.time()
                response_lines = []

                while time.time() - start_time < 0.5:  # Reduced to 0.5 second timeout
                    if self.mega_serial.in_waiting > 0:
                        try:
                            line = self.mega_serial.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                response_lines.append(line)
                                self.stats['bytes_received'] += len(line)

                                # Check if we have the complete sensor data
                                if "========================" in line or "Detailed Sensor Readings" in line:
                                    # Wait minimal time for all data (reduced from 0.1s)
                                    time.sleep(0.05)
                                    while self.mega_serial.in_waiting > 0:
                                        line = self.mega_serial.readline().decode('utf-8', errors='ignore').strip()
                                        if line:
                                            response_lines.append(line)
                                            self.stats['bytes_received'] += len(line)
                                    break

                        except UnicodeDecodeError:
                            continue

                    time.sleep(0.005)  # Reduced polling interval

                if response_lines:
                    self.last_activity_time = time.time()
                    logger.debug(f"Received {len(response_lines)} lines from Mega in {(time.time() - start_time)*1000:.1f}ms")
                    return self._parse_sensor_response(response_lines)
                else:
                    logger.warning(f"No response received from Mega for sensor reading (timeout after {(time.time() - start_time)*1000:.1f}ms)")
                    return None

        except Exception as e:
            logger.error(f"Error reading sensor data from Mega: {str(e)}")
            self.stats['commands_failed'] += 1
            return None

    def _parse_sensor_response(self, response_lines):
        """Parse the sensor response from Mega"""
        try:
            sensor_data = {
                'laser_sensors': {},
                'ultrasonic_sensors': {},
                'line_sensors': {},
                'motor_status': {},
                'sync_status': None,
                'vff': None,
                'moving': None,
            }

            for line in response_lines:
                line = line.strip()

                # Look for the main SENSORS line with comma-separated values
                if line.startswith('SENSORS:'):
                    # Parse the comma-separated sensor values
                    sensors_part = line[8:]  # Remove 'SENSORS:' prefix
                    sensor_pairs = sensors_part.split(',')

                    for pair in sensor_pairs:
                        if ':' in pair:
                            key, value = pair.split(':', 1)
                            key = key.strip()
                            value = value.strip()

                            # Skip non-sensor data but parse motor/sync/vff/moving
                            if key.startswith('MTR'):
                                try:
                                    # MTR:id,setpoint,rpm,syncerr,position,output
                                    parts = value.split(',')
                                    if len(parts) >= 6:
                                        motor_id = int(parts[0])
                                        sensor_data['motor_status'][f'motor_{motor_id}'] = {
                                            'setpoint': float(parts[1]),
                                            'rpm': float(parts[2]),
                                            'sync_error': float(parts[3]),
                                            'position': float(parts[4]),
                                            'output': float(parts[5]),
                                        }
                                except (ValueError, IndexError):
                                    pass
                                continue
                            elif key == 'SYNC':
                                try:
                                    parts = value.split(',')
                                    if len(parts) >= 2:
                                        sensor_data['sync_status'] = {
                                            'target_rpm': float(parts[0]),
                                            'active_motors': int(parts[1]),
                                        }
                                except (ValueError, IndexError):
                                    pass
                                continue
                            elif key == 'VFF':
                                try:
                                    # VFF:x,y,m|vx,vy
                                    force_part, vel_part = value.split('|')
                                    fx, fy, mag = force_part.split(',')
                                    vx, vy = vel_part.split(',')
                                    sensor_data['vff'] = {
                                        'force_x': float(fx), 'force_y': float(fy),
                                        'magnitude': float(mag),
                                        'velocity_x': float(vx), 'velocity_y': float(vy),
                                    }
                                except (ValueError, IndexError):
                                    pass
                                continue
                            elif key == 'MOVING':
                                sensor_data['moving'] = value.strip() == '1'
                                continue

                            # Parse IR sensors (Sharp GP2Y0A02YK0F: ADC → mm)
                            if key.startswith('IR_'):
                                if value != 'INV':
                                    try:
                                        adc = float(value)
                                        # Sharp GP2Y0A02YK0F conversion: ADC → voltage → distance mm
                                        # Voltage = ADC * 5.0 / 1023.0
                                        # Distance (mm) = 27.86 / (voltage - 0.1) approx
                                        # Simplified: if ADC > 0, distance ≈ 12343.85 * ADC^(-1.269)
                                        # For safety, use simpler linear approx for common range
                                        if adc > 0:
                                            voltage = adc * 5.0 / 1023.0
                                            if voltage > 0.3:
                                                distance_mm = 27.86 / (voltage - 0.1)
                                            else:
                                                distance_mm = 1500.0  # max range
                                        else:
                                            distance_mm = 0.0

                                        sensor_map = {
                                            'IR_LF': 'left_front', 'IR_LB': 'left_back',
                                            'IR_RF': 'right_front', 'IR_RB': 'right_back',
                                            'IR_BL': 'back_left', 'IR_BR': 'back_right',
                                        }
                                        sensor_data['laser_sensors'][sensor_map[key]] = round(distance_mm, 1)
                                    except ValueError:
                                        pass

                            # Parse ultrasonic sensors
                            elif key.startswith('US_'):
                                if key == 'US_FL':  # Front Left
                                    try:
                                        # Convert cm to mm for consistency
                                        sensor_data['ultrasonic_sensors']['front_left'] = float(value) * 10
                                    except ValueError:
                                        pass
                                elif key == 'US_FR':  # Front Right
                                    try:
                                        sensor_data['ultrasonic_sensors']['front_right'] = float(value) * 10
                                    except ValueError:
                                        pass

            logger.info(f"Parsed sensor data: {len(sensor_data['laser_sensors'])} IR, {len(sensor_data['ultrasonic_sensors'])} ultrasonic sensors")
            return sensor_data

        except Exception as e:
            logger.error(f"Error parsing sensor response: {str(e)}")
            return None

    def set_individual_wheel_speed(self, wheel, speed):
        """Set individual wheel speed (w[wheel][speed])"""
        if not (0 <= wheel <= 3):
            logger.error(f"Invalid wheel ID: {wheel} (must be 0-3)")
            return False

        if not (-100 <= speed <= 100):
            logger.error(f"Invalid wheel speed: {speed} (must be -100 to 100)")
            return False

        command = f"w{wheel}{speed}"
        return self.send_command_to_mega(command)

    def stop_individual_wheel_control(self):
        """Stop all individual wheel control"""
        return self.send_command_to_mega('wstop')

    def read_available_data(self, max_lines: int = 100) -> list[str]:
        """Read available data from Mega serial buffer"""
        data_lines = []

        with self._lock:
            if not self.mega_connected or not self.mega_serial:
                return data_lines

            try:
                lines_read = 0
                while self.mega_serial.in_waiting > 0 and lines_read < max_lines:
                    try:
                        line = self.mega_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            data_lines.append(line)
                            self.stats['bytes_received'] += len(line.encode('utf-8'))
                            lines_read += 1
                            self.last_activity_time = time.time()
                    except UnicodeDecodeError:
                        # Skip lines that can't be decoded
                        continue

                    # Prevent reading too much at once
                    if lines_read >= max_lines:
                        break

            except (serial.SerialException, OSError) as e:
                logger.warning(f"Error reading from Mega: {e}")
                self._handle_connection_loss()

            except Exception as e:
                logger.error(f"Unexpected error reading from Mega: {e}")
                self._handle_connection_loss()

        return data_lines

    def cleanup(self):
        """Clean up serial connection and monitoring thread"""
        logger.info("Cleaning up Mega interface...")

        # Stop monitoring thread
        self._stop_monitoring()

        with self._lock:
            self.mega_connected = False

            if self.mega_serial:
                try:
                    self.mega_serial.close()
                    logger.info('Mega serial connection closed')
                except Exception as e:
                    logger.error(f'Error closing Mega serial: {str(e)}')
                finally:
                    self.mega_serial = None

            # Notify callbacks
            self._notify_connection_callbacks(False)

        logger.info("Mega interface cleanup completed")

    def get_connection_status(self) -> Dict[str, Any]:
        """Get detailed connection status"""
        with self._lock:
            return {
                'connected': self.mega_connected,
                'port': self.current_port,
                'auto_reconnect': self.auto_reconnect,
                'reconnect_attempts': self.reconnect_attempts,
                'last_activity': self.last_activity_time,
                'uptime': time.time() - self.stats['uptime_start'],
                'stats': self.stats.copy()
            }

    def force_reconnect(self) -> bool:
        """Force immediate reconnection attempt"""
        logger.info("Forcing Mega reconnection...")
        self.reconnect_attempts = 0  # Reset attempt counter
        return self.connect_to_mega()

    def is_healthy(self) -> bool:
        """Check if connection is healthy"""
        with self._lock:
            if not self.mega_connected or not self.mega_serial:
                return False

            # Check if we've had recent activity
            if time.time() - self.last_activity_time > 60.0:
                return False

            # Try a quick health check
            try:
                # Check if we can still access the port
                return self.mega_serial.is_open
            except:
                return False
