"""
GPIO Controller for servos, motors, and actuators
Handles direct hardware control when not using Arduino Mega
"""

import time
import logging
from config import ADC_VREF, ADC_RESOLUTION

logger = logging.getLogger(__name__)

class GPIOController:
    """Direct GPIO control for servos, motors, and actuators"""

    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        self.gpio_initialized = False
        self.spi = None
        self.adc_vref = ADC_VREF
        self.adc_resolution = ADC_RESOLUTION

        if not self.simulation_mode:
            self._initialize_gpio()
        else:
            logger.info("GPIO Controller running in SIMULATION mode")

    def _initialize_gpio(self):
        """Initialize GPIO libraries"""
        try:
            # Try Raspberry Pi GPIO first
            import lgpio
            self.lgpio_available = True
            logger.info("Using lgpio library")
        except ImportError:
            self.lgpio_available = False
            logger.warning("lgpio not available")

        try:
            self.gpiozero_available = True
            logger.info("gpiozero available as fallback")
        except ImportError:
            self.gpiozero_available = False
            logger.warning("gpiozero not available")

        if self.lgpio_available:
            try:
                # Initialize lgpio
                self.gpio_chip = lgpio.gpiochip_open(0)
                self.gpio_initialized = True
                logger.info("GPIO initialized with lgpio")
            except Exception as e:
                logger.error(f"Failed to initialize lgpio: {str(e)}")
                if self.gpiozero_available:
                    logger.info("Trying gpiozero as fallback...")
                    self._try_gpiozero_fallback()
                else:
                    logger.error("No GPIO libraries available - running in SIMULATION mode")
                    self.simulation_mode = True
        elif self.gpiozero_available:
            self._try_gpiozero_fallback()
        else:
            logger.error("No GPIO libraries available - running in SIMULATION mode")
            self.simulation_mode = True

    def _try_gpiozero_fallback(self):
        """Fallback to gpiozero if lgpio fails"""
        if not self.gpiozero_available:
            logger.error("gpiozero not available for fallback")
            self.simulation_mode = True
            return

        try:
            # Initialize basic gpiozero components for testing
            self.gpio_initialized = True
            logger.info("GPIO initialized with gpiozero (fallback)")
        except Exception as e:
            logger.error(f"gpiozero fallback failed: {str(e)}")
            logger.info("Falling back to SIMULATION mode")
            self.simulation_mode = True
            self.gpio_initialized = False

    def read_adc_channel(self, channel):
        """Read ADC channel value"""
        if self.simulation_mode:
            # Return simulated ADC value
            return int(ADC_RESOLUTION * 0.5)  # Mid-range value

        try:
            # Hardware ADC reading would go here
            # This is a placeholder for actual ADC implementation
            logger.debug(f"Reading ADC channel {channel}")
            return int(ADC_RESOLUTION * 0.5)  # Placeholder
        except Exception as e:
            logger.error(f"ADC read error on channel {channel}: {str(e)}")
            raise

    def adc_to_voltage(self, adc_value):
        """Convert ADC value to voltage"""
        return (adc_value * self.adc_vref) / self.adc_resolution

    def sharp_gp2y0a02_voltage_to_distance(self, voltage):
        """Convert voltage to distance in mm for Sharp GP2Y0A02YK0F sensor"""
        if voltage < 0.4 or voltage > 2.7:
            return None  # Out of range

        # Polynomial approximation for GP2Y0A02YK0F
        # Distance in mm = f(voltage)
        if voltage > 1.5:
            distance = 2000 / (voltage + 0.5)  # Closer range
        else:
            distance = 10000 / (voltage + 1.0)  # Farther range

        return max(200, min(1500, distance))  # Clamp to sensor range

    def read_sharp_sensor(self, channel, sensor_name=None):
        """Read a Sharp GP2Y0A02YK0F sensor and return distance in mm"""
        try:
            # Take multiple readings and average for stability
            readings = []
            for _ in range(5):
                adc_value = self.read_adc_channel(channel)
                voltage = self.adc_to_voltage(adc_value)
                distance = self.sharp_gp2y0a02_voltage_to_distance(voltage)
                if distance is not None:
                    readings.append(distance)
                time.sleep(0.001)

            if readings:
                # Return median to filter out noise
                readings.sort()
                distance = readings[len(readings) // 2]
                return distance
            else:
                return None

        except Exception as e:
            logger.error(f"Error reading Sharp sensor on channel {channel}: {str(e)}")
            return None

    def set_gripper_base(self, height):
        """Set gripper base position (-1 to 1, continuous servo)"""
        if self.simulation_mode:
            logger.info(f"[SIM] Gripper base: {height}")
            return True

        # Note: Mega may not support gripper base control yet
        # This functionality might need to be added to the Mega firmware
        logger.warning(f"[NOT IMPLEMENTED] Gripper base control not available on Mega: {height}")
        return False

    def home_servos(self):
        """Home all servos to default positions"""
        if self.simulation_mode:
            logger.info("[SIM] Homing servos")
            return True

        logger.warning("[NOT IMPLEMENTED] Servo homing not available on Mega")
        return False

    def move_robot(self, direction, speed=0.5):
        """Move robot in specified direction"""
        if self.simulation_mode:
            logger.info(f"[SIM] Moving robot {direction} at speed {speed}")
            return True

        logger.warning(f"[NOT IMPLEMENTED] Robot movement not available on GPIO: {direction}")
        return False

    def turn_robot(self, direction, speed=0.5):
        """Turn robot in specified direction"""
        if self.simulation_mode:
            logger.info(f"[SIM] Turning robot {direction} at speed {speed}")
            return True

        logger.warning(f"[NOT IMPLEMENTED] Robot turning not available on GPIO: {direction}")
        return False

    def stop_robot(self):
        """Stop robot movement"""
        if self.simulation_mode:
            logger.info("[SIM] Stopping robot")
            return True

        logger.warning("[NOT IMPLEMENTED] Robot stopping not available on GPIO")
        return False

    def control_container(self, container_id, action):
        """Control container mechanism"""
        if self.simulation_mode:
            logger.info(f"[SIM] Container {container_id}: {action}")
            return True

        logger.warning(f"[NOT IMPLEMENTED] Container control not available on GPIO: {container_id} {action}")
        return False

    def cleanup(self):
        """Clean up GPIO resources"""
        if hasattr(self, 'gpio_chip') and self.gpio_chip:
            try:
                import lgpio
                lgpio.gpiochip_close(self.gpio_chip)
                logger.info('GPIO cleaned up')
            except Exception as e:
                logger.error(f'Error cleaning up GPIO: {str(e)}')
