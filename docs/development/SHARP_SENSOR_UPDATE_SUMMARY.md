# Sharp GP2Y0A02YK0F Sensor Integration Update

## Summary

Updated the robot interface to properly handle analog Sharp GP2Y0A02YK0F IR distance sensors instead of digital I2C sensors. The sensors now read analog voltage through an MCP3008 ADC via SPI interface.

## Changes Made

### 1. Hardware Interface Update

Changed from:
- Digital I2C sensors (VL53L0X)
- TCA9548A I2C multiplexer
- GPIO2/GPIO3 (I2C interface)

To:
- Analog IR sensors (Sharp GP2Y0A02YK0F)
- MCP3008 10-bit ADC
- SPI0 interface (GPIO8, 9, 10, 11)

### 2. Code Changes

File: `scripts/web_robot_interface.py`

Added imports:
```python
import spidev
import math
```

Added class initialization:
```python
# Initialize SPI for MCP3008 ADC
self.spi = spidev.SpiDev()
self.spi.open(0, 0)
self.spi.max_speed_hz = 1350000

# Sensor channel mapping
self.sensor_channels = {
    'left_front': 0,
    'left_back': 1,
    'right_front': 2,
    'right_back': 3,
    'back_left': 4,
    'back_right': 5
}

# ADC configuration
self.adc_vref = 3.3
self.adc_resolution = 1024
```

New methods added:
- `read_adc_channel(channel)` - Read raw ADC value from MCP3008
- `adc_to_voltage(adc_value)` - Convert ADC value to voltage
- `sharp_gp2y0a02_voltage_to_distance(voltage)` - Convert voltage to distance using empirical formula
- `read_sharp_sensor(channel)` - Read sensor with averaging and filtering
- `read_all_sensors()` - Read all sensors and return structured data

New API routes:
- `/api/robot/sensors` - Returns all sensor data in JSON format
- `/api/robot/status` - Returns system status

### 3. Conversion Formula

The Sharp GP2Y0A02YK0F has a non-linear relationship between voltage and distance:

```python
distance_cm = 60 * (voltage ** -1.1) - 1
```

This formula provides accurate readings in the 20-150cm range.

### 4. Sensor Reading Strategy

- Takes 5 samples per sensor
- Calculates median value to filter noise
- 1ms delay between samples
- Total reading time per sensor: ~5ms
- All 6 sensors: ~30ms

### 5. Documentation Updates

- Updated `notes.txt` with correct sensor specifications
- Created `SENSOR_WIRING.md` with detailed wiring guide
- Updated HTML interface labels and documentation
- Added SPI pinout diagram
- Added sensor channel mapping

### 6. Interface Updates

- Changed "Laser Distance Sensors (6x VL53L0X)" to "IR Distance Sensors (6x Sharp GP2Y0A02YK0F)"
- Updated wiring documentation from I2C to SPI
- Added sensor specifications in interface
- Updated hardware information panel

## Required Dependencies

```
spidev - Python SPI library for Raspberry Pi
```

Install:
```bash
pip install spidev
```

## System Configuration

Enable SPI interface on Raspberry Pi:
```bash
sudo raspi-config
# Navigate to: Interface Options → SPI → Enable
```

## Wiring Summary

### MCP3008 to Raspberry Pi (SPI0)
- DOUT → GPIO9 (Pin 21) MISO
- DIN → GPIO10 (Pin 19) MOSI
- CLK → GPIO11 (Pin 23) SCLK
- CS → GPIO8 (Pin 24) CE0
- VREF → 3.3V
- VDD → 3.3V
- AGND → GND
- DGND → GND

### Sharp Sensors to MCP3008
| Sensor Location | MCP3008 Channel | Sensor Vcc | Sensor Vout |
|----------------|-----------------|-----------|-------------|
| Left Front | CH0 | 5V | CH0 |
| Left Back | CH1 | 5V | CH1 |
| Right Front | CH2 | 5V | CH2 |
| Right Back | CH3 | 5V | CH3 |
| Back Left | CH4 | 5V | CH4 |
| Back Right | CH5 | 5V | CH5 |

All sensor GND pins connect to common ground.

## Testing

Start the web interface:
```bash
cd ros2_ws
ros2 run my_robot_automation web_robot_interface.py
```

Access the interface:
```
http://localhost:8000
```

Navigate to "Sensors" tab to view real-time sensor readings.

## Benefits of Analog Sensors

1. True analog distance measurement (not time-of-flight)
2. Fast response time (~38ms)
3. No I2C address conflicts
4. Simple integration via ADC
5. Cost-effective solution
6. Reliable in various lighting conditions
7. No complex multiplexer needed

## Limitations

1. Non-linear response curve requires conversion
2. Affected by surface reflectivity
3. Dead zone below 20cm
4. Limited range (150cm max)
5. 5V power required (but safe 0.4-2.7V output)

## Future Improvements

1. Calibration routine for each sensor
2. Temperature compensation
3. Surface reflectivity detection
4. Outlier rejection algorithm
5. Moving average filter option
6. Configurable sample count
7. Distance threshold alerts

## References

- Sharp GP2Y0A02YK0F Datasheet
- MCP3008 Datasheet (Microchip)
- Raspberry Pi SPI Documentation
- Robot sensor layout diagram

