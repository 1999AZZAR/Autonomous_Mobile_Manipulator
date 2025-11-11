# Sharp GP2Y0A02YK0F IR Distance Sensor Wiring Guide

## Overview

The robot uses 6x Sharp GP2Y0A02YK0F analog infrared distance sensors for wall alignment and obstacle detection. These sensors output analog voltage that must be converted to digital using an ADC (Analog-to-Digital Converter).

## Sensor Specifications

- Model: Sharp GP2Y0A02YK0F
- Type: Analog IR Distance Sensor
- Range: 20-150 cm (200-1500 mm)
- Output: Analog voltage 0.4V-2.7V
- Supply Voltage: 5V DC
- Current Consumption: 33 mA (typical)
- Response Time: 38±10 ms

## Voltage-Distance Relationship

The sensor output voltage is inversely proportional to distance:
- At 20 cm: ~2.7V
- At 150 cm: ~0.4V

Formula used: Distance (cm) = 60 × (Voltage ^ -1.1) - 1

## ADC Configuration

### MCP3008 ADC

The MCP3008 is a 10-bit, 8-channel ADC connected via SPI interface.

Specifications:
- Resolution: 10-bit (1024 levels)
- Reference Voltage: 3.3V
- SPI Interface: SPI0 on Raspberry Pi
- Maximum Sample Rate: 200 ksps

### SPI Wiring

Raspberry Pi to MCP3008:
```
RPi Pin         MCP3008 Pin     Function
GPIO9  (21)  →  DOUT (12)       MISO (Master In Slave Out)
GPIO10 (19)  →  DIN  (11)       MOSI (Master Out Slave In)
GPIO11 (23)  →  CLK  (13)       SCLK (Serial Clock)
GPIO8  (24)  →  CS   (10)       CE0  (Chip Enable 0)
3.3V         →  VREF (15)       Reference Voltage
3.3V         →  VDD  (16)       Power
GND          →  AGND (14)       Analog Ground
GND          →  DGND (9)        Digital Ground
```

### Sensor Channel Mapping

| MCP3008 Channel | Sensor Location | Robot Position |
|-----------------|-----------------|----------------|
| CH0             | Left Front      | Front-left side |
| CH1             | Left Back       | Rear-left side |
| CH2             | Right Front     | Front-right side |
| CH3             | Right Back      | Rear-right side |
| CH4             | Back Left       | Rear-left corner |
| CH5             | Back Right      | Rear-right corner |

## Sensor Wiring

Each Sharp GP2Y0A02YK0F sensor has 3 wires:

```
Sharp GP2Y0A02YK0F Pinout:
1. Red    - Vcc (5V)
2. Black  - GND
3. Yellow - Analog Output (Vout)
```

### Connection Diagram

```
                        +5V Power Supply
                            |
                    +-------+-------+
                    |       |       |
                   R1      R2   ... R6  (Red - Vcc)
                    |       |       |
                [Sensor1][Sensor2]...[Sensor6]
                    |       |       |
                   B1      B2   ... B6  (Black - GND)
                    |       |       |
                    +-------+-------+
                            |
                           GND
                            
       Yellow wires (Vout) connect to:
       
       Sensor1 → MCP3008 CH0
       Sensor2 → MCP3008 CH1
       Sensor3 → MCP3008 CH2
       Sensor4 → MCP3008 CH3
       Sensor5 → MCP3008 CH4
       Sensor6 → MCP3008 CH5
```

## Important Notes

1. The sensors require 5V power supply, but output voltage must NOT exceed 3.3V
2. Since the sensor output range is 0.4V-2.7V, it's safe to connect directly to MCP3008 (3.3V max)
3. Enable SPI interface on Raspberry Pi: `sudo raspi-config` → Interface Options → SPI → Enable
4. Install required Python library: `pip install spidev`
5. Sensor readings are averaged (5 samples) and median-filtered for stability
6. Response time is ~38ms, so reading all 6 sensors takes approximately 250ms

## Calibration

The conversion formula is based on typical Sharp GP2Y0A02YK0F characteristics. For better accuracy:

1. Measure actual voltages at known distances
2. Plot voltage vs 1/distance
3. Adjust the formula constants if needed:
   - Current: `distance_cm = 60 * (voltage ** -1.1) - 1`
   - Adjust `60` and `-1.1` based on your measurements

## Testing

Test individual sensor reading:
```python
import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

# Test channel 0
while True:
    value = read_adc(0)
    voltage = (value * 3.3) / 1024
    print(f"ADC: {value}, Voltage: {voltage:.2f}V")
    time.sleep(0.5)
```

## Troubleshooting

1. No readings: Check SPI is enabled, wiring is correct
2. Inconsistent readings: Check power supply is stable 5V
3. Out of range values: Ensure sensor is within 20-150cm from target
4. Noisy readings: Add 10µF capacitor between Vcc and GND near each sensor

