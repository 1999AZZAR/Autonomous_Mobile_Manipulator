// I2C Scanner for Arduino Mega
// Scans the I2C bus to detect connected devices
// Useful for verifying motor driver and servo connections

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  Serial.println("\nI2C Scanner for Arduino Mega");
  Serial.println("Scanning I2C bus for connected devices...\n");

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C speed

  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (");

      // Identify common devices
      switch (address) {
        case 0x40:
          Serial.print("YFROBOT Motor Driver");
          break;
        case 0x20:
          Serial.print("PCA9685 Servo Driver");
          break;
        default:
          Serial.print("Unknown device");
          break;
      }

      Serial.println(")");
      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  Serial.println("\nScan complete.");
  Serial.print("Found ");
  Serial.print(deviceCount);
  Serial.println(" device(s).\n");

  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Possible issues:");
    Serial.println("- Check SDA/SCL connections (D20/D21)");
    Serial.println("- Verify power connections (5V for motor driver and servos)");
    Serial.println("- Check for short circuits or loose connections");
    Serial.println("- Ensure pull-up resistors are present");
  } else {
    bool foundMotorDriver = false;
    bool foundServoDriver = false;

    // Quick check for expected devices
    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() == 0) foundMotorDriver = true;

    Wire.beginTransmission(0x20);
    if (Wire.endTransmission() == 0) foundServoDriver = true;

    Serial.println("Expected devices:");
    Serial.print("- YFROBOT Motor Driver (0x40): ");
    Serial.println(foundMotorDriver ? "✓ DETECTED" : "✗ NOT FOUND");
    Serial.print("- PCA9685 Servo Driver (0x20): ");
    Serial.println(foundServoDriver ? "✓ DETECTED" : "✗ NOT FOUND");
  }
}

void loop() {
  // Nothing to do in loop
  delay(1000);
}
