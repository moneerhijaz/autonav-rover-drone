#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SCAN_PERIOD_MS = 3000;

void print_hex_address(uint8_t address)
{
  Serial.print("0x");
  if (address < 16) {
    Serial.print("0");
  }
  Serial.print(address, HEX);
}

void scan_i2c_bus()
{
  uint8_t device_count = 0;

  Serial.println();
  Serial.println("Scanning I2C bus on Teensy Wire pins SDA=18, SCL=19...");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at ");
      print_hex_address(address);
      Serial.println();
      device_count++;
    } else if (error == 4) {
      Serial.print("Unknown error at ");
      print_hex_address(address);
      Serial.println();
    }
  }

  if (device_count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Done. Device count: ");
    Serial.println(device_count);
  }
}

}  // namespace

void setup()
{
  Serial.begin(SERIAL_BAUD);

  const uint32_t start_ms = millis();
  while (!Serial && millis() - start_ms < 4000) {
    delay(10);
  }

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("I2C scanner ready.");
  scan_i2c_bus();
}

void loop()
{
  delay(SCAN_PERIOD_MS);
  scan_i2c_bus();
}
