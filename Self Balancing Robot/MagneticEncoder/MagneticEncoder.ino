#include <Wire.h>

#define TCA9548A_ADDR 0x70  // Default address of the TCA9548A
#define AS5600_ADDR   0x36  // Fixed address of AS5600

void TCA_Select(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // Enable the specific channel (0 or 1)
  Wire.endTransmission();
}
uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // Register for raw angle (MSB)
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes (MSB + LSB)
  
  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;  // Combine MSB & LSB
  }
  
  return 0; // Return 0 if no data received
}
void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  //Select Channel 2 (AS5600 Encoder 1)
  TCA_Select(2);
  uint16_t angle1 = readAS5600();
  Serial.print("Encoder 1 Angle: ");
  Serial.print(angle1 * 0.08789); // Convert to degrees (360/4096)
  Serial.print(" ");
  
  // Select Channel 7 (AS5600 Encoder 2)
  TCA_Select(7);
  uint16_t angle2 = readAS5600();
  Serial.print("Encoder 2 Angle: ");
  Serial.println(angle2 * 0.08789); // Convert to degrees
  // Select Channel 5 (AS5600 Encoder 3)
  
  // TCA_Select(5);
  // uint16_t angle3 = readAS5600();
  // Serial.print("Encoder 2 Angle: ");
  // Serial.println(angle3 * 0.08789); // Convert to degrees

}