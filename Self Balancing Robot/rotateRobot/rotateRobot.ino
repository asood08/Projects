/*
Left Wheel Turn Forward = Positive Angle Change
Right Wheel Turn Forward = Negative Angle Change
*/

#include <Wire.h>

#define TCA9548A_ADDR 0x70  // Default address of the TCA9548A
#define AS5600_ADDR   0x36  // Fixed address of AS5600

float lw_distance, rw_distance;
float robot_base = 0.245; //distance from motor to motor
float wheel_radius = 0.04;
float wheel_circumference = 2*3.14159*wheel_radius;
uint16_t oldLeftAngle,newLeftAngle, oldRightAngle,newRightAngle;
float dLeft,dRight;
float robotRotationAngle = 0;
float correctedRobotRotationAngle = 0;
float testAngleSumLeft, testAngleSumRight;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  // Select Channel 0 (AS5600 Encoder 1)
  TCA_Select(2);
  newRightAngle = readAS5600()*0.08789;
  // Select Channel 1 (AS5600 Encoder 2)
  TCA_Select(7);
  newLeftAngle = readAS5600()*0.08789;
  dLeft = getDeltaAngle(newLeftAngle, oldLeftAngle);
  dRight = -1.0*getDeltaAngle(newRightAngle, oldRightAngle);

  robotRotationAngle += (dRight/360.0 - dLeft/360.0)*(wheel_circumference/robot_base)*(180.0/3.14159);
  
  if(robotRotationAngle > 360.0)
  {
    robotRotationAngle = robotRotationAngle - 360.0;
  }
  else if(robotRotationAngle < 0)
  {
    robotRotationAngle = robotRotationAngle + 360.0;
  }
  Serial.println(robotRotationAngle);

  oldRightAngle = newRightAngle;
  oldLeftAngle = newLeftAngle;


}

float getDeltaAngle(float new_angle, float old_angle)
{
  float delta = new_angle - old_angle;

    // Handle wrapping cases
    if (delta > 180) {
        delta -= 360;  // Example: 350 - 10 = 340 → Corrected to -20
    } else if (delta < -180) {
        delta += 360;  // Example: 10 - 350 = -340 → Corrected to +20
    }

    return delta;
}

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
