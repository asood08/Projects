#include <Wire.h>

#define BMI270_ADDRESS    0x68  // I2C address for BMI270

// Register addresses for configuration and data reading
#define BMI270_REG_PWR_CTRL  0x7C
#define BMI270_REG_ACC_CONF  0x40
#define BMI270_REG_GYR_CONF  0x42
#define BMI270_REG_ACC_DATA  0x12
#define BMI270_REG_GYR_DATA  0x22

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C communication

  // Initialize the BMI270: power on and set ODR for acc and gyro to 1600 Hz
  initBMI270();
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Read sensor data for accelerometer and gyroscope
  readIMUData(ax, ay, az, gx, gy, gz);
  
  // Print accelerometer data
  Serial.print("Acc: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);
  
  // Print gyroscope data
  Serial.print("Gyro: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
  
  delay(1);  // Approximately 1 ms delay for 1600 Hz sampling rate
}

// Initializes the BMI270 sensor for operation at 1600 Hz ODR
void initBMI270() {
  // Power on the accelerometer and gyroscope
  writeRegister(BMI270_REG_PWR_CTRL, 0x01); // 0x01 typically turns on both sensors

  // Set the accelerometer ODR to 1600 Hz
  writeRegister(BMI270_REG_ACC_CONF, 0x20); // 0x20 sets ODR to 1600 Hz
  
  // Set the gyroscope ODR to 1600 Hz
  writeRegister(BMI270_REG_GYR_CONF, 0x20); // 0x20 sets ODR to 1600 Hz
}

// Reads accelerometer and gyroscope data from the BMI270
void readIMUData(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  // Read accelerometer data (6 bytes: X, Y, Z, each 2 bytes)
  Wire.beginTransmission(BMI270_ADDRESS);
  Wire.write(BMI270_REG_ACC_DATA);
  Wire.endTransmission();
  Wire.requestFrom(BMI270_ADDRESS, 6);
  
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope data (6 bytes: X, Y, Z, each 2 bytes)
  Wire.beginTransmission(BMI270_ADDRESS);
  Wire.write(BMI270_REG_GYR_DATA);
  Wire.endTransmission();
  Wire.requestFrom(BMI270_ADDRESS, 6);
  
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// Writes a single byte to a specified register on the BMI270
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI270_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
