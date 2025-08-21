#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include "mbed.h"
#include <Wire.h>

//MAGNETIC ENCODERS DEFINITIONS
#define TCA9548A_ADDR 0x70  // Default address of the TCA9548A
#define AS5600_ADDR   0x36  // Fixed address of AS5600

//WHEEL MOTOR PINS
#define RIGHT_MOTOR_FORWARD_PIN  A0
#define RIGHT_MOTOR_BACKWARD_PIN  A1
#define LEFT_MOTOR_FORWARD_PIN  A2
#define LEFT_MOTOR_BACKWARD_PIN  A3

// Define PWM objects for the analog pins
mbed::PwmOut pwmA0(digitalPinToPinName(A2));
mbed::PwmOut pwmA1(digitalPinToPinName(A3));
mbed::PwmOut pwmA2(digitalPinToPinName(A0));
mbed::PwmOut pwmA3(digitalPinToPinName(A1));

// Gyroscope Values
float Theta_Gyro = 0;
// Accelerometer Values
float Theta_Acc = 0;
// Filtered Values
float Theta_Final = 0;
float Theta_Test = 0;
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0;
float Theta_Offset = 0; 

float k2 = 0.95;
int start = 0; //if start == 1 (START!!)

//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 2.95; //Proportional
float ki = 32.8; //Integral
float kd = 0.1545; //Derivative
float desired_angle = 1.45; //1. //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;
float t0, t1, dt, t2, t3, t4;
float ANGLE_OFFSET = 0; //If robot goes towards blue arrow (Increase positively). If robot goes backward (opposite of blue arrow) then decrease towards negative.

//ENCODER PID Variables
float encoder_et_old, encoder_et_new, encoder_kp_et, encoder_ki_et, encoder_kd_et, encoder_et_integral, encoder_et_derivatve;
float encoder_kp = 0;
float encoder_ki = 0;
float encoder_kd = 0;
float encoder_desired = 0; //0m from reference
float encoder_pid_output = 0;
uint16_t leftMotorAngleOld = 0;
uint16_t leftMotorAngleNew = 0;
uint16_t rightMotorAngleOld = 0;
uint16_t rightMotorAngleNew = 0;
float deltaLeftMotor = 0;
float deltaRightMotor = 0;
float wheel_radius = 0.04;
//REFERENCE VARIABLES FOR ENCODER
float distance_from_reference = 0.0; 

float LEFT_FORWARD_OFFSET = 0.054;
float LEFT_BACKWARD_OFFSET = 0.051;
float RIGHT_FORWARD_OFFSET = 0.056;
float RIGHT_BACKWARD_OFFSET = 0.051;

float MASTER_GAIN = 1.0;

//Bluetooth Declarations
#define BUFFER_SIZE 64

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterPIDOUTPUTCharacteristic("00000003-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKpOutputCharacteristic("00000004-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKiOutputCharacteristic("00000005-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKdOutputCharacteristic("00000006-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

BLEDevice laptopMaster;
int Laptop2RobotLength = 0;
int receiveLength = 0;

//IMU Variables
float gxbias,gybias,gzbias;
float accel_magnitude;
float threshold = 0.04;

void setup() {
  initializeAll();
}

void loop() {

  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();
  receiveBLE();
  
}

void initializeAll() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  calibrateIMU();

  Theta_Final = 0;

  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);

  pwmA0.write(1.0);
  pwmA1.write(1.0);
  pwmA2.write(1.0);
  pwmA3.write(1.0);

  Wire.begin();

  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  BLE.setLocalName("C5-BLE");
  BLE.setDeviceName("C5-BLE");
  BLE.setAdvertisedService(laptopMasterService);
  laptopMasterService.addCharacteristic(laptopMasterCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterReceiveCharacteristic);
  BLE.addService(laptopMasterService);

  BLE.advertise();

  while (true) {
    laptopMaster = BLE.central();
    if (laptopMaster) {
      Serial.println("Connected to MASTER LAPTOP");
      break;
    }
  }
}

void calibrateIMU()
{ 
  int iterations = 0;
  while(iterations < 1000)
  {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      IMU.readGyroscope(gy_0, gx_0, gz_0);
      IMU.readAcceleration(ay_0, ax_0, az_0);

      gxbias += gx_0;
      gybias += gy_0;
      gzbias += gz_0;
      iterations++;
    }
  }

  gxbias /= 1000;
  gybias /= 1000;
  gzbias /= 1000;
}


void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    gx_0 -= gxbias;
    gy_0 -= gybias;
    gz_0 -= gzbias;
    
    t1 = micros();
    dt = (t1 - t0) / 1000000.0;
    t0 = t1;

    ax_0 += 0.041;

    if(abs(ax_0) >= 0 && abs(ax_0) <= 0.01)
    {
      ax_0 = 0;
    }
    if(abs(ay_0) >= 0 && abs(ay_0) <= 0.01)
    {
      ay_0 = 0;
    }
    if(abs(az_0) >= 1)
    {
      //az_0 should never exceed positive 1g technically
      az_0 = 1.0;
    }

    Theta_Gyro = -1.0*gy_0 *dt + Theta_Final;
    Theta_Acc = atan(ax_0 /az_0 )* 180 / 3.14159;
    // Theta_Acc = atan(sqrt(1-pow(az_0,2))/az_0)*180/3.14159;
    // Theta_Acc = ax_0 > 0 ? Theta_Acc : -1.0*Theta_Acc;

    //accel_magnitude = sqrt(pow(ax_0, 2) + pow(ay_0, 2) + pow(az_0, 2));

    Theta_Final = (1-k2)*Theta_Acc + k2*Theta_Gyro;

    if(start == 1)
    {
      PID();
    }
  }
}

void SerialPrintFunctions()
{
  Serial.print(-25);
  Serial.print(" ");
  Serial.print(Theta_Final);
  Serial.print(" ");
  Serial.println(25);
}

void PID()
{
    //Kp, Ki, and Kd are choosen for radians not for degrees.
    //et_new is in radians not degrees
    
    et_new = (desired_angle - Theta_Final );
    //et_new = (desired_angle - Theta_Final); //degrees
    kp_et = kp*et_new;

    et_integral += et_new*(float)dt;
    ki_et = ki*et_integral;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    //DriftPID();

    PID_OUTPUT = (kp_et + ki_et + kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -100.0, 100.0);
    PID_OUTPUT /= 100.0;

    // Serial.print(Theta_Final);
    // Serial.print(" ");
    // Serial.print("kp_et: ");
    // Serial.print(kp_et);
    // Serial.print(" ki_et: ");
    // Serial.print(ki_et);
    // Serial.print(" kd_et: ");
    // Serial.print(kd_et);
    // Serial.print(" ");
    // Serial.println(PID_OUTPUT);

    if(abs(Theta_Final) >= 13)
    {
      //Avoid crashing if angle exceeds 13 degrees
      pwmA0.write(1.0);
      pwmA1.write(1.0);
      pwmA2.write(1.0);
      pwmA3.write(1.0);
    }
    else if(PID_OUTPUT <= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
    }
    else if(PID_OUTPUT >= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //Theta < 0, PID_OUT will be positive (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 1, abs(PID_OUTPUT), 1);
    }

    et_old = et_new;
}

/************************************************************************************************************
 LEFT_MOTOR_PWM_SPEED (0,255) :
 LEFT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Left Motor and 1 = Backward Direction of Left Motor
 RIGHT_MOTOR_PWM_SPEED (0,255) : 
 RIGHT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Right Motor and 1 = Backward Direction of Right Motor
*************************************************************************************************************/

void controlWheelMotors(float LEFT_MOTOR_PWM_SPEED, float LEFT_MOTOR_DIR, float RIGHT_MOTOR_PWM_SPEED, float RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    pwmA2.write(1.0);
    pwmA3.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_FORWARD_OFFSET));
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_BACKWARD_OFFSET));
    pwmA3.write(1.0);

  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(1.0);
    pwmA1.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_FORWARD_OFFSET));
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_BACKWARD_OFFSET));
    pwmA1.write(1.0);
  }
}

void receiveBLE()
{
  if(laptopMaster.connected())
  {
    if (laptopMasterReceiveCharacteristic.written()) {

      receiveLength = laptopMasterReceiveCharacteristic.valueLength();
      byte receiveBuffer[receiveLength];
      if (laptopMasterReceiveCharacteristic.readValue(receiveBuffer, receiveLength)) {

        String receiveString = String((char*)receiveBuffer);

        //ANGLE_OFFSET = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        kp = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        ki = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        kd = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        resetIntegral = receiveString.substring(3, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_FORWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_BACKWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_FORWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_BACKWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();

        start = 1;

        // Serial.print("Kp: ");
        // Serial.print(kp, 3);
        // Serial.print(" Ki: ");
        // Serial.print(ki, 3);
        // Serial.print(" Kd: ");
        // Serial.print(kd, 3);
        // Serial.print(" LMF: ");
        // Serial.print(LEFT_FORWARD_OFFSET);
        // Serial.print(" LMB: ");
        // Serial.print(LEFT_BACKWARD_OFFSET);
        // Serial.print(" RMF: ");
        // Serial.print(RIGHT_FORWARD_OFFSET);
        // Serial.print(" RMB: ");
        // Serial.println(RIGHT_BACKWARD_OFFSET);
        Serial.println(MASTER_GAIN);

        /*MAX_KP = kp*(MAX_TILT);
        MAX_KI = ki*pow((MAX_TILT),2)/2.0;
        MAX_KD = kd*(1.0/0.01); //1 degree in 10 ms is the assumed max change in error we expect.*/
      }
    }
  }
  laptopMasterCharacteristic.writeValue(String(Theta_Final, 2) + " " + String(PID_OUTPUT, 2) + " " + String(kp_et/100.0, 2) + " " + String(ki_et/100.0, 2) + " " + String(kd_et/100.0, 2));
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

void DriftPID()
{
  TCA_Select(2);
  uint16_t rightMotorAngle = readAS5600();
  TCA_Select(7);
  uint16_t leftMotorAngle = readAS5600();
  distance_from_reference += (leftMotorAngle + rightMotorAngle)/2.0;
}