#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include "mbed.h"
#include <Wire.h>

//MAGNETIC ENCODER DECLARATIONS
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
float yaw = 0;

float k2 = 0.95; //for robot tilt angle
float k3 = 0.95; //for robot rotation angle
int start = 0; //if start == 1 (START!!)

//PID Variables
float MASTER_GAIN = 1.0;
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 1.7*MASTER_GAIN; //Proportional (1.7)
float ki = 16*MASTER_GAIN; //Integral (16)
float kd = 0.085*MASTER_GAIN; //Derivative (0.085)
float kp_reference = kp;
float ki_reference = ki;
float kd_reference = kd;
float desired_angle = 1.45; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float desired_angle_reference = desired_angle;
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;
float t0, t1, dt, t2, t3, t4, t5, t6;
float ANGLE_OFFSET = 0; //If robot goes towards blue arrow (increase positvely). If robot goes backward (opposite of blue arrow) then decrease towards negative.

float LEFT_FORWARD_OFFSET = 0.054;
float LEFT_BACKWARD_OFFSET = 0.051;
float RIGHT_FORWARD_OFFSET = 0.056;
float RIGHT_BACKWARD_OFFSET = 0.051;


//ROBOT MOVING VARIABLES
int outputCase = 0;
float moveForwardOffset = 0;
float moveBackwardOffset = 0;
float moveLeftOffset = 0;
float moveRightOffset = 0;
int referenceCase = 0;
int timeWait1 = 0;
int timeWait2 = 0;
float turningRatio = 0.6;

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

float lw_distance, rw_distance;
float robot_base = 0.245; //distance from motor to motor
float wheel_radius = 0.04;
float wheel_circumference = 2*3.14159*wheel_radius;
uint16_t oldLeftAngle,newLeftAngle, oldRightAngle,newRightAngle;
float dLeft,dRight;
float robotRotationAngle = 0;


void setup() {
  initializeAll();
}

void loop() {

  if(resetIntegral == 1 && abs(Theta_Final) <= 5 && millis() - t3 > 3000) //You have to hold robot at upright position for 3 seconds before balancing occurs again
  {
    et_integral = 0;
    resetIntegral = 0;
  }
  //getRobotRotationData();
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
    //et_new is in degrees
    
    et_new = (desired_angle - Theta_Final);
    //et_new = (desired_angle - Theta_Final); //degrees
    kp_et = kp*et_new;

    et_integral += (et_new)*(float)dt;
    ki_et = ki*et_integral;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    //PID_OUTPUT = (kp_et + ki_et + kd_et)/(MAX_KP + MAX_KI + MAX_KD); // normalizing it to be between 0 and 1
    PID_OUTPUT = (kp_et + ki_et + kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -100.0, 100.0);
    PID_OUTPUT /= 100.0;

    switch(outputCase) 
    {
      case 0:
        //Balancing
        // kp = kp_reference;
        // ki = ki_reference;
        // kd = kd_reference;
        desired_angle = desired_angle_reference;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        balanceRobot();
        break;
      case 1:
        //Forward
        desired_angle = desired_angle_reference + 0.6;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        balanceRobot();
        break;
      case 2:
        //Backward
        desired_angle = desired_angle_reference - 0.5;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        balanceRobot();
        break;
      case 3:
        //Right
        // controlWheelMotors(0, 0, PID_OUTPUT + 0.02, 0);
        // et_integral = 0;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        break;
      case 4:
        //Left
        // controlWheelMotors(PID_OUTPUT + 0.02, 0, 0, 0);
        // et_integral = 0;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        break;
      default:
        kp = kp_reference;
        ki = ki_reference;
        kd = kd_reference;
        desired_angle = desired_angle_reference;
        moveForwardOffset = 0;
        moveBackwardOffset = 0;
        moveLeftOffset = 0;
        moveRightOffset = 0;
        break;
    }

    if(millis() - t4 <= timeWait1 && (referenceCase != 3 && referenceCase != 4))
    {
      outputCase = 0; //balances
    }
    else if(millis() - t4 <= timeWait2)
    {
      outputCase = referenceCase; //go forward or backward or turn
    }
    else
    {
      t4 = millis();
    }


    switch (referenceCase) {
      case 3: // Turning right
          if (PID_OUTPUT <= 0) {
              controlWheelMotors(abs(PID_OUTPUT) * (1 + turningRatio) + 0.05, 0, abs(PID_OUTPUT) * (1 - turningRatio) - 0.05, 0);
          } else {
              controlWheelMotors(abs(PID_OUTPUT) * (1 - turningRatio) - 0.05, 1, abs(PID_OUTPUT) * (1 + turningRatio) + 0.05, 1);
          }
          break;

      case 4: // Turning left
          if (PID_OUTPUT <= 0) {
              controlWheelMotors(abs(PID_OUTPUT) * (1 - turningRatio) - 0.05, 0, abs(PID_OUTPUT) * (1 + turningRatio) + 0.05, 0);
          } else {
              controlWheelMotors(abs(PID_OUTPUT) * (1 + turningRatio) + 0.05, 1, abs(PID_OUTPUT) * (1 - turningRatio) - 0.05, 1);
          }
          break;

      default:
          // Optional: Handle other cases if needed
          break;
    }


    if(abs(Theta_Final) >= 25)
    {
      //Avoid crashing if angle exceeds 13 degrees
      pwmA0.write(1.0);
      pwmA1.write(1.0);
      pwmA2.write(1.0);
      pwmA3.write(1.0);
      resetIntegral = 1;

      t3 = millis();
    }

    et_old = et_new;
}

void balanceRobot()
{
  if(PID_OUTPUT <= 0.00 && resetIntegral == 0)
  {
    //PID_OUTPUT /= 1000.0;
    //PID_OUT will be negative (mostly)
    controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
  }
  else if(PID_OUTPUT >= 0.00 && resetIntegral == 0)
  {
    //PID_OUTPUT /= 1000.0;
    //Theta < 0, PID_OUT will be positive (mostly)
    controlWheelMotors(abs(PID_OUTPUT), 1, abs(PID_OUTPUT), 1);
  } 
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
    pwmA3.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_FORWARD_OFFSET + moveForwardOffset));
  }
  else
  {
    pwmA2.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_BACKWARD_OFFSET + moveBackwardOffset));
    pwmA3.write(1.0);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(1.0);
    pwmA1.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_FORWARD_OFFSET + moveForwardOffset));
  }
  else
  {
    pwmA0.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_BACKWARD_OFFSET + moveBackwardOffset));
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

        //outputCase = receiveString.substring(0, receiveString.indexOf(' ')).toInt();   
        referenceCase =  receiveString.substring(0, receiveString.indexOf(' ')).toInt();   
        t4 = millis();  
        start = 1;
        switch(referenceCase)
        {
          case 0:
            timeWait1 = 300;
            timeWait2 = 800;
            kp = kp_reference;
            ki = ki_reference;
            kd = kd_reference;
            break;
          case 1:
            timeWait1 = 300;
            timeWait2 = 800;
            kp = kp_reference;
            ki = ki_reference;
            kd = kd_reference;
            break;
          case 2:
            timeWait1 = 100;
            timeWait2 = 800;
            kp = kp_reference;
            ki = ki_reference;
            kd = kd_reference;
            break;
          case 3:
            timeWait1 = 400;
            timeWait2 = 500;
            break;
          case 4:
            timeWait1 = 400;
            timeWait2 = 500;
            break;
          default:
            timeWait1 = 300;
            timeWait2 = 800;
            break;
        }

        //Serial.println(outputCase);
      }
    }
  }
  laptopMasterCharacteristic.writeValue(String(Theta_Final, 2));
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

void getRobotRotationData()
{
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
