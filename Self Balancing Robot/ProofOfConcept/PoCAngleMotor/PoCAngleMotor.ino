#include <mbed.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h> 

#define LEFT_MOTOR_FORWARD_PIN  A0
#define LEFT_MOTOR_BACKWARD_PIN  A1
#define RIGHT_MOTOR_FORWARD_PIN  A2
#define RIGHT_MOTOR_BACKWARD_PIN  A3

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

Thread readIMU;
Thread moveMotors;

//Gyroscope Values
float Theta_Gyro = 0;

//Accelerometer Values
float Theta_Acc = 0;

//Weighted Values
float Theta_Final = 0;
float k = 0.6; //0.6 before

float gx_0, gy_0, gz_0, ax_0,ay_0,az_0, x_0, x;

int maxRPM = 467;
float RPMRequired = 0;
int DCycle = 0;
int DCycleLeft = 0;
int DCycleRight = 0;

/***************************************************************************************************
  positiveAngle from (0 Degrees to 90 Degrees) = 0% MAX RPM to 100% MAX RPM IN FORWARD DIRECTION
  negativeAngle from (0 Degrees to -90 Degrees) = 0% MAX RPM to 100% MAX RPM IN BACKWARD DIRECTION
***************************************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
  }

  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);

  readIMU.start(readIMUFunction);
  moveMotors.start(moveMotorsFunction);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Angle: ");
  Serial.print(Theta_Final);
  Serial.print(" DC_L(%): ");
  Serial.print(DCycleLeft);
  Serial.print(" DC_R(%): ");
  Serial.println(DCycleRight);
}

void readIMUFunction()
{
  while(true)
  {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

      IMU.readGyroscope(gy_0, gx_0, gz_0);
      IMU.readAcceleration(ay_0,ax_0,az_0);

      Theta_Acc = atan(ax_0/az_0) * 180/3.14159;
      Theta_Gyro = gy_0*0.01 + Theta_Final;

      Theta_Final = (Theta_Gyro)*k + Theta_Acc*(1-k);
    }
  }
}

/*
Theta_Final is between 0 to 45 and 0 to -45 degrees from 0 to max rpm
*/

void moveMotorsFunction()
{
  while(true)
  {
    if(Theta_Final < 3 && Theta_Final > -3)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      DCycleLeft = 0;
      DCycleRight = 0;
  
    }
    else if(Theta_Final > 0 && Theta_Final <= 45)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, (calcMotorSpeed((Theta_Final/45.0)*100.0)/100.0)*255.0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      DCycleLeft = DCycle;
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, (calcMotorSpeed((Theta_Final/45.0)*100.0)/100.0)*255.0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      DCycleRight = DCycle;
    }
    else if(Theta_Final < 0 && Theta_Final >= -45)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, (calcMotorSpeed((-Theta_Final/45.0)*100.0)/100.0)*255.0);
      DCycleLeft = DCycle;
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, (calcMotorSpeed((-Theta_Final/45.0)*100.0)/100.0)*255.0);
      DCycleRight = DCycle;
    }
    else
           {
      //If angle is past the +- 45 degrees then just keep the current speed (should be max RPM technically)
    }
  }
}

//Returns Duty Cycle needed for % of maxRPM
int calcMotorSpeed(float angleInput)
{
  RPMRequired = (angleInput/100.0)*maxRPM;
  DCycle = 5.34 - 0.0167*RPMRequired + 1.47*pow(10,-3)*pow(RPMRequired,2) - 6.82*pow(10,-6)*pow(RPMRequired,3) + 1.01*pow(10,-8)*pow(RPMRequired,4);
  if(DCycle > 100)
  {
    DCycle = 100;
  }
  return DCycle;
}