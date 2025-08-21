#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include "mbed.h"

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
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0;

float k2 = 0.99;
int start = 0; //if start == 1 (START!!)


//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;
float t0, t1, dt;

//IMU Variables
float gxbias,gybias,gzbias;
float accel_magnitude;
float threshold = 0.04;

// Kalman filter variables
float P[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix
float K[2];  // Kalman gain
float S;  // Innovation (or residual) covariance
float P00_temp, P01_temp, P10_temp, P11_temp;  // Temporary variables for calculations
float Q = 0.001; // Process noise covariance (gyro uncertainty)
float R = 0.1;   
double y;

void setup() {
  initializeAll();
}

void loop() {

  if(Serial.available())
  {
    String input = Serial.readString();

    threshold = input.substring(0, input.indexOf(' ')).toDouble();
    Theta_Final = 0;

    // start = input.substring(0, input.indexOf(' ')).toDouble();
    // input = input.substring(input.indexOf(' ') + 1);
    // kp = input.substring(0, input.indexOf(' ')).toDouble();
    // input = input.substring(input.indexOf(' ') + 1);
    // ki = input.substring(0, input.indexOf(' ')).toDouble();
    // input = input.substring(input.indexOf(' ') + 1);
    // kd = input.substring(0, input.indexOf(' ')).toDouble();
    // input = input.substring(input.indexOf(' ') + 1);
    // resetIntegral = input.substring(0, input.indexOf(' ')).toInt();

  }
  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();

}

void initializeAll() {
  Serial.begin(115200);
  Wire.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);

  pwmA0.write(0);
  pwmA1.write(0);
  pwmA2.write(0);
  pwmA3.write(0);

  calibrateIMU();
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

    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
    Theta_Gyro = gy_0 * 0.01 + Theta_Final;
    if(start == 1)
    {
      Theta_Final = (Theta_Gyro);
      PID();
    }
    else
    {
      accel_magnitude = abs(sqrt(pow(ax_0,2)+pow(az_0,2)) - 1);
      if(accel_magnitude > threshold)
      {
        k2 = 1;
      }
      else
      {
        k2 = 0.995;
      }
      Theta_Final = (Theta_Gyro)*k2 + Theta_Acc * (1 - k2);
      //kalmanFilter();
      SerialPrintFunctions();
    }
  }
}

void SerialPrintFunctions()
{
  Serial.print(10);
  Serial.print(" ");
  Serial.print(Theta_Final);
  Serial.print(" ");
  Serial.print(ax_0);
  Serial.print(" ");
  Serial.print(ay_0);
  Serial.print(" ");
  Serial.print(az_0);
  Serial.print(" ");
  Serial.print(accel_magnitude);
  Serial.print(" ");
  Serial.println(-10);
}

void PID()
{
    //Kp, Ki, and Kd are choosen for radians not for degrees.
    //et_new is in radians not degrees
    
    et_new = (desired_angle - Theta_Final);
    //et_new = (desired_angle - Theta_Final); //degrees
    kp_et = kp*et_new;

    et_integral += et_new*(float)dt;
    ki_et = ki*et_integral;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    //PID_OUTPUT = (kp_et + ki_et + kd_et)/(MAX_KP + MAX_KI + MAX_KD); // normalizing it to be between 0 and 1
    PID_OUTPUT = (kp_et + ki_et + kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -100.0, 100.0);
    PID_OUTPUT /= 100.0;

    Serial.print(Theta_Final);
    Serial.print(" ");
    Serial.print("kp_et: ");
    Serial.print(kp_et);
    Serial.print(" ki_et: ");
    Serial.print(ki_et);
    Serial.print(" kd_et: ");
    Serial.print(kd_et);
    Serial.print(" ");
    Serial.println(PID_OUTPUT);

    if(PID_OUTPUT >= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
    }
    else if(PID_OUTPUT <= 0.00)
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
    pwmA2.write(LEFT_MOTOR_PWM_SPEED);
    pwmA3.write(0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(0);
    pwmA3.write(LEFT_MOTOR_PWM_SPEED);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(RIGHT_MOTOR_PWM_SPEED);
    pwmA1.write(0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(0);
    pwmA1.write(RIGHT_MOTOR_PWM_SPEED);
  }
}

void kalmanFilter()
{
  // Kalman Filter Update (combining the accelerometer and gyroscope angles)
    // Prediction Step
    P00_temp = P[0][0] + dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q);
    P01_temp = P[0][1] - dt * P[1][1];
    P10_temp = P[1][0] - dt * P[1][1];
    P11_temp = P[1][1] + Q;

    // Measurement Update
    S = P00_temp + R;  // Innovation (or residual) covariance
    K[0] = P00_temp / S;  // Kalman gain for the angle
    K[1] = P10_temp / S;  // Kalman gain for the rate

    // Compute the new angle estimate
    y = Theta_Acc - Theta_Gyro;  // Residual (difference between the accelerometer and gyroscope)
    Theta_Gyro += K[0] * y;
    
    // Update error covariance matrix
    P[0][0] = P00_temp - K[0] * P00_temp;
    P[0][1] = P01_temp - K[0] * P01_temp;
    P[1][0] = P10_temp - K[1] * P00_temp;
    P[1][1] = P11_temp - K[1] * P01_temp;

    // The final angle estimate after the Kalman filter update
    Theta_Final = Theta_Gyro;

}