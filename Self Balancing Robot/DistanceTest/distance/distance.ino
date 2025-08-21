#include "Arduino_BMI270_BMM150.h"

/*
Name: Task 4 Arduino
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis
*/

//Gyroscope Values
float Theta_Gyro = 0;

//Accelerometer Values
float Theta_Acc = 0;

//Weighted Values
float Theta_Final = 0;
float k = 0.6;
float dt = 0.01;
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0, vx, vy, vz, dx, dy, dz, ax_new, ax_old, vx_0;
float grav_const = 9.80665;
float acc_const_tuning = 0.04;
float a_x_prime = 0;
float m2cm = 100;

float ax_alpha = 0.8;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
}

void loop() {

  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    Theta_Acc = atan(ax_0 / az_0) * 180.0 / 3.14159;
    Theta_Gyro = gy_0 * 0.01 + Theta_Final;

    Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);

    a_x_prime = sqrt(pow(ax_0, 2) + pow(az_0, 2));

    ax_new = grav_const * cos(Theta_Final * 3.14159 / 180.0) * (a_x_prime - 1);

    if ((ax_0 > 0 && ax_new > 0) || (ax_0 < 0 && ax_new < 0)) {
      ax_new *= -1;
    }

    if (ax_new < 0.03 && ax_new > -0.03) {
      ax_new = 0;
    }

    Serial.print(vx, 2);
    Serial.println("\t");
  }
}
