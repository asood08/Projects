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
float k = 0.6; //0.6 before

float gx_0, gy_0, gz_0, ax_0,ay_0,az_0, x_0, x;

void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {

    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

        IMU.readGyroscope(gy_0, gx_0, gz_0);
        IMU.readAcceleration(ay_0,ax_0,az_0);

        Theta_Acc = atan(ax_0/az_0) * 180/3.14159;
        Theta_Gyro = gy_0*0.01 + Theta_Final;

        Theta_Final = (Theta_Gyro)*k + Theta_Acc*(1-k);

        Serial.print(Theta_Final, 2);
        Serial.print(',');
        Serial.print(Theta_Acc, 2);
        Serial.print(',');
        Serial.println(Theta_Gyro, 2);
    }
}
