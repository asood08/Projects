#include "Arduino_BMI270_BMM150.h"

/*
Name: Task 3 Arduino
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis
*/

float Theta_Gyro = 0;

float gx_0, gy_0, gz_0;

void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gy_0, gx_0, gz_0);

        // Send x, y, z values as comma-separated
        // Serial.print(x_0, 2);
        /*Serial.print(',');
        Serial.print(y_0, 2);
        Serial.print(',');
        Serial.println(z_0, 2);*/

        Theta_Gyro = gy_0*0.01 + Theta_Gyro;
        

        Serial.print(Theta_Gyro);
        Serial.print('\n');
    }
}


