#include "Arduino_BMI270_BMM150.h"

/*
Name: Task 2 Arduino
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis
*/

float Theta_Final = 0;
void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {
    float ax, ay, az;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ay, ax, az); //Axis for X and Y are flipped

        Theta_Final = atan(ax/az) * 180 / 3.14159; // Calculate the angle in degrees

        // Send x, y, z values as comma-separated
        Serial.print(Theta_Final);
        Serial.print('\n');
        //Serial.print(',');
        //Serial.print(y, 2);
        //Serial.print(',');
        //Serial.println(z, 2);
    }
}
