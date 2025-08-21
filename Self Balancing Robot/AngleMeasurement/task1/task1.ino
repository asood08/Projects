#include "Arduino_BMI270_BMM150.h"

/*
Name: Task 1 Arduino
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis
*/

float x, y, z;

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
        IMU.readGyroscope(x, y, z);

        // Send x, y, z values as comma-separated
        Serial.print(x, 2);
        Serial.print(',');
        Serial.print(y, 2);
        Serial.print(',');
        Serial.println(z, 2);
    }
}
