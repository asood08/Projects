/*
  HC-SR04 NewPing Library Demonstration
  HC-SR04-NewPing.ino
  Demonstrates functions of NewPing Library for HC-SR04 Ultrasonic Range Finder
  Displays results on Serial Monitor

  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

// This uses Serial Monitor to display Range Finder distance readings

// Include NewPing Library
#include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin 10, Echo to Arduino pin 13
// Maximum Distance is 400 cm

#define TRIGGER_PIN_1  9
#define ECHO_PIN_1     10
#define TRIGGER_PIN_2  8
#define ECHO_PIN_2     7
#define MAX_DISTANCE 400
 
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
//NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

float distance1,distance2;

void setup() {
  Serial.begin (9600);
}

void loop() {
   
  //distance = sonar1.ping_cm();
  distance1 = sonar1.ping_cm();
  //distance2 = sonar2.ping_cm();
  
  // Send results to Serial Monitor
  Serial.println(distance1);
  // Serial.print(" ");
  // Serial.println(distance2);
}