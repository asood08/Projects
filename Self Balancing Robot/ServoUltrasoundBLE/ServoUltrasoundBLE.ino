#include <ArduinoBLE.h>

#define TRIG_PIN1 7
#define ECHO_PIN1 8
#define TRIG_PIN2 9
#define ECHO_PIN2 10

float lastValidDistance1 = 10.0; // Initial valid distance
float lastValidDistance2 = 10.0; // Initial valid distance
int t1 = 0;
float Theta_Final = 23;
int stepper_angle = 0;

//Bluetooth Declarations
#define BUFFER_SIZE 64

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEDevice laptopMaster;

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);

    if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
    }

    BLE.setLocalName("C5-BLE");
    BLE.setDeviceName("C5-BLE");
    BLE.setAdvertisedService(laptopMasterService);
    laptopMasterService.addCharacteristic(laptopMasterCharacteristic);
    BLE.addService(laptopMasterService);

    BLE.advertise();

    while (true) {
      laptopMaster = BLE.central();
      if (laptopMaster) {
        Serial.println("Connected to MASTER LAPTOP");
        break;
      }
    }
    delay(1000); //Wait 1 second after initialization
}

// Function to get valid distance
float getValidDistance(int trigPin, int echoPin, float &lastValidDistance) {
    long duration;
    float distance;

    // Send a 10µs pulse to trigger the sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the time for the echo to return
    duration = pulseIn(echoPin, HIGH);

    // Convert time to distance (in cm)
    distance = (duration * 0.0343) / 2;

    // Check if distance is valid
    if (distance > 200 || distance == 0) {
        return lastValidDistance; // Use previous valid value
    } else {
        lastValidDistance = distance; // Update valid value
        return distance;
    }
}

void loop() {
    // float distance1 = getValidDistance(TRIG_PIN1, ECHO_PIN1, lastValidDistance1);
    // float distance2 = getValidDistance(TRIG_PIN2, ECHO_PIN2, lastValidDistance2);

    // Serial.print("Sensor 1 Distance: ");
    // Serial.print(distance1);
    // Serial.print(" cm | Sensor 2 Distance: ");
    // Serial.print(distance2);
    // Serial.println(" cm");

    laptopMasterCharacteristic.writeValue(String(Theta_Final, 2));

    delay(200);
}
