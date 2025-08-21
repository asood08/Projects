#define SERIAL_BAUD 115200

void setup() {
    Serial.begin(SERIAL_BAUD);
}

void loop() {
    for (int angle = 0; angle < 360; angle += 10) {  // Scan in 10-degree increments
        float distance = 100 + 50 * (angle % 90 == 0); // Simulated square shape
        Serial.print(angle);
        Serial.print(",");
        Serial.println(distance); // Print angle,distance format
        delay(100);  // Simulate scan time
    }
}
