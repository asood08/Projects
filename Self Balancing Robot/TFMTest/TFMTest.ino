void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); // Change to 9600 if needed
}

void loop() {
    static uint8_t buf[9];
    if (Serial1.available() >= 9) {  // TF-Luna sends 9-byte frames
        for (int i = 0; i < 9; i++) {
            buf[i] = Serial1.read();
        }

        // Validate frame header
        if (buf[0] == 0x59 && buf[1] == 0x59) {
            int distance = buf[2] | (buf[3] << 8);  // Combine low and high bytes
            int strength = buf[4] | (buf[5] << 8);
            int temperature = (buf[6] | (buf[7] << 8)) / 8 - 256; // Temp formula
            
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print(" cm, Strength: ");
            Serial.print(strength);
            Serial.print(", Temp: ");
            Serial.print(temperature);
            Serial.println("°C");
        }
    }
}
