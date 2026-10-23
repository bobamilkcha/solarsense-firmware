#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C Scanner starting...");
}

void loop() {
  byte count = 0;
  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print(count);
    Serial.println(" device(s) found");
  }

  delay(3000);
}

