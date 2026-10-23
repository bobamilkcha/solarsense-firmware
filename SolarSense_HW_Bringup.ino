#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_LSM303_U.h>
#include <WiFi.h>
#include "secrets.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);

  if (!sht31.begin(0x44)) Serial.println("SHT31 not found");
  if (!accel.begin()) Serial.println("LSM303 accel not found");
  if (!mag.begin()) Serial.println("LSM303 mag not found");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nConnected, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\nWiFi failed");
  }
}

void loop() {
  float t, h;
  if (!sht31.readBoth(&t, &h)) { t = NAN; h = NAN; }

  sensors_event_t a, m;
  accel.getEvent(&a);
  mag.getEvent(&m);

  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x,
                sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float heading = atan2(m.magnetic.y, m.magnetic.x) * 180.0 / PI;
  if (heading < 0) heading += 360;

  Serial.printf("T=%.1fC  RH=%.1f%%  roll=%.1f  pitch=%.1f  heading=%.0f\n",
                t, h, roll, pitch, heading);
  delay(1000);
}

