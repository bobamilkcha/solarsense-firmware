#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_LSM303_U.h>
#include <WiFi.h>
#include "secrets.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

const int ADC_PIN = 32;
const float SHUNT_OHMS = 10.0;
const float ISC_STC_MA = 210.0;     // estimated from Imp=200mA; replace after calibration
const float ADC_FLOOR_MV = 142.0;   // ESP32 reading at 0 V (measured)

bool shtOK = false, accelOK = false, magOK = false;

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  Nothing found");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());

  Wire.begin(21, 22);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  scanI2C();

  shtOK = sht31.begin(0x44);
  if (!shtOK) shtOK = sht31.begin(0x45);
  accelOK = accel.begin();
  magOK   = mag.begin();
  Serial.printf("SHT31: %s | Accel: %s | Mag: %s\n",
                shtOK ? "OK" : "NOT FOUND",
                accelOK ? "OK" : "NOT FOUND",
                magOK ? "OK" : "NOT FOUND");

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
  float t = NAN, h = NAN;
  float roll = NAN, pitch = NAN, heading = NAN;

  if (shtOK) {
    t = sht31.readTemperature();
    h = sht31.readHumidity();
  }

  if (accelOK) {
    sensors_event_t a;
    accel.getEvent(&a);
    roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    pitch = atan2(-a.acceleration.x,
            sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  }

  if (magOK) {
    sensors_event_t m;
    mag.getEvent(&m);
    heading = atan2(m.magnetic.y, m.magnetic.x) * 180.0 / PI;
    if (heading < 0) heading += 360;
  }

  long sum = 0;
  for (int i = 0; i < 64; i++) { sum += analogReadMilliVolts(ADC_PIN); delay(2); }
  float mv = sum / 64.0;
  if (mv <= ADC_FLOOR_MV + 5) mv = 0;          // below ADC dead zone → treat as zero
  float ma = mv / SHUNT_OHMS;
  float irr = (ma / ISC_STC_MA) * 1000.0;      // irradiance in W/m²

  if (mv > 2450) Serial.println("WARN: near ADC ceiling, reading may be clipped");

  Serial.printf("V=%.0fmV I=%.1fmA G=%.0fW/m2 | T=%.1fC RH=%.1f%% roll=%.1f pitch=%.1f heading=%.0f\n",
                mv, ma, irr, t, h, roll, pitch, heading);
  delay(1000);
}
