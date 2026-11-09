#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_LSM303_U.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include "secrets.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

WiFiClientSecure net;
PubSubClient mqtt(net);

const int ADC_PIN = 32;
const float SHUNT_OHMS = 10.0;
const float ISC_STC_MA = 220.0;     // estimated from Imp=200mA; replace after calibration
const float ADC_FLOOR_MV = 142.0;   // ESP32 reading at 0 V (measured)
const unsigned long PUBLISH_INTERVAL_MS = 10000;

bool shtOK = false, accelOK = false, magOK = false;
char deviceId[24];
char topic[64];
unsigned long lastPublish = 0;

// Write a float into a buffer, or "null" if NaN (NaN is not valid JSON)
void fmt(char* out, size_t n, float v, int decimals) {
  if (isnan(v)) snprintf(out, n, "null");
  else snprintf(out, n, "%.*f", decimals, v);
}

void connectMQTT() {
  static unsigned long lastAttempt = 0;
  if (mqtt.connected() || millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  Serial.print("MQTT connecting... ");
  if (mqtt.connect(deviceId, MQTT_USER, MQTT_PASS)) {
    Serial.println("connected");
  } else {
    Serial.printf("failed, state=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  shtOK = sht31.begin(0x44);
  if (!shtOK) shtOK = sht31.begin(0x45);
  accelOK = accel.begin();
  magOK   = mag.begin();
  Serial.printf("SHT31: %s | Accel: %s | Mag: %s\n",
                shtOK ? "OK" : "NOT FOUND",
                accelOK ? "OK" : "NOT FOUND",
                magOK ? "OK" : "NOT FOUND");

  // Unique device ID from the ESP32's MAC
  uint64_t mac = ESP.getEfuseMac();
  snprintf(deviceId, sizeof(deviceId), "solarsense-%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  snprintf(topic, sizeof(topic), "solarsense/%s/readings", deviceId);
  Serial.printf("Device ID: %s\nTopic: %s\n", deviceId, topic);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected, IP: %s, RSSI: %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("\nWiFi failed");
  }

  // NTP time so each reading carries a real timestamp (UTC epoch)
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  net.setInsecure();   // TESTING ONLY: skips certificate check (see note below)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);   // default 256 is too small for our JSON
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
    mqtt.loop();
  }

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
    roll  = (roll < 0) ? roll + 180.0 : roll - 180.0;   // module mounted upside down
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
  if (mv <= ADC_FLOOR_MV + 5) mv = 0;
  float ma = mv / SHUNT_OHMS;
  float irr = (ma / ISC_STC_MA) * 1000.0;

  if (mv > 2450) Serial.println("WARN: near ADC ceiling, reading may be clipped");

  Serial.printf("V=%.0fmV I=%.1fmA G=%.0fW/m2 | T=%.1fC RH=%.1f%% roll=%.1f pitch=%.1f heading=%.0f\n",
                mv, ma, irr, t, h, roll, pitch, heading);

  // Publish every PUBLISH_INTERVAL_MS
  if (mqtt.connected() && millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = millis();

    time_t now = time(nullptr);
    char ts[16];
    if (now > 1700000000) snprintf(ts, sizeof(ts), "%ld", (long)now);
    else snprintf(ts, sizeof(ts), "null");   // NTP not synced yet

    char sT[12], sH[12], sR[12], sP[12], sHd[12];
    fmt(sT, sizeof(sT), t, 2);
    fmt(sH, sizeof(sH), h, 2);
    fmt(sR, sizeof(sR), roll, 1);
    fmt(sP, sizeof(sP), pitch, 1);
    fmt(sHd, sizeof(sHd), heading, 0);

    char payload[384];
        snprintf(payload, sizeof(payload),
      "{\"device\":\"%s\",\"ts\":%s,\"v_mv\":%.0f,"
      "\"temp\":%s,\"rh\":%s,\"roll\":%s,\"pitch\":%s,\"heading\":%s,\"rssi\":%d}",
      deviceId, ts, mv, sT, sH, sR, sP, sHd, WiFi.RSSI());

    if (mqtt.publish(topic, payload)) Serial.printf("Published: %s\n", payload);
    else Serial.println("Publish failed");
  }

  delay(1000);
}