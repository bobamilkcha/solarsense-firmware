#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_LSM303_U.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "secrets.h"

#define FW_VERSION "0.4.1"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

WiFiClientSecure net;
PubSubClient mqtt(net);
Preferences prefs;

// ---------- Hardware ----------
const int ADC_PIN = 32;
const float SHUNT_OHMS = 10.0;      // local Serial display only; backend does the real maths
const float ISC_STC_MA = 220.0;     // local Serial display only
const float ADC_FLOOR_MV = 142.0;   // ESP32 reading at 0 V (measured)

// ---------- Remote-configurable ----------
const uint32_t INTERVAL_MIN_S = 5;
const uint32_t INTERVAL_MAX_S = 3600;
uint32_t publishIntervalS = 60;     // default; overridden by saved value

// ---------- Reliability ----------
const uint32_t WDT_TIMEOUT_S      = 60;                 // reboot if loop freezes this long
const unsigned long WIFI_RETRY_MS   = 30UL * 1000;      // WiFi reconnect attempt interval
const unsigned long WIFI_RESTART_MS = 15UL * 60 * 1000; // restart if WiFi down this long
const unsigned long MQTT_RESTART_MS = 5UL * 60 * 1000;  // restart if MQTT down this long (WiFi up)

// ---------- Offline buffer ----------
const char* BUF_PATH = "/buffer.txt";
const size_t BUF_MAX_BYTES = 1000000;   // ~1 MB ≈ 3+ days at 60 s
const int FLUSH_BATCH = 10;             // buffered readings sent per loop
bool fsOK = false;
uint32_t flushPos = 0;

bool shtOK = false, accelOK = false, magOK = false;
char deviceId[24];
char readingsTopic[64], configTopic[64], statusTopic[64];
unsigned long lastPublish = 0;
volatile bool statusPending = false;
uint32_t bootCount = 0;
int resetReason = 0;

// ---------- Helpers ----------
void fmt(char* out, size_t n, float v, int decimals) {
  if (isnan(v)) snprintf(out, n, "null");
  else snprintf(out, n, "%.*f", decimals, v);
}

long nowEpoch() {
  time_t now = time(nullptr);
  return (now > 1700000000) ? (long)now : 0;
}

// ---------- Status ----------
void publishStatus() {
  long e = nowEpoch();
  char ts[16];
  if (e) snprintf(ts, sizeof(ts), "%ld", e);
  else snprintf(ts, sizeof(ts), "null");

  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"device\":\"%s\",\"online\":true,\"fw\":\"%s\",\"publish_interval_s\":%u,"
    "\"boot_count\":%u,\"reset_reason\":%d,\"ts\":%s}",
    deviceId, FW_VERSION, publishIntervalS, bootCount, resetReason, ts);
  mqtt.publish(statusTopic, payload, true);
  Serial.printf("Status: %s\n", payload);
}

// ---------- Incoming config ----------
void onMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, configTopic) != 0) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("Config parse error: %s\n", err.c_str());
    return;
  }

  if (doc["publish_interval_s"].is<uint32_t>()) {
    uint32_t v = doc["publish_interval_s"];
    if (v >= INTERVAL_MIN_S && v <= INTERVAL_MAX_S) {
      if (v != publishIntervalS) {
        publishIntervalS = v;
        prefs.putUInt("interval", v);
        Serial.printf("Config applied: publish_interval_s=%u\n", v);
      }
    } else {
      Serial.printf("Rejected publish_interval_s=%u (allowed %u-%u)\n", v, INTERVAL_MIN_S, INTERVAL_MAX_S);
    }
  }
  statusPending = true;   // publish from loop(), not inside the callback
}

// ---------- Offline buffer ----------
void bufferReading(const char* payload, long epoch) {
  if (!fsOK) return;
  if (!epoch) {   // without a real timestamp the backend would mis-date it
    Serial.println("Not buffered: no valid timestamp yet");
    return;
  }
  File f = LittleFS.open(BUF_PATH, "a");
  if (!f) return;
  if (f.size() > BUF_MAX_BYTES) {
    f.close();
    Serial.println("Buffer full, reading dropped");
    return;
  }
  f.println(payload);
  f.close();
  Serial.println("Offline: reading buffered");
}

void flushBuffer() {
  if (!fsOK || !LittleFS.exists(BUF_PATH)) return;
  File f = LittleFS.open(BUF_PATH, "r");
  if (!f) return;

  if (flushPos >= f.size()) {   // everything sent
    f.close();
    LittleFS.remove(BUF_PATH);
    flushPos = 0;
    prefs.putUInt("flushpos", 0);
    Serial.println("Buffer fully flushed");
    return;
  }

  f.seek(flushPos);
  int sent = 0;
  while (f.available() && sent < FLUSH_BATCH) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (!mqtt.publish(readingsTopic, line.c_str())) break;
      sent++;
    }
    flushPos = f.position();
  }
  f.close();

  if (sent) {
    prefs.putUInt("flushpos", flushPos);   // survives a reboot mid-flush
    Serial.printf("Flushed %d buffered readings\n", sent);
  }
}

// ---------- Connectivity ----------
void connectMQTT() {
  if (!nowEpoch()) return;   // wait for NTP: needed for cert check + timestamps
  static unsigned long lastAttempt = 0;
  if (mqtt.connected() || millis() - lastAttempt < 5000) return;
  lastAttempt = millis();
  Serial.print("MQTT connecting... ");

  char will[96];
  snprintf(will, sizeof(will), "{\"device\":\"%s\",\"online\":false}", deviceId);

  if (mqtt.connect(deviceId, MQTT_USER, MQTT_PASS, statusTopic, 1, true, will)) {
    Serial.println("connected");
    mqtt.subscribe(configTopic, 1);
    publishStatus();
  } else {
    char err[128];
    net.lastError(err, sizeof(err));
    Serial.printf("failed, state=%d, TLS: %s\n", mqtt.state(), err);
    net.stop();
  }
}

void maintainConnectivity() {
  static unsigned long wifiDownSince = 0, mqttDownSince = 0, lastWifiRetry = 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiDownSince) wifiDownSince = millis();
    if (millis() - lastWifiRetry > WIFI_RETRY_MS) {
      lastWifiRetry = millis();
      Serial.println("WiFi down, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    if (millis() - wifiDownSince > WIFI_RESTART_MS) {
      Serial.println("WiFi down too long, restarting");
      delay(100);
      ESP.restart();
    }
    return;
  }
  wifiDownSince = 0;

  connectMQTT();
  mqtt.loop();

  if (!mqtt.connected()) {
    if (!mqttDownSince) mqttDownSince = millis();
    if (millis() - mqttDownSince > MQTT_RESTART_MS) {
      Serial.println("MQTT down too long, restarting");
      delay(100);
      ESP.restart();
    }
  } else {
    mqttDownSince = 0;
  }
}

void setupWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   // watch the loop task
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  resetReason = (int)esp_reset_reason();
  prefs.begin("solarsense", false);
  publishIntervalS = prefs.getUInt("interval", publishIntervalS);
  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);

  fsOK = LittleFS.begin(true);   // format on first use
  flushPos = prefs.getUInt("flushpos", 0);
  if (fsOK && !LittleFS.exists(BUF_PATH)) flushPos = 0;
  Serial.printf("Boot #%u, reset reason %d, FS %s\n", bootCount, resetReason, fsOK ? "OK" : "FAILED");

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

  uint64_t mac = ESP.getEfuseMac();
  snprintf(deviceId, sizeof(deviceId), "solarsense-%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  snprintf(readingsTopic, sizeof(readingsTopic), "solarsense/%s/readings", deviceId);
  snprintf(configTopic,   sizeof(configTopic),   "solarsense/%s/config",   deviceId);
  snprintf(statusTopic,   sizeof(statusTopic),   "solarsense/%s/status",   deviceId);
  Serial.printf("Device ID: %s | FW %s | interval %us\n", deviceId, FW_VERSION, publishIntervalS);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // steady current draw; avoids power bank auto-off
  WiFi.setAutoReconnect(true);
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
    Serial.println("\nWiFi failed (will keep retrying)");
  }

  configTime(0, 0, "pool.ntp.org", "time.google.com");

  net.setCACert(MQTT_CA_CERT);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setCallback(onMessage);

  setupWatchdog();
}

// ---------- Loop ----------
void loop() {
  esp_task_wdt_reset();
  maintainConnectivity();

  if (statusPending && mqtt.connected()) {
    statusPending = false;
    publishStatus();
  }
  if (mqtt.connected()) flushBuffer();

  float t = NAN, h = NAN;
  float roll = NAN, pitch = NAN, heading = NAN;

  static int shtFails = 0;
  if (shtOK) {
    t = sht31.readTemperature();
    h = sht31.readHumidity();
    if (isnan(t) || isnan(h)) {
      if (++shtFails % 10 == 0) {          // every 10 failed reads (~10 s)
        Serial.println("SHT31 failing, resetting sensor");
        sht31.reset();
      }
    } else {
      shtFails = 0;
    }
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

  if (millis() - lastPublish >= publishIntervalS * 1000UL) {
    lastPublish = millis();

    long e = nowEpoch();
    char ts[16];
    if (e) snprintf(ts, sizeof(ts), "%ld", e);
    else snprintf(ts, sizeof(ts), "null");

    char sT[12], sH[12], sR[12], sP[12], sHd[12];
    fmt(sT, sizeof(sT), t, 2);
    fmt(sH, sizeof(sH), h, 2);
    fmt(sR, sizeof(sR), roll, 1);
    fmt(sP, sizeof(sP), pitch, 1);
    fmt(sHd, sizeof(sHd), heading, 0);

    char payload[320];
    snprintf(payload, sizeof(payload),
      "{\"device\":\"%s\",\"ts\":%s,\"v_mv\":%.0f,"
      "\"temp\":%s,\"rh\":%s,\"roll\":%s,\"pitch\":%s,\"heading\":%s,\"rssi\":%d}",
      deviceId, ts, mv, sT, sH, sR, sP, sHd, WiFi.RSSI());

    if (!e) {
      Serial.println("Skipped: clock not synced yet (NTP)");
    } else if (mqtt.connected() && mqtt.publish(readingsTopic, payload)) {
      Serial.printf("Published: %s\n", payload);
    } else {
      bufferReading(payload, e);
    }
  }

  delay(1000);
}
