#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include "DHTesp.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------------- CONFIG ----------------
#define DHT_PIN 17 // DHT11 data pin

const char *WIFI_SSID = "NCW-Personal";
const char *WIFI_PASS = "Ncw5201314";

const char *MQTT_BROKER = "192.168.1.61"; // Raspberry Pi IP
const int MQTT_PORT = 1883;

const char *MQTT_TOPIC = "greenhouse/sensor/air_th"; // 修改不同 ESP32 topic
// ----------------------------------------

DHTesp dht;
Adafruit_SGP30 sgp;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Forward declaration
uint32_t getAbsoluteHumidity(float temperature, float humidity);
void reconnectMQTT();

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=== ESP32 DHT11 + SGP30 MQTT Test ===");

  // Initialize DHT11
  dht.setup(DHT_PIN, DHTesp::DHT11);
  Serial.printf("DHT11 initialized. Min sampling period: %d ms\n",
                dht.getMinimumSamplingPeriod());

  // Initialize I2C and SGP30
  // Wire.begin();
  // if (!sgp.begin()) {
  //   Serial.println("ERROR: SGP30 not found! Check wiring and pull-ups.");
  //   while (true) delay(10);
  // }
  // sgp.IAQinit();
  // Serial.println("SGP30 detected and initialized successfully!\n");

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected! IP: " + WiFi.localIP().toString());

  // Setup MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop()
{
  // Ensure MQTT connection
  if (!mqttClient.connected())
  {
    reconnectMQTT();
  }
  mqttClient.loop();

  static uint32_t last = 0;
  if (millis() - last > 10000) // every 10 seconds
  {
    last = millis();
    // Read DHT11
    TempAndHumidity values = dht.getTempAndHumidity();
    if (dht.getStatus() != 0)
    {
      Serial.println("DHT11 read error!");
      delay(3000);
      return;
    }

    // Feed SGP30 for compensation
    // uint32_t absoluteHumidity = getAbsoluteHumidity(values.temperature, values.humidity);
    // sgp.setHumidity(absoluteHumidity);

    // Read SGP30
    // if (!sgp.IAQmeasure()) {
    //   Serial.println("SGP30 measurement failed");
    //   delay(3000);
    //   return;
    // }

    // Prepare JSON
    JsonDocument doc;
    doc["temp"] = values.temperature;
    doc["humidity"] = values.humidity;
    // doc["eco2"] = sgp.eCO2;
    // doc["tvoc"] = sgp.TVOC;
    doc["rssi"] = WiFi.RSSI();

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);

    // Publish to MQTT
    if (mqttClient.publish(MQTT_TOPIC, jsonBuffer))
    {
      Serial.println("MQTT publish successful!");
    }
    else
    {
      Serial.println("MQTT publish failed!");
    }

    // Debug print
    Serial.println("JSON sent: " + String(jsonBuffer));
    Serial.println("-----------------------------------");

    delay(1000); // 1-second interval
  }
}

// ---------------- MQTT Reconnect ----------------
void reconnectMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect("Air_TH_Node"))
    {
      Serial.println(" connected!");
    }
    else
    {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(". Retrying in 2 seconds...");
      delay(2000);
    }
  }
}

// ---------------- Absolute Humidity ----------------
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  if (humidity < 0 || humidity > 100 || temperature < -40 || temperature > 85)
    return 0;
  const float absHum = 216.7f *
                       ((humidity / 100.0f) * 6.112f *
                        exp((17.67f * temperature) / (temperature + 243.5f)) /
                        (273.15f + temperature));
  return static_cast<uint32_t>(absHum * 1000); // mg/m³
}
