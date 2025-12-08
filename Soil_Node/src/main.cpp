#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------------- CONFIG ----------------
#define SOIL_PIN 32

const char *WIFI_SSID = "NCW-Personal";
const char *WIFI_PASS = "Ncw5201314";

const char *MQTT_BROKER = "192.168.1.61"; // Raspberry Pi IP
const int MQTT_PORT = 1883;

const char *MQTT_TOPIC = "greenhouse/sensor/soil";
// ----------------------------------------

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Forward declaration
void reconnectMQTT();

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=== ESP32 Soil Moisture Sensor MQTT ===");

  // Initialize ADC pin
  pinMode(SOIL_PIN, INPUT);

  // ---------------- Wi-Fi Connect ----------------
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected! IP: " + WiFi.localIP().toString());

  // ---------------- MQTT Setup ----------------
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

  // ---------------- Read Soil Sensor ----------------
  int rawValue = analogRead(SOIL_PIN);
  float percent = map(rawValue, 0, 4095, 100, 0);
  // map：越接近 0 = 很濕  → 100%
  //      越接近 4095 = 很乾 →   0%

  // ---------------- Create JSON ----------------
  JsonDocument doc;
  doc["soil_raw"] = rawValue;
  doc["soil_percent"] = percent;
  doc["rssi"] = WiFi.RSSI();

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  // ---------------- Publish to MQTT ----------------
  if (mqttClient.publish(MQTT_TOPIC, jsonBuffer))
  {
    Serial.println("MQTT publish successful!");
  }
  else
  {
    Serial.println("MQTT publish failed!");
  }

  Serial.println("JSON sent: " + String(jsonBuffer));
  Serial.println("-----------------------------------");

  delay(3000); // Read every 3 seconds
}

// ---------------- MQTT Reconnect ----------------
void reconnectMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect("Soil_Node"))
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
