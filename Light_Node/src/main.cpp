#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------------- CONFIG ----------------
const char *WIFI_SSID = "NCW-Personal";
const char *WIFI_PASS = "Ncw5201314";

const char *MQTT_BROKER = "192.168.1.61"; // Raspberry Pi IP
const int MQTT_PORT = 1883;

const char *MQTT_TOPIC = "greenhouse/sensor/light";
// ----------------------------------------

BH1750 lightMeter;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Forward declaration
void reconnectMQTT();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

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
  if (millis() - last > 10000) // Every 10 seconds
  {
    last = millis();
    float lux = lightMeter.readLightLevel();

    JsonDocument doc;
    doc["lux"] = roundf(lux * 10) / 10.0;
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
    if (mqttClient.connect("Light_Node"))
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