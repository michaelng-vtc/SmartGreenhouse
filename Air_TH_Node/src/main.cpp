#include <Arduino.h>
#include <Wire.h>
#include "DHTesp.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// ---------------- CONFIG ----------------
#define DHT_PIN 18 // DHT11 data pin
#define AIR_TX_PIN 17 
#define AIR_RX_PIN 16 

const char *WIFI_SSID = "NCW-Personal";
const char *WIFI_PASS = "Ncw5201314";

const char *MQTT_BROKER = "192.168.1.61"; // Raspberry Pi IP
const int MQTT_PORT = 1883;

const char *MQTT_TOPIC = "greenhouse/sensor/air_th"; // 修改不同 ESP32 topic
// ----------------------------------------

DHTesp dht;
HardwareSerial airSensorSerial(2);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Air sensor variables
const int packetSize = 9;
float latestTVOC = 0.0;
float latestCH2O = 0.0;
uint16_t latestCO2 = 0;

// Forward declaration
void reconnectMQTT();
bool readAirSensorPacket();

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=== ESP32 DHT11 + SGP30 MQTT Test ===");

  // Initialize DHT11
  dht.setup(DHT_PIN, DHTesp::DHT11);
  Serial.printf("DHT11 initialized. Min sampling period: %d ms\n",
                dht.getMinimumSamplingPeriod());
  
  // Initialize Air Quality Sensor Serial
  airSensorSerial.begin(9600, SERIAL_8N1, AIR_RX_PIN, AIR_TX_PIN);
  Serial.println("CO2/VOC/CH2O Sensor started");

  // Connect to Wi-Fi
  // WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Serial.print("Connecting to Wi-Fi");
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nWi-Fi connected! IP: " + WiFi.localIP().toString());

  // Setup MQTT
  // mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop()
{
  // Ensure MQTT connection
  // if (!mqttClient.connected())
  // {
  //   reconnectMQTT();
  // }
  // mqttClient.loop();

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

    // Read air sensor data
    readAirSensorPacket();

    // Prepare JSON
    JsonDocument doc;
    doc["temp"] = values.temperature;
    doc["humidity"] = values.humidity;
    doc["tvoc"] = latestTVOC;
    doc["ch2o"] = latestCH2O;
    doc["co2"] = latestCO2;
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

// ---------------- Air Sensor Packet Reading ----------------
bool readAirSensorPacket()
{
  static uint8_t packet[9];
  static uint8_t index = 0;
  static bool headerFound = false;

  while (airSensorSerial.available())
  {
    uint8_t byte = airSensorSerial.read();

    if (!headerFound)
    {
      if (byte == 0x2C)
      {
        packet[0] = byte;
        if (airSensorSerial.available())
        {
          byte = airSensorSerial.read();
          if (byte == 0xE4)
          {
            packet[1] = byte;
            index = 2;
            headerFound = true;
          }
        }
      }
      continue;
    }

    packet[index++] = byte;
    if (index >= packetSize)
    {
      index = 0;
      headerFound = false;

      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++)
      {
        checksum += packet[i];
      }
      checksum &= 0xFF;

      if (checksum != packet[8])
      {
        Serial.println("Checksum error");
        Serial.print("Packet: ");
        for (int i = 0; i < packetSize; i++)
        {
          Serial.print(packet[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        return false;
      }

      latestTVOC = ((packet[2] << 8) | packet[3]) * 0.001f;
      latestCH2O = ((packet[4] << 8) | packet[5]) * 0.001f;
      latestCO2 = (packet[6] << 8) | packet[7];
      return true;
    }
  }

  return false;
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