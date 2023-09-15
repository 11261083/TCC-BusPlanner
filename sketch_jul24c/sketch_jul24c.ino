#include <WiFi.h>
#include "time.h"
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>

// Replace with your network credentials
const char* ssid = "tcclucas";
const char* password = "tcctcctcc";

const char* mqttServer = "mqtt.tago.io";
const int mqttPort = 1883;
const char* mqttUser = "TccTagoIO"; // Replace with your TagoIO token
const char* mqttPassword = "862b7c31-7caf-4420-8dfd-2cca33c6f587"; 

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// /* GPIO do módulo WiFi LoRa 32(V2) que o pino de comunicação do sensor está ligado. */
// #define DHTPIN    13 /* (GPIO 13) */

// /* Definicoes para comunicação com radio LoRa */
// #define SCK_LORA           5
// #define MISO_LORA          19
// #define MOSI_LORA          27
// #define RESET_PIN_LORA     14
// #define SS_PIN_LORA        18

// #define HIGH_GAIN_LORA     20  /* dBm */
// #define BAND               915E6  /* 915MHz de frequencia */

// Variable to save current epoch time
unsigned long epochTime; 

WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// Setup unix
void printTime(time_t timeInput, char* output) {
  // Format the time components into the output char array
  sprintf(output, "Year: %d",
          year(timeInput));
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('\n');
    Serial.print(WiFi.status());
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  configTime(0, 0, ntpServer);

  MQTT.setServer(mqttServer, mqttPort);
  while (!MQTT.connected()) {
    Serial.println("Connecting to MQTT...");
    if (MQTT.connect("TCCTCCTCC", mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed with state: ");
      Serial.print(MQTT.state());
      delay(2000);
    }
  }
}

void loop() {
  epochTime = getTime();
  time_t readableTimestamp = (time_t)epochTime;
  char formattedTime[50];
  printTime(readableTimestamp, formattedTime);

  Serial.print("Epoch Time: ");
  Serial.println(epochTime);
  delay(1000);
  MQTT.publish("TccTagoIO/teste", formattedTime);
}