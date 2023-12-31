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
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "pedro";
const char* password = "04022018";

const char* mqttServer = "mqtt.tago.io";
const int mqttPort = 1883;
const char* mqttUser = "TccTagoIO"; //"Bus"; // Replace with your TagoIO token
const char* mqttPassword = "e8e22b2e-80b7-4f1c-9b74-708a12d9f2ce"; //"0837b686-ab2b-4b83-b318-61220fb35b55";

/* GPIO do módulo WiFi LoRa 32(V2) que o pino de comunicação do sensor está ligado. */
#define DHTPIN    13 /* (GPIO 13) */

/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20  /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */

/* Definicoes do OLED */
#define OLED_SDA_PIN    4
#define OLED_SCL_PIN    15
#define SCREEN_WIDTH    128 
#define SCREEN_HEIGHT   64  
#define OLED_ADDR       0x3C 
#define OLED_RESET      16

/* Offset de linhas no display OLED */
#define OLED_LINE1     0
#define OLED_LINE2     10
#define OLED_LINE3     20
#define OLED_LINE4     30
#define OLED_LINE5     40
#define OLED_LINE6     50

/* Definicoes gerais */
#define DEBUG_SERIAL_BAUDRATE    115200

/* Variaveis e objetos globais */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";
unsigned long epochTime; 
long timezone = -3;
byte daysavetime = 1;

WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient


/* definicoes de estrutura de tipos */
enum PacketTypes {
    TIMESTAMP_PACKET = 1,
    BUSARRIVALDATA_PACKET = 2,
    BUSARRIVALDATA_TO_GATEWAY = 3,
    // Adicione mais tipos conforme necessário
};

typedef struct __attribute__((__packed__))
{
  uint32_t lineId;
  uint8_t packetType;
}LoRaPacketHeader;

struct TimestampPacket : public LoRaPacketHeader
{
  uint32_t timestamp;
} ;

struct BusArrivalDataPacket : public LoRaPacketHeader
{
  uint32_t stopId;
  uint32_t busId;
  uint32_t time;
};

#define MAX_HISTORY  8   //
struct BusInfo 
{
  uint32_t busId;
  uint32_t lineId;
  uint32_t stopId;
  uint32_t time;
  bool infoType; // estimativa: false, tempo de chegada: true;
};
BusInfo busHistory[MAX_HISTORY];
int currentHistoryIndex = 0;



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

// Initialize Display
void display_init()
{
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) 
    {
        Serial.println("[LoRa Gateway] Falha ao inicializar comunicacao com OLED");        
    }
    else
    {
        Serial.println("[LoRa Gateway] Comunicacao com OLED inicializada com sucesso");
    
        /* Limpa display e configura tamanho de fonte */
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
    }
}

void MQTT_init()
{
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


// Initialize LoRa Communication
bool LoRa_init(void)
{
    bool status_init = false;
    Serial.println("[LoRa Gateway] Tentando iniciar comunicacao com o radio LoRa...");
    SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
    LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, LORA_DEFAULT_DIO0_PIN);
    
    if (!LoRa.begin(BAND)) 
    {
        Serial.println("[LoRa Gateway] Comunicacao com o radio LoRa falhou. Nova tentativa em 1 segundo...");        
        delay(1000);
        status_init = false;
    }
    else
    {
        /* Configura o ganho do receptor LoRa para 20dBm, o maior ganho possível (visando maior alcance possível) */ 
        LoRa.setTxPower(HIGH_GAIN_LORA); 
        Serial.println("[LoRa Gateway] Comunicacao com o radio LoRa ok");
        status_init = true;
    }

    return status_init;
}


void envia_configTimestamp_LoRa(uint32_t timestamp)
{
  TimestampPacket timestampPacket;

  timestampPacket.lineId = 0;
  timestampPacket.packetType = TIMESTAMP_PACKET;
  timestampPacket.timestamp = timestamp;

  LoRa.beginPacket();
  LoRa.write((unsigned char *)&timestampPacket, sizeof(TimestampPacket));
  Serial.println("Sending Timestamp Configuration");
  Serial.println(sizeof(TimestampPacket));
  Serial.println(timestampPacket.lineId);
  Serial.println(timestampPacket.timestamp);
  LoRa.endPacket();
  if (LoRa.endPacket() == 1) {
  Serial.println("Pacote enviado com sucesso!");
  } 
  else {
    Serial.println("Falha ao enviar o pacote.");
  }
}

void processBusArrivalPacketData(BusArrivalDataPacket arrivalData)
{
  Serial.println("Bus ID: " + String(arrivalData.busId));
  Serial.println("Line ID: " + String(arrivalData.lineId));
  Serial.println("Stop ID: " + String(arrivalData.stopId));

  uint32_t unixTime = arrivalData.time;
  struct tm timeStruct;
  gmtime_r((const time_t *)&unixTime, &timeStruct);

  int currentHour = timeStruct.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }

  Serial.println("Arrival Time: " + String(currentHour) + " : " + String(timeStruct.tm_min) + " : " + String(timeStruct.tm_sec));

  String date = (String((timeStruct.tm_year) + 1900) + "-" + String(( timeStruct.tm_mon) + 1) + "-" + String(timeStruct.tm_mday));
  String hour = (String(currentHour) + ":" + String(timeStruct.tm_min) + ":" + String(timeStruct.tm_sec));
  const char* timeChar = ("Date: " + date + " - Time: " + hour).c_str();

  const size_t  capacity = JSON_OBJECT_SIZE(5);
  StaticJsonDocument<capacity> doc;
  doc["id"] = arrivalData.busId;
  doc["line"] = arrivalData.lineId;
  doc["date"] = date;
  doc["time"] =  hour;
  doc["timestamp"] = unixTime;
  String mqttOutput;
  serializeJson(doc, mqttOutput);

  MQTT.publish("info/bus", mqttOutput.c_str());

  BroadcastArrivalData(arrivalData);
}

void BroadcastArrivalData(BusArrivalDataPacket arrivalData)
{
  arrivalData.packetType = BUSARRIVALDATA_PACKET;

  LoRa.beginPacket();
  LoRa.write((unsigned char *)&arrivalData, sizeof(BusArrivalDataPacket));
  Serial.println("Sending Bus data");
  Serial.println(arrivalData.lineId);
  Serial.println(arrivalData.stopId);
  LoRa.endPacket();
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  MQTT_init();
  while(LoRa_init() == false);

  configTime(3600 * timezone, 0, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");

  struct tm timeInfo ;
  Serial.println(timeInfo.tm_min);
  timeInfo.tm_year = 0;
  if (!getLocalTime(&timeInfo, 10000)) {
    Serial.println("Failed to obtain time");
    return;
  }
  time_t unix_timestamp = mktime(&timeInfo);
  Serial.println(unix_timestamp);
  if(unix_timestamp >= 1600000) 
  {
    uint32_t timestamp_to_send = static_cast<uint32_t>(unix_timestamp);
    Serial.println("Enviando Timestamp");
    // Envie o timestamp_to_send para o nó via LoRa
    envia_configTimestamp_LoRa(timestamp_to_send);
  } 
  else 
  {
    // Trate erro: timestamp é anterior a 1970, o que não é esperado em seu caso
    Serial.println("Erro no timestamp");
  }
}

/* variaveis utilizados para simular onibus chegando, realiza o trabalho de um contador assincrono que permite nao utilizar delay no loop principal */
unsigned long previousMillis = 0;
const long interval = 30000; // 30s

void loop()
 {
  // if statement principal para a leitura do pacote
  int packetSize = 0;
  packetSize = LoRa.parsePacket();
  if(packetSize) 
  {
    // Primeiro, lemos o cabeçalho para identificar o tipo de pacote
    LoRaPacketHeader header;
    LoRa.readBytes((uint8_t*)&header, sizeof(LoRaPacketHeader));
    Serial.println(header.lineId);

    if(header.lineId != 0)
    {
      // Com base no tipo, decidimos como ler o restante
      switch(header.packetType)
      {
        // colocar mais case statement conforme o necessario
        case BUSARRIVALDATA_TO_GATEWAY:
          BusArrivalDataPacket arrivalData;
          memcpy(&arrivalData, &header, sizeof(LoRaPacketHeader));
          LoRa.readBytes(((uint8_t*)&arrivalData) + sizeof(LoRaPacketHeader), sizeof(BusArrivalDataPacket) - sizeof(LoRaPacketHeader));
          processBusArrivalPacketData(arrivalData);
          break;
          

        default:
          Serial.println("Tipo de pacote desconhecido recebido.");
          break;
      }
    }
  }
}