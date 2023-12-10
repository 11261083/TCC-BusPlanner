#include <WiFi.h>
#include "time.h"
#include <ctime>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <map>

// Replace with your network credentials
const char* ssid = "pedro";
const char* password = "04022018";

const char* mqttServer = "mqtt.tago.io";
const int mqttPort = 1883;
const char* mqttUser = "Bus2"; // Replace with your TagoIO token
const char* mqttPassword = "e76ce4ff-dc5f-4317-8852-0e8b4b43bbd9"; 

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
    BUSPREDICTDATA_PACKET = 4,
    // Adicione mais tipos conforme necessário
};

std::map<String, int> stopIdToIndex = {
    {"101", 0},
    {"102", 1},
    {"8012", 0},
    {"8022", 1},
    // outros mapeamentos
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

struct BusPredictDataPacket : public LoRaPacketHeader
{
  uint32_t stopId;
  uint32_t busId;
  uint32_t predictTime;
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

// ASSUMINDO
//   STOPS: 1 e 2
//   LINES: 12 e 22

const int NUM_STOPS = 2;
const int NUM_LINES = 2;
const int NUM_RECORDS = 2;  // Para armazenar os últimos 2 ônibus

struct BusStopData {
  BusArrivalDataPacket records[NUM_RECORDS]; 
};

BusStopData stops[NUM_STOPS][NUM_LINES];

void initRecords()
{
  for(int i = 0; i < NUM_STOPS; i++)
  {
    for(int j = 0; j < NUM_LINES; j++)
    {
      for(int k = 0; k < NUM_RECORDS; k++)
      {
        stops[i][j].records[k].lineId = 0;
        stops[i][j].records[k].packetType = 0;
        stops[i][j].records[k].busId = 0;
        stops[i][j].records[k].stopId = 0;
        stops[i][j].records[k].time = 0;
      }
    }
  }
  Serial.println(stops[0][0].records[0].time);
}

// Função que atualiza os arrays de cada parada, mantendo sempre 2 itens em cada array
void updateRecords(int stopId, int lineId, const BusArrivalDataPacket& newRecord) {

  // Desloca os registros existentes para abrir espaço para o novo registro
  for (int i = NUM_RECORDS - 1; i > 0; --i) {
    stops[stopId][lineId].records[i] = stops[stopId][lineId].records[i - 1];
  }

    // Insere o novo registro no início do array
    // Sempre o último ônibus que passou estará na posição 0 do Array
    Serial.println(newRecord.time);
    stops[stopId][lineId].records[0] = newRecord;
}

// Função para gerar uma previsão de tempo com base nos últimos registros
uint32_t getPredictTime(int stopId, int lineId) {
    
  if(stops[stopId][lineId].records[1].time != 0 &&
      stops[stopId + 1][lineId].records[0].time != 0)
      {
        return stops[stopId + 1][lineId].records[0].time - stops[stopId][lineId].records[1].time;
      }
  // else if(stopId == 0){
  //   return 60; // 1 min
  // }
  else{
    return 0;
  }
}


void processBusArrivalPacketData(BusArrivalDataPacket arrivalData)
{
  Serial.println("Bus ID: " + String(arrivalData.busId));
  Serial.println("Line ID: " + String(arrivalData.lineId));
  Serial.println("Stop ID: " + String(arrivalData.stopId));

  int stopIndex = stopIdToIndex[String(arrivalData.stopId)];
  Serial.println(stopIndex);
  int lineIndex = stopIdToIndex[String(arrivalData.lineId)];

  // ATUALIZANDO ARRAYS
  updateRecords(stopIndex, lineIndex, arrivalData);

  // OBTENDO PREVISAO
  uint32_t predictTime = getPredictTime(stopIndex, lineIndex);
  Serial.println(predictTime);

  // ENVIANDO PACOTE PREDICT VIA MQTT P/ TAGOIO
  BusPredictDataPacket busPredictDataPacket;
  busPredictDataPacket.stopId = arrivalData.stopId + 1;
  busPredictDataPacket.lineId = arrivalData.lineId;
  busPredictDataPacket.busId = arrivalData.busId;
  busPredictDataPacket.packetType = BUSPREDICTDATA_PACKET;
  busPredictDataPacket.predictTime = arrivalData.time + predictTime;
  // chamar func p/ enviar p/ tagoIO

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

  const size_t  capacity = JSON_OBJECT_SIZE(15);
  StaticJsonDocument<capacity> doc;
  doc["R_id_"+String(arrivalData.stopId)] = arrivalData.busId;
  doc["R_line_"+String(arrivalData.stopId)] = arrivalData.lineId;
  doc["R_date_"+String(arrivalData.stopId)] = date;
  doc["R_hour_"+String(arrivalData.stopId)] =  hour;
  doc["R_time_"+String(arrivalData.stopId)] = unixTime;
  doc["R_time_"+String(arrivalData.stopId)+ "_" + String(arrivalData.lineId)] = unixTime; //atencao! variavel para display
  String topic = "info/" + String(arrivalData.stopId);
  String mqttOutput;
  serializeJson(doc, mqttOutput);

  MQTT.publish(topic.c_str(), mqttOutput.c_str());

  unixTime = arrivalData.time + predictTime;
  struct tm timeStruct1;
  gmtime_r((const time_t *)&unixTime, &timeStruct1);

  currentHour = timeStruct1.tm_hour - 3;
  if(currentHour < 0)
  {
    currentHour = currentHour + 24;
  }

  date = (String((timeStruct1.tm_year) + 1900) + "-" + String(( timeStruct1.tm_mon) + 1) + "-" + String(timeStruct1.tm_mday));
  hour = (String(currentHour) + ":" + String(timeStruct1.tm_min) + ":" + String(timeStruct1.tm_sec));
  timeChar = ("Date: " + date + " - Time: " + hour).c_str();
  Serial.println(hour);

  StaticJsonDocument<capacity> doc2;
  doc2["P_id_"+String(busPredictDataPacket.stopId)] = busPredictDataPacket.busId;
  doc2["P_line_"+String(busPredictDataPacket.stopId)] = busPredictDataPacket.lineId;
  doc2["P_date_"+String(busPredictDataPacket.stopId)] = date;
  doc2["P_hour_"+String(busPredictDataPacket.stopId)] =  hour;
  doc2["P_time_"+String(busPredictDataPacket.stopId)] = unixTime;
  doc2["P_time_"+String(busPredictDataPacket.stopId) + "_" + String(busPredictDataPacket.lineId)] = unixTime; //atencao! variavel para display
  String topic2 = "predicts/" + String(busPredictDataPacket.stopId);
  String mqttOutput2;
  serializeJson(doc2, mqttOutput2);

  MQTT.publish(topic2.c_str(), mqttOutput2.c_str());

  BroadcastPredictData(busPredictDataPacket);
}

void BroadcastArrivalData(BusPredictDataPacket busPredictDataPacket)
{
  busPredictDataPacket.packetType = BUSPREDICTDATA_PACKET;

  LoRa.beginPacket();
  LoRa.write((unsigned char *)&busPredictDataPacket, sizeof(BusPredictDataPacket));
  Serial.println("Sending Arrival Data");
  Serial.println(busPredictDataPacket.lineId);
  Serial.println(busPredictDataPacket.stopId);
  LoRa.endPacket();
}

void BroadcastPredictData(BusPredictDataPacket predictData)
{
  predictData.packetType = BUSPREDICTDATA_PACKET;

  LoRa.beginPacket();
  LoRa.write((unsigned char *)&predictData, sizeof(BusPredictDataPacket));
  Serial.println("Sending Predict Data");
  Serial.println(predictData.lineId);
  Serial.println(predictData.stopId);
  LoRa.endPacket();
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  MQTT_init();
  while(LoRa_init() == false);
  configTime(3600 * timezone, 0, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");

  struct tm tmstructFirst ;
  tmstructFirst.tm_year = 0;
  getLocalTime(&tmstructFirst);

  time_t unix_timestamp = mktime(&tmstructFirst);
  Serial.println(unix_timestamp);
  if(unix_timestamp >= 1000000) 
  {
    uint32_t timestamp_to_send = static_cast<uint32_t>(unix_timestamp);
    Serial.println("entrou");
    // Envie o timestamp_to_send para o nó via LoRa
    envia_configTimestamp_LoRa(timestamp_to_send);
  } 
  else 
  {
    // Trate erro: timestamp é anterior a 1970, o que não é esperado em seu caso
    Serial.println("Erro no timestamp");
  }
  initRecords();
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
