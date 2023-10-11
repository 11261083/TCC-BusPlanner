#include <DHT.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>


/* Endereço I2C do display */
#define OLED_ADDR 0x3c

/* distancia, em pixels, de cada linha em relacao ao topo do display */
#define OLED_LINE1 0
#define OLED_LINE2 10
#define OLED_LINE3 20
#define OLED_LINE4 30
#define OLED_LINE5 40
#define OLED_LINE6 50

/* Configuração da resolucao do display (este modulo possui display 128x64) */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20  /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */


/* Definicaco do Unique ID do dispositivo */
#define My_UID            101

/* objeto do display */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 16);

bool timestampConfigured = false;

/* typedefs */
// typedef struct __attribute__((__packed__))   nao sabemos se vamos usar assim ainda
// {
//   uint32_t uid;            // Unique Identifier of the intended recipient node
//   uint8_t packetType;      // Type of packet, e.g., 0x01 for timestamp config
//   uint32_t unixTimestamp;  // Unix Timestamp from the Gateway
//   uint16_t checksum;       // Simple checksum or CRC for error-checking
// }TimeStampConfigPacket;

enum PacketTypes {
    TIMESTAMP_PACKET = 1,
    BUSARRIVALDATA_PACKET = 2
    // Adicione mais tipos conforme necessário
};

typedef struct __attribute__((__packed__))
{
  uint32_t uid;
  uint8_t packetType;
}LoRaPacketHeader;

struct TimestampPacket : public LoRaPacketHeader
{
  uint32_t timestamp;
} ;


// Initialize LoRa connection
bool LoRa_init(void)
{
    bool status_init = false;
    Serial.println("[LoRa Node] Tentando iniciar comunicacao com o radio LoRa...");
    SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
    LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, LORA_DEFAULT_DIO0_PIN);
    
    if (!LoRa.begin(BAND)) 
    {
        Serial.println("[LoRa Node] Comunicacao com o radio LoRa falhou. Nova tentativa em 1 segundo...");        
        delay(1000);
        status_init = false;
    }
    else
    {
        /* Configura o ganho do receptor LoRa para 20dBm, o maior ganho possível (visando maior alcance possível) */ 
        LoRa.setTxPower(HIGH_GAIN_LORA); 
        Serial.println("[LoRa Node] Comunicacao com o radio LoRa ok");
        status_init = true;
    }

    return status_init;
}

// Initialize Display
void display_init()
{
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) 
    {
        Serial.println("[LoRa Node] Falha ao inicializar comunicacao com OLED");        
    }
    else
    {
        Serial.println("[LoRa Node] Comunicacao com OLED inicializada com sucesso");
    
        /* Limpa display e configura tamanho de fonte */
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
    }
}

void setRTCWithUnixTimestamp(time_t unixTimestamp) {
    struct tm timeinfo;
    gmtime_r(&unixTimestamp, &timeinfo); // Converte o timestamp para tm struct
    timeinfo.tm_isdst = 0; // Define o horário de verão como desconhecido

    // Configura o RTC com a estrutura tm
    timeval tv;
    tv.tv_sec = unixTimestamp;  // definindo os segundos
    tv.tv_usec = 0;             // e os microsegundos

    timezone tz;
    tz.tz_minuteswest = 180;  // 3 horas = 180 minutos
    tz.tz_dsttime = 0;

    settimeofday(&tv, &tz);

    timestampConfigured = true;
}

void processTimestamp(TimestampPacket timestampData)
{
  uint32_t receivedTimestamp = timestampData.timestamp;
  
  setRTCWithUnixTimestamp(static_cast<time_t>(receivedTimestamp));
}

void PrintTime()  // funcao de teste, TODO: deletar depois
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Falha ao obter o tempo local");
    return;
  }

  Serial.print("Data e Hora: ");
  Serial.print(timeinfo.tm_year + 1900);  // os anos são contados a partir de 1900
  Serial.print('/');
  Serial.print(timeinfo.tm_mon + 1);     // meses vão de 0 a 11
  Serial.print('/');
  Serial.print(timeinfo.tm_mday);
  Serial.print(" ");
  Serial.print(timeinfo.tm_hour - 3);
  Serial.print(':');
  if (timeinfo.tm_min < 10) Serial.print('0'); // para garantir dois dígitos
  Serial.print(timeinfo.tm_min);
  Serial.print(':');
  if (timeinfo.tm_sec < 10) Serial.print('0'); // para garantir dois dígitos
  Serial.print(timeinfo.tm_sec);
  Serial.println();
}


void setup() 
{
  Serial.begin(115200);
  Serial.println("iniciando");

  Wire.begin(4, 15);
  display_init();
  display.clearDisplay();    
  display.setCursor(0, OLED_LINE1);
  display.print("Aguarde...");
  display.display();

  // tenta ate obter sucesso
  while(LoRa_init() == false);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  // if statement principal para a leitura do pacote
  int packetSize = 0;
  packetSize = LoRa.parsePacket();
  if(packetSize) 
  {
    // Primeiro, lemos o cabeçalho para identificar o tipo de pacote
    LoRaPacketHeader header;
    LoRa.readBytes((uint8_t*)&header, sizeof(LoRaPacketHeader));
    Serial.println(header.uid);

    if(header.uid == My_UID)
    {
      // Com base no tipo, decidimos como ler o restante
      switch(header.packetType)
      {
        case TIMESTAMP_PACKET:
          TimestampPacket timestampData;
          // Copiamos o cabeçalho já lido
          memcpy(&timestampData, &header, sizeof(LoRaPacketHeader));
          // Lemos o restante do pacote
          LoRa.readBytes(((uint8_t*)&timestampData) + sizeof(LoRaPacketHeader), sizeof(TimestampPacket) - sizeof(LoRaPacketHeader));

          // Agora, timestampData contém os dados completos
          processTimestamp(timestampData);
          break;
        
        // colocar mais case statement conforme o necessario
          case BUSARRIVALDATA_PACKET:
          

        default:
          Serial.println("Tipo de pacote desconhecido recebido.");
          break;
      }
    }
  }

  if(timestampConfigured)
  {
    PrintTime(); // funcao de teste, TODO: deletar depois
    delay(3000);
  }
}
