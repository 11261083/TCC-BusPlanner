/*
  Projeto: InterSCity - Projeto IPT
  Teste 01 - transmissão de dados do mpu9250 e RTC DS3231 do ESP32 árvore para o ESP32 sala 09 do IPT
  Autor: José Sinézio
  Data 24-06-2022 realizado carga do programa para teste - alterada a senha dia 08-07-2022 - Alterado a senha dia 11-08-2022
  Dia 04-06-2023 ativo wifi casa -
*/

/* Includes para das biliotecas */
#include <LoRa.h>               // biblioteca de controle do protocolo LoRa
#include <SPI.h>                // biblioteca de controle do canal SPI, para controle do canal LoRa
#include <Wire.h>               // biblioteca de controle do canal I2C
#include <Adafruit_GFX.h>       // biblioteca de controle do display ESP32 Heltec
#include <Adafruit_SSD1306.h>   // biblioteca de controle do display ESP32 Heltec
#include <HTTPClient.h>         // biblioteca de controle HTTP para envio de dados para o InterSCity 
#include <esp_task_wdt.h>       //Biblioteca do watchdog
#include <PubSubClient.h>       //Biblioteca cliente para mensagens MQTT
#include <stdio.h>              //Para que serve a biblioteca Stdio.h A linha #include <stdio. h> diz ao compilador que ele deve incluir o arquivo-cabeçalho stdio. h. Neste arquivo existem declarações de funções úteis para entrada e saída de dados (std = standard, padrão em inglês; io = Input/Output, entrada e saída ==> stdio = Entrada e saída padronizadas)
#include <ArduinoJson.h>        //Biblioteca JSON simples e eficiente para C++ incorporado


/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18
#define IRQ_PIN_LORA       26
#define HIGH_GAIN_LORA     20     /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */

/* Endereço I2C do display, MPU6050 */
#define OLED_ADDR   0x3c
#define MPU   0x69

/* distancia, em pixels, de cada linha em relacao ao topo do display */
#define OLED_LINE1     0
#define OLED_LINE2     10
#define OLED_LINE3     20
#define OLED_LINE4     30
#define OLED_LINE5     40
#define OLED_LINE6     50

/* Configuração da resolucao do display (este modulo possui display 128x64) */
#define SCREEN_WIDTH    128 
#define SCREEN_HEIGHT   64
#define OLED_RESET      16

/* Definicoes gerais */
#define DEBUG_SERIAL_BAUDRATE    115200 // Velocidade de comunicação com o monitor

//Tempo para ativar o reset do watchdog timer
#define WDT_TIMEOUT 30     // tempo em segundos para o watchdog timer

/* MQTT definitions */
#define MQTT_PUB_TOPIC "tago/data/post"
#define MQTT_USERNAME  "Dispositivo_Poli_IPT_02"  /* Coloque aqui qualquer valor */
#define MQTT_PASSWORD  "bdbdc660-2e6d-49d3-b5db-36dafb4747dd"  /* coloque aqui o Device Token do seu dispositivo no Tago.io */

//#define MQTT_PUB_TOPIC "tago/data/post"
//#define MQTT_USERNAME  "dispositivo_iot_Poli_sin"  /* Coloque aqui qualquer valor */
//#define MQTT_PASSWORD  "dd7b89df-dabe-4345-8c6b-938a5e0a86d1"  /* coloque aqui o Device Token do seu dispositivo no Tago.io */



/* Objeto do display do heltec*/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* Variaveis e objetos globais */
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient

// Objeto cliente HTTP
HTTPClient gHttp;

/* Variáveis para armazenar valores, sensor, RTC, ... */
  float FAcX,FAcY,FAcZ,FTmp,FGyX,FGyY,FGyZ;  // Valores com fator de correção aplicado
  long num_men_enviada;
  int Display_L4 = 1;                   // Controla o que vai mostra na linha 3 do display
  String ValorDate01, ValorDate02, ValorDate03;      // Valor string para mostrar no formato ISO 8601
  int vtimeYYYY, vtimeMM, vtimeDD, vtimehh, vtimemm, vtimess;
  unsigned long vtimemili;
  String SvtimeYYYY, SvtimeMM, SvtimeDD, Svtimehh, Svtimemm, Svtimess;
  String Svtimemili;
  // Declaração dos dados a ser recebidos via canal LoRa 
  float acelX;
  float acelY;
  float acelZ;
  float temp;
  float girX;
  float girY;
  float girZ;
  long num_men;
  int timeYYYY;
  int timeMM;
  int timeDD;
  int timehh;
  int timemm;
  int timess;
  unsigned long timemili;
  int cdisplay;

  float peso_lida = 1.0;              //valor lido da célula de carga
  float dirvento_graus_lida = 10.0;   //Mostra a direção do vento em graus
  float velvento_lida = 5.0;         //Mostra a velocidade do vento em Km/h


// Controle do teste de Watchdog Timer
int count01 = 1;                     // Valor para o teste de Watchdog Timer
int valresetwdt = 0;                 // Reseta valor para teste de WDT

// Controle de recebimento de dados via monitor
int incomingByte = 0; // variável para o dado recebido
char myString[16];



/* typedefs - Dados estruturados para recebimento de dados via canal LoRa*/
typedef struct __attribute__((__packed__))  
{
  float acelX;
  float acelY;
  float acelZ;
  float temp;
  float girX;
  float girY;
  float girZ;
  long num_men;
  int timeYYYY;
  int timeMM;
  int timeDD;
  int timehh;
  int timemm;
  int timess;
  unsigned long timemili;
  int cdisplay;

}TDadosLora;

// Ativa transmissão de dados
static unsigned long ativaWiFi;               // Ativa a transmissão dos dados via WiFi

/* Constantes para WiFi*/
const char* SSID = "josesinezio 2.4"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "fatima19571960"; // coloque aqui a senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPT-WiFi"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "quadrig@e"; // coloque aqui a senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPT-WiFi-Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "quadrig@e"; // coloque aqui a senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPT-WiFi-Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "magistr@e"; // coloque aqui a senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPT-WiFi-Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "c@rpatos"; // coloque aqui a senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPT-WiFi-Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "esci@tos"; // coloque aqui a senha da rede WI-FI que deseja se conectar - colocada senha dia 11-08-2022

// senha colocada dia 20-09-2022
//const char* SSID = "IPT_Colaborador_Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "abc2k21!"; // coloque aqui a senha da rede WI-FI que deseja se conectar

// senha colocada dia 20-09-2022
//const char* SSID = "IPT-WiFi-Novo"; // coloque aqui o SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "s@rdenha"; // coloque aqui a senha da rede WI-FI que deseja se conectar - colocada senha dia 11-08-2022


/* MQTT */
const char* broker_mqtt = "mqtt.tago.io"; /* MQTT broker URL */
int broker_port = 1883;                      /* MQTT broker port */


/* Local prototypes rotina locais*/
void display_init(void);
bool init_comunicacao_lora(void);
void init_wifi(void);
void connect_wifi(void);
void verify_wifi_connection(void);
void init_MQTT(void);
void connect_MQTT(void);
void verify_mqtt_connection(void);
void send_data_iot_platform(void);

/* Funcao: inicializa comunicacao com o display OLED
 * Parametros: nenhnum
 * Retorno: nenhnum
*/ 
void display_init(void)
{
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) 
    {
        Serial.println("[LoRa Receiver] Falha ao inicializar comunicacao com OLED");        
    }
    else
    {
        Serial.println("[LoRa Receiver] Comunicacao com OLED inicializada com sucesso");
    
        /* Limpa display e configura tamanho de fonte */
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
    }
}



/* Funcao: inicia comunicação com chip LoRa
 * Parametros: nenhum
 * Retorno: true: comunicacao ok
 *          false: falha na comunicacao
*/
bool init_comunicacao_lora(void)
{
    bool status_init = false;
    Serial.println("[LoRa Receiver] Tentando iniciar comunicacao com o radio LoRa...");
    SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
    LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, IRQ_PIN_LORA);
    
    if (!LoRa.begin(BAND)) 
    {
        Serial.println("[LoRa Receiver] Comunicacao com o radio LoRa falhou. Nova tentativa em 1 segundo...");        
        delay(1000);
        status_init = false;
    }
    else
    {
        /* Configura o ganho do receptor LoRa para 20dBm, o maior ganho possível (visando maior alcance possível) */ 
        LoRa.setTxPower(HIGH_GAIN_LORA); 
        Serial.println("[LoRa Receiver] Comunicacao com o radio LoRa ok");
        status_init = true;
    }

    return status_init;
}




/* Função: inicializa e conecta-se na rede WI-FI desejada
 * Parâmetros: nenhum
 *Retorno: nenhum
*/
void init_wifi(void) 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");    
    connect_wifi();
}


void connect_wifi(void) 
{
    /* se já está conectado a rede WI-FI, nada é feito. 
       Caso contrário, são efetuadas tentativas de conexão */
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
    
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(5000);
        Serial.print(SSID); Serial.print(" - ");
        Serial.print(PASSWORD); Serial.print(" - ");
        Serial.print("."); Serial.println(" - ");
    }
  
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}

/* Funcao: verifica e garante conexao wi-fi
 * Parametros: nenhum
 * Retorno: nenhum 
 */
void verify_wifi_connection(void)
{
    connect_wifi(); 
}

/* Função: inicializa parâmetros de conexão MQTT(endereço do broker e porta)
 * Parâmetros: nenhum
 * Retorno: nenhum
*/
void init_MQTT(void) 
{
    //informa qual broker e porta deve ser conectado
    MQTT.setServer(broker_mqtt, broker_port);   
}

void connect_MQTT(void) 
{
    char mqtt_id_randomico[10] = {0};
  
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(broker_mqtt);

        /* gera id mqtt randomico */
        randomSeed(random(9999));
        sprintf(mqtt_id_randomico, "%ld", random(9999));
        
        if (MQTT.connect(mqtt_id_randomico, MQTT_USERNAME, MQTT_PASSWORD))
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");           
        }
        else 
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
            break;
        }
    }
}

/* Funcao: verifica e garante conexao MQTT
 * Parametros: nenhum
 * Retorno: nenhum 
 */

void verify_mqtt_connection(void)
{
    connect_MQTT();  
}

/*
JSON a ser enviado para Tago.io:

{
    "variable": "nome_da_variavel",
    "unit"    : "unidade",
    "value"   : valor
}
*/

void send_data_iot_platform(void)
{

    if (peso_lida < 2.0) {
      peso_lida = peso_lida + 0.1;  // do Thing A
    }
    else {
      peso_lida = 1.0;// do Thing C
    }

    if (dirvento_graus_lida < 360.0) {
      dirvento_graus_lida = dirvento_graus_lida + 10.0;  // do Thing A
    }
    else {
      dirvento_graus_lida = 10.0;// do Thing C
    }


    if (dirvento_graus_lida < 30.0) {
     velvento_lida = velvento_lida + 1.0;  // do Thing A
    }
    else {
      velvento_lida = 5.0;// do Thing C
    }
       

   StaticJsonDocument<250> tago_json_temperature;
   StaticJsonDocument<250> tago_json_acelerometroX;
   StaticJsonDocument<250> tago_json_acelerometroY;
   StaticJsonDocument<250> tago_json_acelerometroZ;
   StaticJsonDocument<250> tago_json_mensagem;
   StaticJsonDocument<250> tago_json_peso;
   StaticJsonDocument<250> tago_json_dirvento;
   StaticJsonDocument<250> tago_json_velvento;
   char json_string[250] = {0};


   /* Envio da temperatura *//*
   tago_json_temperature["variable"] = "temperatura";
   tago_json_temperature["unit"] = "C";
   tago_json_temperature["value"] = FTmp;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_temperature, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);
*/
   /* Envio da acelerometroX */
   tago_json_acelerometroX["variable"] = "acelerometroX";
   tago_json_acelerometroX["unit"] = "g";
   tago_json_acelerometroX["value"] = FAcX;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_acelerometroX, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);

   /* Envio da acelerometroY */
   tago_json_acelerometroY["variable"] = "acelerometroY";
   tago_json_acelerometroY["unit"] = "g";
   tago_json_acelerometroY["value"] = FAcY;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_acelerometroY, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);

   /* Envio da acelerometroZ */
   tago_json_acelerometroZ["variable"] = "acelerometroZ";
   tago_json_acelerometroZ["unit"] = "g";
   tago_json_acelerometroZ["value"] = FAcZ;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_acelerometroZ, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);

   /* Envio da mensagem */
   tago_json_mensagem["variable"] = "mensagem";
   tago_json_mensagem["unit"] = " ";
   tago_json_mensagem["value"] = num_men_enviada;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_mensagem, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);

   /* Envio do peso *//*
   tago_json_peso["variable"] = "peso";
   tago_json_peso["unit"] = "g";
   tago_json_peso["value"] = peso_lida;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_peso, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);
*/
   /* Envio do direção do vento *//*
   tago_json_dirvento["variable"] = "direcaovento";
   tago_json_dirvento["unit"] = "graus";
   tago_json_dirvento["value"] = dirvento_graus_lida;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_dirvento, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);
*/
   /* Envio do velocidade do vento *//*
   tago_json_velvento["variable"] = "velocidadevento";
   tago_json_velvento["unit"] = "Km/h";
   tago_json_velvento["value"] = velvento_lida;
   memset(json_string, 0, sizeof(json_string));
   serializeJson(tago_json_velvento, json_string);
   MQTT.publish(MQTT_PUB_TOPIC, json_string);
*/      
}



/* Funcao setup */
void setup() 
{

    // Faz a primeira inicialização da variável ativaWiFi
    ativaWiFi = millis();  

    // Inicializa canal seral do monitor
    Serial.begin(DEBUG_SERIAL_BAUDRATE);


    
    // Inicializa canal I2C do display OLED, MPU6050 e RTC 
    Wire.begin (4, 15);   // sda= GPIO_04 e scl= GPIO_15

    /* Display init */
    display_init();

    /* Print message telling to wait */
    display.clearDisplay();    
    display.setCursor(0, OLED_LINE1);
    display.print("Aguarde Init LoRa...");
    display.display();

    /* Tenta, até obter sucesso, comunicacao com o chip LoRa */
    while(init_comunicacao_lora() == false); 
        Serial.println("LoRa: comunicação inicializada");


    /* inicializações do WI-FI e MQTT e faz conexao ao broker MQTT  */
    init_wifi();
    init_MQTT();
    connect_MQTT(); 

    //Habilita o watchdog configurando o timeout para o valor de WDT_TIMEOUT segundos
    esp_task_wdt_init(WDT_TIMEOUT, true);   //comando de habilita  o ESP32 para resetar
    esp_task_wdt_add(NULL);                 //Adicionar rotina atual ao relógio WDT

         
}      

/* Programa principal */
void loop() 
{

    // apenas responde quando dados são recebidos:
    if (Serial.available() > 0) {
      // lê do buffer o dado recebido:
      incomingByte = Serial.parseInt();
      //incomingByte = Serial.read();
      myString[16] = incomingByte;
  
      if (incomingByte != 0) {
      // responde com o dado recebido:
      Serial.print("I received: ");
      Serial.print(" - Decimal: "); Serial.print(incomingByte, DEC);
      Serial.print(" - Hexadecimal: "); Serial.print(incomingByte, HEX);
      Serial.print(" - Caracter: "); Serial.println(myString[16]);
      valresetwdt = incomingByte;
      }
    }

    //Serial.print("valresetwdt = "); Serial.println(valresetwdt);
  
    // Teste de Watchdog Timer
    //Fica preso no loop enquanto o botão estiver pressionado
    while (valresetwdt == 12) {
      Serial.print("Teste de WatchDog Timer... ");
      Serial.print("Contagem até 20 - Valor atual = ");
      Serial.println(count01);
      count01++;
      delay(1000);
    }


     /* Verifica se as conexões wi-fi estão ativas 
     Se alguma delas não estiver ativa, a reconexão é feita */
     verify_wifi_connection(); 



    TDadosLora dados_lora;
    
    char byte_recebido;
    int packet_size = 0;
    int lora_rssi = 0;
    long informacao_recebida = 0;
    char * ptInformaraoRecebida = NULL;
  
    /* Verifica se chegou alguma informação do tamanho esperado */
    packet_size = LoRa.parsePacket();
    
    if (packet_size == sizeof(TDadosLora)) 
    {
        Serial.print("[LoRa Receiver] Há dados a serem lidos - Númro da mensagem = ");
        //Serial.println(dados_lora.num_men);
        
        /* Recebe os dados conforme protocolo */               
        ptInformaraoRecebida = (char *)&dados_lora;  
        while (LoRa.available()) 
        {
            byte_recebido = (char)LoRa.read();
            *ptInformaraoRecebida = byte_recebido;
            ptInformaraoRecebida++;
        }

        Serial.println(dados_lora.num_men);


        // Dados recebido via canal LoRa
        FAcX = dados_lora.acelX;
        FAcY = dados_lora.acelY;
        FAcZ = dados_lora.acelZ;
        FTmp = dados_lora.temp;
        FGyX = dados_lora.girX;
        FGyY = dados_lora.girY;
        FGyZ = dados_lora.girZ;
        num_men_enviada = dados_lora.num_men;
        vtimeYYYY = dados_lora.timeYYYY;
        vtimeMM = dados_lora.timeMM;
        vtimeDD = dados_lora.timeDD;
        vtimehh = dados_lora.timehh;
        vtimemm = dados_lora.timemm;
        vtimess = dados_lora.timess;
        vtimemili = dados_lora.timemili;
        Display_L4 = dados_lora.cdisplay;



        // Transforma o valor recebido do RTC pela comunicação LoRa de inteiro em string
        SvtimeYYYY = String(vtimeYYYY, DEC);
        if (vtimeMM < 10) {         // acrescenta o carácter "0" zero na frente do número quando for apenas um
          SvtimeMM = "0" + String(vtimeMM, DEC);
        }
        else {                      // Quando for dois caracteres só transforma de inteiro para string
          SvtimeMM = String(vtimeMM, DEC);
        }
        if (vtimeDD < 10) {         // acrescenta o carácter "0" zero na frente do número quando for apenas um
          SvtimeDD = "0" + String(vtimeDD, DEC);
        }
        else {                      // Quando for dois caracteres só transforma de inteiro para string
          SvtimeDD = String(vtimeDD, DEC);
        }
        if (vtimehh < 10) {         // acrescenta o carácter "0" zero na frente do número quando for apenas um
          Svtimehh = "0" + String(vtimehh, DEC);
        }
        else {                      // Quando for dois caracteres só transforma de inteiro para string
          Svtimehh = String(vtimehh, DEC);
        }
        if (vtimemm < 10) {         // acrescenta o carácter "0" zero na frente do número quando for apenas um
          Svtimemm = "0" + String(vtimemm, DEC);
        }
        else {                      // Quando for dois caracteres só transforma de inteiro para string
          Svtimemm = String(vtimemm, DEC);
        }
        if (vtimess < 10) {         // acrescenta o carácter "0" zero na frente do número quando for apenas um
          Svtimess = "0" + String(vtimess, DEC);
        }
        else {                      // Quando for dois caracteres só transforma de inteiro para string
          Svtimess = String(vtimess, DEC);
        }
        if (vtimemili < 10) {         // acrescenta o carácter "00" zero na frente do número quando for apenas um
          Svtimemili = "00" + String(vtimemili, DEC);
        }
        else if (vtimemili < 100) {         // acrescenta o carácter "00" zero na frente do número quando for apenas dois
          Svtimemili = "0" + String(vtimemili, DEC);
        }
        else {                      // Quando for três caracteres só transforma de inteiro para string
          Svtimemili = String(vtimemili, DEC);
        }        
    
        // Monta a string do TimeStamp da IOS 8601
        ValorDate01 = SvtimeYYYY + "-" + SvtimeMM + "-" + SvtimeDD + "T" + Svtimehh + ":" + Svtimemm + ":" + Svtimess;  // Monta Ano/Mês/Dia/hora/minuto/sendo para a string "YYYY-MM-DDThh:mm:ss"
        ValorDate02 = ValorDate01 + "." + Svtimemili + "+00:00";  // Monta Ano/Mês/Dia/hora/minuto/Milisegundo/fuso horário sendo para a string "YYYY-MM-DDThh:mm:ss.mmm+00:00"
    
        //Mostra os valores na serial
        Serial.print("Acel. X = "); Serial.print(FAcX);
        Serial.print(" | Y = "); Serial.print(FAcY);
        Serial.print(" | Z = "); Serial.print(FAcZ);
        Serial.print(" | Gir. X = "); Serial.print(FGyX);
        Serial.print(" | Y = "); Serial.print(FGyY);
        Serial.print(" | Z = "); Serial.println(FGyZ);
    
        Serial.print(SvtimeYYYY); Serial.print("-");
        Serial.print(SvtimeMM); Serial.print("-");
        Serial.print(SvtimeDD); Serial.print("T");
        Serial.print(Svtimehh); Serial.print(":");
        Serial.print(Svtimemm); Serial.print(":");
        Serial.print(Svtimess); Serial.print(".");
        Serial.print(Svtimemili); Serial.print("-03:00");     
        Serial.println("-Final"); 


        // Ativa transmissão com WiFi
        if ((millis() - ativaWiFi) > 1000) { // era 30000

        // Verifica status da da WiFi
        if (WiFi.status() == WL_CONNECTED) //check se WiFi conectado
        {
          HTTPClient http;      
          // Your Domain name with URL path or IP address with path           
          String vUri = "http://Iot.ipt.br:8000/adaptor/resources/be86b918-3c26-4c5f-9d7d-f3c6a58506db/data";
                
          Serial.println("passou aqui 01");
    
          String strSend = "{\"data\": {\"environment_monitoring\": [";
          strSend += "{\"Temp\": " + String(FTmp, 2) + ",";           //envia inteiro
          strSend += "\"AcX\": " + String(FAcX, 2) + ",";        //envia decimal com 3 casas
          strSend += "\"AcY\": " + String(FAcY, 2) + ",";
          strSend += "\"AcZ\": " + String(FAcZ, 2) + ",";
          strSend += "\"GyX\": " + String(FGyX, 2) + ",";
          strSend += "\"GyY\": " + String(FGyY, 2) + ",";
          strSend += "\"GyZ\": " + String(FGyZ, 2) + ",";
          strSend += "\"date\":\"" + ValorDate02 + "\"";
          strSend += "}";
          strSend += "]}}";
          Serial.println(strSend);

          Serial.println("passou aqui 02 - WiFi");

        /* Verifica se as conexões MQTT e wi-fi estão ativas 
        Se alguma delas não estiver ativa, a reconexão é feita */
        verify_mqtt_connection();

        /* Faz o envio da temperatura e umidade para a plataforma IoT (Tago.io) */
        send_data_iot_platform();


          ativaWiFi = millis();
          Serial.println("Passou aqui - Transmitiu - WiFi");
          
          gHttp.begin(vUri);      
          //gHttp.setTimeout(10000); //<<<<<
          gHttp.setConnectTimeout(5000); 
          gHttp.addHeader("Content-Type", "application/json"); //enviar formato json
          //gHttp.addHeader("Content-Type", "text/plain"); // envia formta texto
          int httpResponseCode = gHttp.POST(strSend);
          Serial.print("HTTP Response code: ");
          Serial.println(httpResponseCode);
          if (httpResponseCode == 200){ //recebeu corretamente
            String vPayload = gHttp.getString(); //pegar resposta               
            Serial.print(vPayload);

          Serial.println("passou aqui 03 - WiFi");
                            
          } else {
            //erro 
            //httpResponseCode  tem o codigo de erro http
            //ex: httpResponseCode = 403 sem permissão
          } 
          // Free resources
          gHttp.end();//Send                
      }

      }

       

        // Enviados dados para o display e dados recebidos pelo canal LoRa
        lora_rssi = LoRa.packetRssi();
        display.clearDisplay();
        // Controle da linha 1 do display
        display.setCursor(0, OLED_LINE1);
        display.println(ValorDate01);
        // Controle da linha 2 do display
        display.setCursor(0, OLED_LINE2);
        display.print("Milis: ");
        display.println(Svtimemili); 
        // Controle da linha 3 do display
        display.setCursor(0, OLED_LINE3);
        display.print("RSSI: ");
        display.print(lora_rssi);
        display.println(" Receiver");
        // Controle da linha 4 do display
        switch (Display_L4) {
          case 1:
        display.setCursor(0, OLED_LINE4);
        display.print("Acel. X = ");
        display.println(FAcX);  // controle do display 4 vem do outro módulo do LoRa
        Display_L4 = Display_L4 + 1;
            break;
          case 2:
        display.setCursor(0, OLED_LINE4);
        display.print("Acel. Y = ");
        display.println(FAcY);
        Display_L4 = Display_L4 + 1;
            break;
          case 3:
        display.setCursor(0, OLED_LINE4);
        display.print("Acel. Z = ");
        display.println(FAcZ);
        Display_L4 = 1;
            break;
        }
        // Controle da linha 5 do display
        display.setCursor(0, OLED_LINE5);
        display.print("Bat = ");
        display.println(FGyX);
        // Controle da linha 6 do display
        display.setCursor(0, OLED_LINE6);
        display.print(num_men_enviada);
        display.println(" mensagem");
        display.display();



         
    }




         /* Verifica se as conexões wi-fi estão ativas 
        Se alguma delas não estiver ativa, a reconexão é feita */
        verify_wifi_connection(); 

    //Reseta o temporizador do watchdog
    esp_task_wdt_reset();



    //delay(30000);
    
}
