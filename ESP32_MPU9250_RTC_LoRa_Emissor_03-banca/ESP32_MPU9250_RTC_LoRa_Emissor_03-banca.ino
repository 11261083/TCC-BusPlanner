/* Programa de teste de leitura do MPU9250, acelerômetro 1,2 e 3 -  Em campo com estruturas no paradigma da IoT*/

/*  Rotinas
 *  1 - Configuração e programação ESP32
 *  2 - Retira o MPU do modo sleep e testa comuincação
 *  3 - Realiza configuração do MPU 9250
 *  4 - Seleciona a escala +/- 2g e +/- 250 graus/s
 *  5 - Relógio de tempo real RTC
 *  6 - Ativa a rotina de comunicação LoRa
 *  6 - Mostra os dados a cada 0,5 seg.
 *  
 */


/* Biblioteca incluidas */
#include <Wire.h>             // Ativa biblioteca com comandos de controle do canal I2C
#include <LoRa.h>             // biblioteca de controle do protocolo LoRa
#include <SPI.h>              // Biblioteca para comunicação SPI
#include <mySD.h>             // Habilita a rotina para gravação no SD
#include "RTClib.h"           // biblioteca de controle do RTC DS3231
#include <Adafruit_GFX.h>     // Biblioteca gráfica, fornecendo um conjunto comum de primitivos gráficos (pontos, linhas, círculos, etc.)
#include <Adafruit_SSD1306.h> // Biblioteca do display oled for monochrome 128x64
#include <esp_task_wdt.h>     //Biblioteca do watchdog

/* Mapeamento de Hardware */
#define MPU_ADR 0x69    // endereça o MPU 9250 com 69
#define OLED_ADDR 0x3C  // endereça o Display oled com 3C

/* Esacala para o acelerômetro  */
#define ACEL_FS_2G  0  // +/- 2g
#define ACEL_FS_4G  1  // +/- 4g
#define ACEL_FS_8G  2  // +/- 8g
#define ACEL_FS_16G 3  // +/- 16g

/* Esacala para o giroscópio  */
#define GIRO_FS_250  0  // +/- 250   graus/seg
#define GIRO_FS_500  1  // +/- 500   graus/seg
#define GIRO_FS_1000 2  // +/- 1000  graus/seg
#define GIRO_FS_2000 3  // +/- 2000  graus/seg

// Registros utilizados do MPU 9250
#define ACCEL_XOUT_H   0x3B
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75
#define PWR_MGMT_1     0x6B
#define CONFIG         0x1A
#define SMPLRT_DIV     0x19
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define INT_ENABLE     0x38
#define INT_STATUS     0x3A

/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18
#define IRQ_PIN_LORA       26
#define HIGH_GAIN_LORA     20     /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */

/* Offset de linhas no display OLED */
#define OLED_LINE1     0
#define OLED_LINE2     10
#define OLED_LINE3     20
#define OLED_LINE4     30
#define OLED_LINE5     40
#define OLED_LINE6     50

/* Configuração da resolucao do display (este modulo possui display 128x64) */
#define SCREEN_WIDTH    128 
#define SCREEN_HEIGHT   64

/* C   */
# define ADC_MAX 4095.0   // Resolução do canal analógico
const int pino = 13;      // pino 13 do GPIO13
float Batvalor = 0;       // Valor da entrada analógica em tensão correspondente a tensão da bateria

//Tempo para ativar o reset do watchdog timer
#define WDT_TIMEOUT 30     // tempo em segundos para o watchdog timer

/* Objeto do display */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 16);

// Habilita RTC DS3231
RTC_DS3231 rtc;

/* typedefs dados estruturado para transmissão via canal LoRa*/
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


/* Variáveis e constantes globais */

//Variaveis para armazenar valores dos sensores
float FAcX,FAcY,FAcZ,FTmp,FGyX,FGyY,FGyZ;  // Valores com fator de correção aplicado

float giro_res, acel_res;   // resolução, depende da escala
int16_t bias_int[6];        // erro acel:x,y,z e giro:x,y,z
float   bias_float[6];      // erro em g e graus/s

double spxyzac, spxyzdc;        // Calcula do valor de x²+y²+z² antes e depois da correção do erro
double sqrtxyzac, sqrtxyzdc;    // Calcula do valor da raiz quadrada de x²+y²+z² antes e depois da correção do erro
float csqrtxyzac, csqrtxyzdc;   // Converte para float o valor da raiz quadrada de x²+y²+z² antes e depois da correção do erro

// Dados do  RTC
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; // Vetor para usar no RTC com o nome dos dias da semana
String ValorDate01, ValorDate02, ValorDate03;      // Valor string para mostrar no formato ISO 8601

unsigned long MaskMiliseg = 1023;  // Máscara para retirar do valor do Milli() de 0 a 1023
unsigned long VMaskMiliseg;  // Valor dos milesegundos após a máscara
int vtimeYYYY, vtimeMM, vtimeDD, vtimehh, vtimemm, vtimess;
unsigned long vtimemili;
String SvtimeYYYY, SvtimeMM, SvtimeDD, Svtimehh, Svtimemm, Svtimess;
String Svtimemili;
int vcdisplay; // valor do controle do display

int Display_L4 = 1;                   // Controla o que vai mostra na linha 2 do display
int contador = 0;

// Controle do número de leituras
int var01 = 0, var02 = 0;
int count = 0;
int count01 = 1;                     // Valor para o teste de Watchdog Timer
int valresetwdt = 0;                 // Reseta valor para teste de WDT

// Controle de recebimento de dados via monitor
int incomingByte = 0; // variável para o dado recebido
char myString[16];




// --- Variáveis Globais a função da média móvel ---
const int   ams = 45;             // número de amostras para a média
float       amsf = 45.0;          // número de amostras para a média em ponto flutuante
float       Valor_axs;            // recebe o valor de leitura do eixo X do MPU9250 no ESP32
float       Valor_ays;            // recebe o valor de leitura do eixo Y do MPU9250 no ESP32          
float       Valor_azs;            // recebe o valor de leitura do eixo Z do MPU9250 no ESP32
float       numbers_axs[ams];     // vetor com os valores para média móvel do eixo X
float       numbers_ays[ams];     // vetor com os valores para média móvel do eixo Y
float       numbers_azs[ams];     // vetor com os valores para média móvel do eixo Z
double      media_axs;            // faz a média móvel do eixo X do MPU9250 no ESP32
double      media_ays;            // faz a média móvel do eixo Y do MPU9250 no ESP32         
double      media_azs;            // faz a média móvel do eixo Z do MPU9250 no ESP322

// Ativa transmissão de dados
static unsigned long ativaLoRa;               // Ativa a transmissão dos dados via LoRa

/* Chamada de Rotinas locais*/
bool init_comunicacao_lora(void);
void envia_informacoes_lora(float FAcX, float FAcY, float FAcZ, float FTmp, float FGyX, float FGyY, float FGyZ,long contador, int vtimeYYYY, int vtimeMM, int vtimeDD, int vtimehh, int vtimemm, int vtimess, unsigned long vtimemili, int vcdisplay);


/* Funcao: inicia comunicação com chip LoRa
   Parametros: nenhum
   Retorno: true: comunicacao ok
            false: falha na comunicacao
*/
bool init_comunicacao_lora(void)
{
  bool status_init = false;
  Serial.println("[LoRa Sender] Tentando iniciar comunicacao com o radio LoRa...");
  SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, SS_PIN_LORA);
  LoRa.setPins(SS_PIN_LORA, RESET_PIN_LORA, IRQ_PIN_LORA);

  if (!LoRa.begin(BAND))
  {
    Serial.println("[LoRa Sender] Comunicacao com o radio LoRa falhou. Nova tentativa em 1 segundo...");
    delay(1000);
    status_init = false;
  }
  else
  {
    /* Configura o ganho do receptor LoRa para 20dBm, o maior ganho possível (visando maior alcance possível) */
    LoRa.setTxPower(HIGH_GAIN_LORA);
    Serial.println("[LoRa Sender] Comunicacao com o radio LoRa ok");
    status_init = true;
  }

  return status_init;
}



void envia_informacoes_lora(float FAcX, float FAcY, float FAcZ, float FTmp, float FGyX, float FGyY, float FGyZ,long contador, int vtimeYYYY, int vtimeMM, int vtimeDD, int vtimehh, int vtimemm, int vtimess, unsigned long vtimemili, int vcdisplay)
{
    TDadosLora dados_lora;

    dados_lora.acelX = FAcX;
    dados_lora.acelY = FAcY;
    dados_lora.acelZ = FAcZ;
    dados_lora.temp = FTmp;
    dados_lora.girX = FGyX;
    dados_lora.girY = FGyY;
    dados_lora.girZ = FGyZ;
    dados_lora.num_men = contador;
    dados_lora.timeYYYY = vtimeYYYY;
    dados_lora.timeMM = vtimeMM;
    dados_lora.timeDD = vtimeDD;
    dados_lora.timehh = vtimehh;
    dados_lora.timemm = vtimemm;
    dados_lora.timess = vtimess;
    dados_lora.timemili = vtimemili;
    dados_lora.cdisplay = vcdisplay;
    LoRa.beginPacket();
    LoRa.write((unsigned char *)&dados_lora, sizeof(TDadosLora));
    LoRa.endPacket();
}



// Rotina que escreve um dado em um registro do MPU 9250 no endereço ADR
void i2c_wr(uint8_t adr, uint8_t reg, uint8_t dado) {
  Wire.begin (4, 15);           // sda= GPIO_04 /scl= GPIO_15 // Inicializa canal I2C no ESP32
  Wire.beginTransmission(adr); // Inicializa buffer de transmissão TX
  Wire.write(reg);             // No buffer escreve o registro
  Wire.write(dado);                 // No buffer escreve o dado
  Wire.endTransmission();      // Envia buffer e finaliza a transmissão
}

// Rotina que lê um dado em um registro do MPU 9250 no endereço ADR
uint8_t i2c_rd(uint8_t adr, uint8_t reg) {
  Wire.begin (4, 15);           // sda= GPIO_04 /scl= GPIO_15 // Inicializa canal I2C no ESP32
  Wire.beginTransmission(adr);            // Inicializa buffer de transmissão TX
  Wire.write(reg);                        // No buffer escreve o registro
  Wire.endTransmission(false);            // Envia buffer e e gera um restart
  Wire.requestFrom(adr, (uint8_t) 1);     // Lê um byte no enderço do MPU 9250
  return Wire.read();                     //recebe o dado lido do MPU 9250 
}

// Rotina que lê um dado em um conjunto de registros do MPU 9250 no endereço ADR
void i2c_rd_rep(uint8_t adr, uint8_t reg, uint8_t qtd, uint8_t *vet) {
  uint8_t cont;                           // declara contador das repetições
  Wire.begin (4, 15);           // sda= GPIO_04 /scl= GPIO_15 // Inicializa canal I2C no ESP32
  Wire.beginTransmission(adr);            // Inicializa buffer de transmissão TX
  Wire.write(reg);                        // No buffer escreve o registro
  Wire.endTransmission(false);            // Envia buffer e e gera um restart
  Wire.requestFrom(adr, qtd);             // Lê um bytess no enderço do MPU 9250
  for (cont = 0; cont < qtd; cont++);     // Contador das repetições
    vet[cont]=Wire.read();                //recebe o dado lido do MPU 9250 um por vez
}


// Rotina que acorda o MPU 9250 e programa uso de relógio Giro X
void MPU_acorda (void) {
  Serial.println("Acorda o dispositivo MPU 9250");
  i2c_wr(MPU_ADR, PWR_MGMT_1, 1);
}

//Rotina para voltar o valor lido no registrador WHO_AM_I
uint8_t MPU_whoami(void) {
  return i2c_rd(MPU_ADR, WHO_AM_I);
}

// Rotina para colocar o MPU 9250 num estado conhecido
void MPU_inicializar(void) {
  Serial.println("Inicializa MPU 9250, desperta e coloca relógio PLL do Giroscópio X"); 
  // Desperta MPU 9250, e relógio = PLL do Giro - X
  i2c_wr(MPU_ADR, PWR_MGMT_1, 0x01);
  delay(200); // Espera PLL estabilizar

  // Taxa = 1 Khz, banda: Acel=44 Hz e Giro=42 Hz
  i2c_wr(MPU_ADR, CONFIG, 0x03);

  // Taxa de amostragem = taxa/(1+SMPLRT_DIV)
  i2c_wr(MPU_ADR, SMPLRT_DIV, 0x04);  //Taxa = 200 Hz
}

// Rotina que seleciona escalas e calcula resolução
// Atualiza giro_res e acel_res (globais) 
void MPU_escalas(uint8_t gfs, uint8_t afs) {
  i2c_wr(MPU_ADR, GYRO_CONFIG, gfs << 3); // configura FS do Giroscópio
  i2c_wr(MPU_ADR, ACCEL_CONFIG, afs << 3); // configura FS do Acelerômetro

  // Calcula resolução do giroscópio
  switch (gfs) {
    case GIRO_FS_250:  giro_res =  250.0 / 32768.0; break;
    case GIRO_FS_500:  giro_res =  500.0 / 32768.0; break;
    case GIRO_FS_1000: giro_res = 1000.0 / 32768.0; break;
    case GIRO_FS_2000: giro_res = 2000.0 / 32768.0; break;
  }

  // Calcula resolução do acelerômetro
  switch (afs) {
    case ACEL_FS_2G:  acel_res =  2.0 / 32768.0; break;
    case ACEL_FS_4G:  acel_res =  4.0 / 32768.0; break;
    case ACEL_FS_8G:  acel_res =  8.0 / 32768.0; break;
    case ACEL_FS_16G: acel_res = 16.0 / 32768.0; break;
  }
}


  
  // Rotina que imprime mensagens de erro e trava execução
  void MPU_erro(int  erro) {
    switch (erro) {
      case 1: 
        Serial.println("MPU 9250 não responde ao I2C!"); 
        Serial.print("Valor do MPU_whoami = "); Serial.println(MPU_whoami());    // Retorna o valor lido do WHO_AM_I
        break;
      case 2: Serial.println("MPU 9250 falhou no self-test"); break;
      case 3: Serial.println("Para a rotina"); break;      
    }
    for (;;);
  }

// Rotina da média móvel para os eixos X,Y e Z, do acelerômetro MPU 9250
double moving_average_axs()
{
   //desloca os elementos do vetor de média móvel
   for(int i= ams-1; i>0; i--) numbers_axs[i] = numbers_axs[i-1];

   numbers_axs[0] = Valor_axs;      //posição inicial do vetor recebe a leitura axs
   double acc_axs = 0.0;              //acumulador para somar os pontos da média móvel

   for(int i=0; i<ams; i++) acc_axs += numbers_axs[i]; //faz a somatória do número de pontos

   return acc_axs/amsf;             //retorna a média móvel do eixo X

} //end moving_average_axs

double moving_average_ays()
{
   //desloca os elementos do vetor de média móvel
   for(int i= ams-1; i>0; i--) numbers_ays[i] = numbers_ays[i-1];

   numbers_ays[0] = Valor_ays;      //posição inicial do vetor recebe a leitura ays
   double acc_ays = 0.0;              //acumulador para somar os pontos da média móvel

   for(int i=0; i<ams; i++) acc_ays += numbers_ays[i]; //faz a somatória do número de pontos

   return acc_ays/amsf;             //retorna a média móvel do eixo Y

} //end moving_average_ays

double moving_average_azs()
{
   //desloca os elementos do vetor de média móvel
   for(int i= ams-1; i>0; i--) numbers_azs[i] = numbers_azs[i-1];

   numbers_azs[0] = Valor_azs;      //posição inicial do vetor recebe a leitura azs
   double acc_azs = 0.0;              //acumulador para somar os pontos da média móvel

   for(int i=0; i<ams; i++) acc_azs += numbers_azs[i]; //faz a somatória do número de pontos

   return acc_azs/amsf;             //retorna a média móvel do eixo Z

} //end moving_average_azs


 

// Rotinas de rodam um vez
void setup() {

  // Faz a primeira inicialização da variável ativaLoRa
  ativaLoRa = millis();  


  // Inicia a serial com velocidade de 115200
  Serial.begin (115200);  // Velocidade de transmissão do canal do monitor

  delay(1000);    // Espera 1 segundo da inicialização do canal e envia mensagem - "Starting..."
  // Exibe na serial "Starting..."
  Serial.println("Starting...");


  // Inicializa canal I2C no ESP32
  Wire.begin (4, 15);   // sda= GPIO_04 /scl= GPIO_15
  Wire.beginTransmission(MPU_ADR);
  Wire.write(0x6B); 
  //Inicializa o MPU-9250
  Wire.write(0); 
  Wire.endTransmission(true);  
 
  MPU_acorda(); // Tira do sleep o MPU 9250

  // Testa comunicação com o MPU 9250
  if (MPU_whoami() == 0x71) {
    Serial.println("MPU 9250 Respondendo!");
    Serial.print("Valor do MPU_whoami = "); Serial.println(MPU_whoami());  // Retorna o valor lido do WHO_AM_I
  }
    else MPU_erro(1);

  // Realiza a calibração conforme a escala no caso ACEL_FS_2G e GIRO_FS_250
  //MPU_calibra(bias_int, bias_float);

  // Realiza configuração básica do MPU 9250
  MPU_inicializar();
  MPU_escalas(GIRO_FS_250, ACEL_FS_2G);
  Serial.println("Configurado a escala do MPU 9250 com Giroscópio 250 e o Acelerômetro com 2g");

   // Inicializa o módulo RTC - Relógio de tempo real
   if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      abort();
    }

   if (rtc.lostPower()) { // Configura data e hora
      Serial.println("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      // rtc.adjust(DateTime(2022, 1, 31, 16, 58, 0));
    }

/*
      // Força data e hora no módulo DS3231
      Serial.println("---------------------");
      Serial.println("Ajuste de data e hora");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtc.adjust(DateTime(2022, 3, 9, 17, 25, 0));
      Serial.println("---------------------");

*/


    // inicializa display OLED 
    Wire.begin(4, 15);

    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) 
        Serial.println("Display OLED: falha ao inicializar");        
    else
    {
        Serial.println("Display OLED: inicializacao ok");            
        
        // Limpa display e configura tamanho de fonte 
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
    }


    /* Tenta, até obter sucesso, comunicacao com o chip LoRa */
    while (init_comunicacao_lora() == false);

    //Habilita o watchdog configurando o timeout para o valor de WDT_TIMEOUT segundos
    esp_task_wdt_init(WDT_TIMEOUT, true);   //comando de habilita  o ESP32 para resetar
    esp_task_wdt_add(NULL);                 //Adicionar rotina atual ao relógio WDT




// Final da rotina Setup
    
}

void loop() {
  
  // Variáveis
  uint8_t aux[14];                             // Auxiliar para receber os dados via I2C do MPU 9250 do giroscópio e acelerômetro
  int16_t axi,ayi,azi,gxi,gyi,gzi,tpi;         // Valor inteiros do giroscópio e acelerômetro
  int16_t axie,ayie,azie,gxie,gyie,gzie,tpie;  // Valor inteiros do giroscópio e acelerômetro  
  float ax,ay,az,gx,gy,gz,tp;                  // Valor em ponto flutuante do giroscópio e acelerômetro
  float axs,ays,azs,gxs,gys,gzs,tps;           // Valor em ponto flutuante do giroscópio e acelerômetro sem correção de erro
  float C1FAcX,C1FAcY,C1FAcZ,C1FTmp,C1FGyX,C1FGyY,C1FGyZ; // Converte o valor inteiro para float
  float C2FAcX,C2FAcY,C2FAcZ,C2FTmp,C2FGyX,C2FGyY,C2FGyZ; // Converte o valor inteiro para float




  // apenas responde quando dados são recebidos:
  if (Serial.available() > 0) {     // Esta fora de operação
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
    //valresetwdt = incomingByte;
    }
  }

  // Valor da bateria recebido no pino 13 GPIO13
  Batvalor = analogRead(pino) * (4.1 / ADC_MAX);
  //Serial.println(analogRead(pino));
  //Serial.println(Batvalor); 
  //Serial.println("______"); 

  //Serial.print("valresetwdt = "); Serial.println(valresetwdt);

  // Teste de Watchdog Timer
  //Fica preso no loop enquanto o botão estiver pressionado
  while (valresetwdt == 12) {
    Serial.print("Teste de WatchDog Timer... ");
    Serial.print("Contagem até 20 - Valor atual = ");
    Serial.println(count);
    count++;
    delay(1000);
  }

   
  // Faz o controle do valor das amostras e quando chega ao valor 32.367, zera o valor va01
  if (var01 < 32,767) {
    var01 = var01 + 1; // Acresce 1 a variável var01
  }
  else {
     var01 = 0;
  }

  Wire.beginTransmission(MPU_ADR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU_ADR,14,true);
 
  //Armazena o valor dos sensores nas variaveis correspondentes em valor interiro
  axi=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  ayi=Wire.read()<<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  azi=Wire.read()<<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tpi=Wire.read()<<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gxi=Wire.read()<<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyi=Wire.read()<<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gzi=Wire.read()<<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


  // Transforma Int em Float
  C2FAcX = float(axi);
  C2FAcY = float(ayi);
  C2FAcZ = float(azi);
  C2FGyX = float(gxi);
  C2FGyY = float(gyi);
  C2FGyZ = float(gzi);

  axs = C2FAcX * acel_res;  // Calcula o valor do acelerômetro X - Acel X
  ays = C2FAcY * acel_res;  // Calcula o valor do acelerômetro Y - Acel Y
  azs = C2FAcZ * acel_res;  // Calcula o valor do acelerômetro Z - Acel Z 
  tps = (float) tpi / 340.00 + 18,53;            // Calcula o valor da temperatura - 36,53 para 18,53
  gxs = C2FGyX * giro_res;  // Calcula o valor do giroscópio X - Giro X
  gys = C2FGyY * giro_res;  // Calcula o valor do giroscópio Y - Giro Y
  gzs = C2FGyZ * giro_res;  // Calcula o valor do giroscópio Z - Giro Z

  // Valor da méida média dos eixo X,Y e Z
  Valor_axs = axs;
  media_axs = moving_average_axs();
  Valor_ays = ays;
  media_ays = moving_average_ays();
  Valor_azs = azs;
  media_azs = moving_average_azs();

  //Mostra os valores no monitor na serial
  Serial.print("Acel. X = "); Serial.print(axs,3);
  Serial.print(" | Acel. mdX = "); Serial.print(media_axs,3);        
  Serial.print(" | Y = "); Serial.print(ays,3);
  Serial.print(" | Acel. mdY = "); Serial.print(media_ays,3);        
  Serial.print(" | Z = "); Serial.print(azs,3);
  Serial.print(" | Acel. mdZ = "); Serial.println(media_azs,3); 

  FAcX = media_axs;  // Calcula o valor da média móvel do acelerômetro X - Acel X
  FAcY = media_ays;  // Calcula o valor da média móvel do acelerômetro Y - Acel Y
  FAcZ = media_azs;  // Calcula o valor da média móvel do acelerômetro Z - Acel Z 
  FTmp = (float) tpi / 340.00 + 36,53;            // Calcula o valor da temperatura
  FGyX = Batvalor;  // Enviando o valor da bateria
  FGyY = 0.00;      // Enviando valor 0.00
  FGyZ = 0.00;      // Enviando valor 0.00
/*
  FGyX = C2FGyX * giro_res;  // Calcula o valor do giroscópio X - Giro X
  FGyY = C2FGyY * giro_res;  // Calcula o valor do giroscópio Y - Giro Y
  FGyZ = C2FGyZ * giro_res;  // Calcula o valor do giroscópio Z - Giro Z
*/
   

/*
  //Mostra os valores no monitor na serial
  Serial.print("Acel. X = "); Serial.print(ax,3);
  Serial.print(" | Y = "); Serial.print(ay,3);
  Serial.print(" | Z = "); Serial.print(az,3);
  Serial.print(" | Gir. X = "); Serial.print(gx,3);
  Serial.print(" | Y = "); Serial.print(gy,3);
  Serial.print(" | Z = "); Serial.print(gz,3);
  Serial.print(" | Temp = "); Serial.println(tp,2);
*/
  //Mostra os valores no monitor na serial
  //Serial.print("Acel. X = "); Serial.print(FAcX,3);
  //Serial.print(" | Y = "); Serial.print(FAcY,3);
  //Serial.print(" | Z = "); Serial.print(FAcZ,3);
  //Serial.print(" | Gir. X = "); Serial.print(gxs,3);
  //Serial.print(" | Y = "); Serial.print(gys,3);
  //Serial.print(" | Z = "); Serial.print(gzs,3);
  //Serial.print(" | Temp = "); Serial.println(tps,2);
/*
  // Calcula do valor de x²+y²+z² antes e depois da correção do erro
  spxyzac = sq(ax) + sq(ay) + sq(az);
  spxyzdc = sq(axs) + sq(ays) + sq(azs);

  // Calcula do valor da raiz quadrada de x²+y²+z² antes e depois da correção do erro
  sqrtxyzac = sqrt(spxyzac);
  sqrtxyzdc = sqrt(spxyzdc);

  // Converte para float o valor da raiz quadrada de x²+y²+z² antes e depois da correção do erro
  csqrtxyzac = float(sqrtxyzac);
  csqrtxyzdc = float(sqrtxyzdc);

  //Mostra os valores no monitor na serial do resultado √x^2+y^2+z^2 para verificar erro no acelerômetro
  //Serial.print("Sem erro √x^2+y^2+z^2 = "); Serial.print(csqrtxyzac,4); Serial.println(" g");
  Serial.print(" - Com erro √x^2+y^2+z^2 = "); Serial.print(csqrtxyzdc,4); Serial.print(" g");
  Serial.print(" - Amostra "); Serial.print(var01); Serial.print(" - ");
*/
   // Pega valor do RTC agora
   DateTime now = rtc.now();
    // Inicia o controle dos milisegundos
    VMaskMiliseg = millis() % 1000;    // faz máscara para tirar valor até 1023 e colocar no timestamp iso 8601
/*
   Serial.print(" Valor Milis "); Serial.print( VMaskMiliseg); Serial.print(" - ");

   // Mostra na serial o valor agora do RTC DS3231
   Serial.print(now.year(), DEC);
   Serial.print('/');
   Serial.print(now.month(), DEC);
   Serial.print('/');
   Serial.print(now.day(), DEC);
   Serial.print(" (");
   Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
   Serial.print(") ");
   Serial.print(now.hour(), DEC);
   Serial.print(':');
   Serial.print(now.minute(), DEC);
   Serial.print(':');
   Serial.print(now.second(), DEC);
   Serial.println();
*/
    vtimeYYYY = now.year();
    vtimeMM = now.month();
    vtimeDD = now.day();
    vtimehh = now.hour();
    vtimemm = now.minute();
    vtimess = now.second();
    vtimemili = VMaskMiliseg;
    vcdisplay = Display_L4;

    // Transforma o valor e corrigi de inteiro em string para envio pelo canal LoRa
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

    //buffer can be defined using following combinations:
    //hh - the hour with a leading zero (00 to 23)
    //mm - the minute with a leading zero (00 to 59)
    //ss - the whole second with a leading zero where applicable (00 to 59)
    //YYYY - the year as four digit number
    //YY - the year as two digit number (00-99)
    //MM - the month as number with a leading zero (01-12)
    //MMM - the abbreviated English month name ('Jan' to 'Dec')
    //DD - the day as number with a leading zero (01 to 31)
    //DDD - the abbreviated English day name ('Mon' to 'Sun')

    // Monta Ano/Mês/Dia/hora/minuto/seundo para a string "YYYY-MM-DDThh:mm:ss.mmm-03:00"
    char buf1[] = "YYYY-MM-DDThh:mm:ss";

    // Mostra valor do RTC DS3231 no formato ISO 8601
    ValorDate01 = now.toString(buf1);
    ValorDate02 = ".";
    ValorDate03 = VMaskMiliseg;  // ValorDate03 = ValorMiliseg; ainda não tem uma lógica boa para o controle de milisegundos
    ValorDate01 += ValorDate02;
    ValorDate01 += ValorDate03;
    ValorDate01 += "+00:00";
    //Serial.println(ValorDate01);
    String VTimeStamp(ValorDate01);    // transfere o valor composto de data e hora no formato ISO 8601 para a tag de envio do LoRa

    //Serial.print("Passou aqui - ");
    //Serial.println(contador); 




  delay(500);  // Espera 0,5 segundos para reiniciar o loop

  //Serial.println("passou aqui no final do programa");

  // Controle e apresentação de dados no display

      
    // Enviados dados para o display
    display.clearDisplay();
    // Controle da linha 1 do display
    display.setCursor(0, OLED_LINE1);
    display.println(now.toString(buf1));
    // Controle da linha 2 do display
    display.setCursor(0, OLED_LINE2);
    display.print("Milis: ");
    display.println(Svtimemili);    
    // Controle da linha 3 do display   
    display.setCursor(0, OLED_LINE3);
    display.print("Temperatura: ");
    display.println(tps);  
    // Controle da linha 4 do display
    switch (Display_L4) {
      case 1:
    display.setCursor(0, OLED_LINE4);
    display.print("Acel. X = ");
    display.println(FAcX);
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
    display.println(Batvalor);
    // Controle da linha 6 do display
    display.setCursor(0, OLED_LINE6);
    display.print(contador);
    display.println(" mensagem"); 
    display.display();

    if ((millis() - ativaLoRa) > 2200) {

    /* chama rotina para envio de mensagem via Lora */
    envia_informacoes_lora(FAcX, FAcY, FAcZ, FTmp, FGyX, FGyY, FGyZ, contador,  vtimeYYYY, vtimeMM, vtimeDD, vtimehh, vtimemm, vtimess, vtimemili, vcdisplay);

    contador++;
    //delay(1000);     // Espera 5 segundo para fazer outra leitura
    ativaLoRa = millis();
    Serial.println("Passou aqui - Transmitiu");
    }
  
  //if (var01 >= 1000) {
  //  MPU_erro(3); //Para o programa
  //}

  //Reseta o temporizador do watchdog
  esp_task_wdt_reset();

}
