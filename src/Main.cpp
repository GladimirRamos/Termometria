#include <Arduino.h>
/*************************************************************
  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/
/* 
   Firmware para o  hardware KC 868-A6 v1.3 ou compatível
     disponivel na pasta oculta do projeto ".pio/build/esp32doit-devkit-v1/firmware.bin"

 - no template do Blynk Console setar para sincronizar hora start, hora stop (V40), modo (V49), etc...
 - Blynk.syncVirtual (V40, V49);     // sincroniza datastream agendamento e modo de operação
 - em Settings.h adicionado:
    46  #define BOARD_LED_PIN               2
    47  #define BOARD_LED_INVERSE           false
    48  #define BOARD_LED_BRIGHTNESS        128

 - em ResetButton.h adicionado:
     7  int flagSetRTC = 0;
    33  DEBUG_PRINT("Botão pressionado rapidamente, o relógio será recalibrado pelo NTP!");
    34  flagSetRTC = 1;

 - em Settings.h comentada linha 41 - warning led

 - RTC: ajuste automático via NTP com um click rápido no GPOI0 ou em horario definido
*/
// Sensor MODBUS 4x1      - Slave ID 32
// Sensor MODBUS CWT-TH04 - Slave ID 01...

#define Slave_ID_EXT             32 // sensor 4x1
#define Slave_ID_01               6 // sensor 1
#define Slave_ID_02               5 // sensor 2
#define Slave_ID_03               3 // sensor 3
#define Slave_ID_04               4 // sensor 4...

#define BLYNK_TEMPLATE_ID        "TMPL2x5zWdfN7"
#define BLYNK_TEMPLATE_NAME      "Temometria Silos"
#define BLYNK_FIRMWARE_VERSION   "0.2.5"
//#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG   
//#define APP_DEBUG

#define USE_ESP32_DEV_MODULE
#include "time.h"                               // biblioteca para ajuste e controle do tempo (NTP)
#include "BlynkEdgent.h"
#include "HardwareSerial.h"
#include "SPI.h"
#include "RTClib.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Wire.h"
#include <Preferences.h>                        // biblioteca para salvar dados na NVS
#include "HeartBeat.h"
#include "uptime_formatter.h"                   // imprimir uptime do sistema

unsigned char output_PLC = 0b11111111;          // variavel para controle dos relês de saída - iniciam tudos desligados (0 liga)

RTC_DS1307           RTC;                       // RTC DS1307 - Address I2C (0x68)
Preferences  preferences;
unsigned int  counterRST;                       // contador de reset's
bool    sendBlynk = true;

  // ATENÇÃO AO WATCHDOG; AJUSTAR COM MARGEM DE ACORDO COM O TEMPO ABAIXO !!!
int tempoStart  =     10; // tempo de espera em segundos para o inicio do sistema a cada reset minimo 10 segundos para RTC start
int timerON     =      0; // usado para mostrar só uma vez a cada reset a tela inicial no diplay com a logomarca
int minAtualiza =      0; // usado para enviar dados ao servidor a cada 15 minutos, começa a enviar no minAtualiza
int BotaoRESET;           // BotaoRESET = Virtual do APP

char monthString[25] = {"010203040506070809101112"};                // modelo 1
int  monthIndex[122] = {0,2,4,8,10,12,14,16,17,18,20,22};

//char monthString[37]= {"JANFEVMARABRMAIJUNJULAGOSETOUTNOVDEZ"};   // modelo 2
//int  monthIndex[122] ={0,3,6,9,12,15,18,21,24,27,30,33};
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define I2C_SCL 15                             // SCL (ESP32 default GPIO 22)
#define I2C_SDA 4                              // SDA (ESP32 default GPIO 21)

#define rearme_PIN 12                          // GPIO12 pino para rearme do quadro
//#define CANAL_DAC0 25                        // Definição do canal DAC GPIO 25 - Saida DAC_01 da KC868-A6 
//#define CANAL_DAC1 26                        // Definição do canal DAC GPIO 26 - Saida DAC_02 da KC868-A6

Adafruit_SSD1306 display(128, 64, &Wire, -1);

//-------------------------------------  NTP Server time ----------------------------------------------
const char* ntpServer = "br.pool.ntp.org";      // "pool.ntp.org"; "a.st1.ntp.br";
const long  gmtOffset_sec = -10800;             // -14400; Fuso horário em segundos (-03h = -10800 seg)
const int   daylightOffset_sec = 0;             // ajuste em segundos do horario de verão

//int flagSetRTC = 0;                           // usada para enviar para a rotina de ajuste automatico do RTC
char* ntpTime;
char* RTC_Time;
//-----------------------------------------------------------------------------------------------------

int currentSec;
int currentMin;
int currentHour;
int currentDay;
int currentMonth;
int currentYear;
int currentSecDAY;
String currentWday    = "-1";

// ******************************************** //
//            Controle do Silo 1
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S1     = "------";               
String old_StrModo_S1 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON1;
int min_PGM_ON1;
int sec_PGM_ON1;
int HoraLiga1;
int HoraDESliga1;
int hora_PGM_OFF1;
int min_PGM_OFF1;
int sec_PGM_OFF1;
int HoraOn_PGMMem1;
int HoraOff_PGMMem1;
int HoraLigaPGM1;
int HoraDESLigaPGM1;
String DiaSemPGM1;
int WdayON1;
// vinculo com app
int forcaLiga1 = 0;
int forcaDESLiga1 = 0;
// uso geral
int varModoOper1;
int cicloON_1 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_1 = 0; 
int timer_Motor1 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor1;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor1;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade1;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao1 = 60; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //
//            Controle do Silo 2
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S2     = "------";               
String old_StrModo_S2 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON2;
int min_PGM_ON2;
int sec_PGM_ON2;
int HoraLiga2;
int HoraDESliga2;
int hora_PGM_OFF2;
int min_PGM_OFF2;
int sec_PGM_OFF2;
int HoraOn_PGMMem2;
int HoraOff_PGMMem2;
int HoraLigaPGM2;
int HoraDESLigaPGM2;
String DiaSemPGM2;
int WdayON2;
// vinculo com app
int forcaLiga2 = 0;
int forcaDESLiga2 = 0;
// uso geral
int varModoOper2;
int cicloON_2 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_2 = 0; 
int timer_Motor2 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor2;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor2;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade2;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao2 = 120; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //
//            Controle do Silo 3
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S3     = "------";               
String old_StrModo_S3 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON3;
int min_PGM_ON3;
int sec_PGM_ON3;
int HoraLiga3;
int HoraDESliga3;
int hora_PGM_OFF3;
int min_PGM_OFF3;
int sec_PGM_OFF3;
int HoraOn_PGMMem3;
int HoraOff_PGMMem3;
int HoraLigaPGM3;
int HoraDESLigaPGM3;
String DiaSemPGM3;
int WdayON3;
// vinculo com app
int forcaLiga3 = 0;
int forcaDESLiga3 = 0;
// uso geral
int varModoOper3;
int cicloON_3 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_3 = 0; 
int timer_Motor3 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor3;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor3;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade3;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao3 = 180; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //

// ----------------------------------- SETUP Watchdog e ResetReason ----------------------------------- 
#include "soc/rtc_wdt.h"
#define WDT_TIMEOUT   120000               // WDT miliseconds (max 120000 segundo manual Espressif)
//#define Heartbeat_PIN 13                 // monitoração de "batimentos" para o watchdog de hardware

//Converte esp_reset_reason para string
const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN RESET";
    case ESP_RST_POWERON:   return "POWER ON RESET";        //Power on or RST pin toggled
    case ESP_RST_EXT:       return "EXTERN PIN RESET";      //External pin - not applicable for ESP32
    case ESP_RST_SW:        return "SOFTWARE REBOOT";       //esp_restart()
    case ESP_RST_PANIC:     return "CRASH RESET";           //Exception/panic
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";    //Interrupt watchdog (software or hardware)
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";         //Task watchdog
    case ESP_RST_WDT:       return "RTC WATCHDOG";          //Other watchdog (RTC)
    case ESP_RST_DEEPSLEEP: return "SLEEP RESET";           //Reset after exiting deep sleep mode
    case ESP_RST_BROWNOUT:  return "BROWNOUT RESET";        //Brownout reset (software or hardware)
    case ESP_RST_SDIO:      return "RESET OVER SDIO";       //Reset over SDIO
    default:                return "";
  }
}

void ResetReason(void) {
  esp_reset_reason_t r = esp_reset_reason();
  //if (r == ESP_RST_POWERON) {delay(100);}                 // se habilitar espera o monitor serial inicializar
  Serial.printf("Reset reason:     %i - %s\r\n\r\n", r, resetReasonName(r));
  // se reiniciar por POWER ON RESET vai iniciar o app em operacao no modo Manual
  /*
  if (r == 1){
    varModoOper1 = 0;
    varModoOper2 = 0;
    varModoOper3 = 0;
    preferences.begin  ("my-app", false);                       // inicia 
    preferences.putUInt("varModoOper1", varModoOper1);          // grava na NVS
    preferences.putUInt("varModoOper2", varModoOper2);
    preferences.putUInt("varModoOper3", varModoOper3);
    preferences.end();
    }
    */
}
// -----------------------------------  Fim Watchdog e ResetReason ----------------------------------- 

#include <ModbusMaster.h>
//ModbusMaster ExtSensor;               // Sensor externo 4x1 (antigo node2)
ModbusMaster Sensor_01;               // Sensor 01
ModbusMaster Sensor_02;               // Sensor 02
ModbusMaster Sensor_03;               // Sensor 03
ModbusMaster Sensor_04;               // Sensor 04
//ModbusMaster ExtSensorCWT;          // Sensor externo CWT

int TempExt    = 100,
    UmiExt     = 100;  
//int PresaoExt  = 0,
//int LuxExt     = 0;  

int Umi_01     = 100,
    Temp_01    = 100,
    Umi_02     = 100,
    Temp_02    = 100,
    Umi_03     = 100,
    Temp_03    = 100,
    Umi_04     = 100,
    Temp_04    = 100;


#define samples             6           // quantidade de amostras para cálculo da média móvel 
int matriz_samples [samples];           // vetor para o deslocamento dos valores da média móvel
                                        // no setup inicia elementos do vetor em "100% de Umidade"
int average_UmiExt     = 100;           // recebe o valor de média móvel (average), inicia em 100%

// ------ protótipo de funções ------
//void heartBeat(void);
void timerStart(void);
void setRTC(void);
void sendLogReset(void);
void ComandoOutput(void);
void timerButtonAPP(void);
void getDataHora(void);
void MODBUS_Sensor(void);
long moving_average();
// ------         FIM          ------

void Main2(){
  unsigned long tempo_start = millis();      // usado no final para imprimir o tempo de execução dessa rotina

  rtc_wdt_feed();                             // a cada "loop executado" reseta o contador do watchdog
    // ------ testa o estado das entradas ------
  Wire.requestFrom(0x22, 1);                  // requisita do endereço 0x22      
    unsigned char inputPCF = Wire.read();     // carrega o valor lido em inputPCF, variavel de 8 bits
    Wire.endTransmission();                   // transfere o valor para as variaveis de sinalizacao e controle
       statusMotor1 = inputPCF & (1<<0);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_1 = inputPCF & (1<<1);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app
       statusMotor2 = inputPCF & (1<<2);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_2 = inputPCF & (1<<3);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app
       statusMotor3 = inputPCF & (1<<4);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_3 = inputPCF & (1<<5);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app

  DateTime now = RTC.now();
  currentSec   = now.second();
  currentMin   = now.minute();
  currentHour  = now.hour();
  currentDay   = now.day();
  currentMonth = now.month();
  currentYear  = now.year();
  int RTCWday  = now.dayOfTheWeek();          // Day of week do RTC: 0 (Domingo) até 6 (Sábado)
                                              //              Blynk: 1 (Segunda) até 7 (Domingo)
  if (RTCWday == 0) {currentWday = 7;}        // compatibiliza os dias da semana com a string do Blynk 
      else {currentWday = RTCWday;}

  currentSecDAY = ((currentHour * 3600) + (currentMin * 60) + currentSec);   // calcula o Segundo do dia atual
  //Serial.printf("Segundos do dia atual: %u\n", currentSecDAY);
  Serial.println("Up time: " + uptime_formatter::getUptime());
  Blynk.virtualWrite(V5, uptime_formatter::getUptime());                    // envia ao Blynk a informação
  Serial.println("-------------------------------------------------------------");
  Serial.print("");

  char RTC_Time[64];                         //Cria uma string formatada da estrutura "timeinfo"
  sprintf(RTC_Time, "%02d.%02d.%04d  -  %02d:%02d:%02d", currentDay, currentMonth, currentYear, currentHour, currentMin, currentSec);
  Serial.print("Data/hora do sistema:   ");
  Serial.println(RTC_Time);
  Blynk.virtualWrite(V0, RTC_Time);          // envia ao Blynk a informação de data, hora e minuto do RTC
  
	Serial.print("Inputs BIN:             ");
	Serial.println(inputPCF, BIN);              // mostra no terminal o estado das entradas em binário

  long rssi = WiFi.RSSI();
  Serial.print("RF Signal Level:        ");
  Serial.println(rssi);                       // Escreve o indicador de nível de sinal Wi-Fi
  Blynk.virtualWrite(V3, rssi);              // Envia ao Blynk informação RF Signal Level

  Serial.print("SETUP de Umidade Silo 1: ");
  Serial.println(setUmidade1);                      
  Blynk.virtualWrite(V38, setUmidade1);

  Serial.print("SETUP de Umidade Silo 2: ");
  Serial.println(setUmidade2);                   
  Blynk.virtualWrite(V66, setUmidade2);

  Serial.print("SETUP de Umidade Silo 3: ");
  Serial.println(setUmidade3);                 
  Blynk.virtualWrite(V86, setUmidade3);       // Envia ao Blynk informação do setup da umidade  

  // mostra no display:  rssi
  display.clearDisplay();
  display.setTextSize(1); 
  display.setCursor(0, 57);                   // coluna, linha 68, 57 sem rssi (44,57);

  display.print(rssi);
  display.print(" ");
  display.print("U:");
  display.print(average_UmiExt);        // umidade externa
  display.print('%');
  display.print(" ");

/*     Desabilitado o desenho das 4 barras indicadoras de nivel de RF
//                  (coluna, linha, largura, altura, cor) 
  if (rssi > -55 & rssi < -3) { 
    display.fillRect( 40, 25, 4, 17,WHITE);      // sinal muito bom
    display.fillRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -55 & rssi > -70) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal bom
    display.fillRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -70 & rssi > -78) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal razoável
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -78 & rssi > -82) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal baixo
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.drawRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sem sinal
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.drawRect( 26, 33, 4,  9,WHITE);
    display.drawRect( 19, 37, 4,  5,WHITE);
  }
  */
  // mostra State Blynk
  display.print(StrStateBlynk);
  Serial.print("Status Blynk Service:   "); Serial.println(StrStateBlynk);  

  display.setCursor(0,22);                     // coluna, linha 
  display.print("Silo 1:");
  display.print(setUmidade1);                  // umidade ajustada para comando automático
  display.print("  T:");
  display.print(timer_Motor1);    
  display.print(" ");
  display.println(StrModo_S1);

  display.setCursor(0,32);  
  display.print("Silo 2:");
  display.print(setUmidade2);        
  display.print("  T:");
  display.print(timer_Motor2);    
  display.print(" ");
  display.println(StrModo_S2);

  display.setCursor(0,42);  
  display.print("Silo 3:");
  display.print(setUmidade3);   
  display.print("  T:");
  display.print(timer_Motor3);    
  display.print(" ");
  display.println(StrModo_S3);

  // mostra hora:minutos:segundos no display
  display.setCursor(15,0);                 //(32,1);
  display.setTextSize(2);
  if(currentHour < 10){
    display.print(' ');
    display.print(currentHour);
    } else {display.print(currentHour, DEC);}
  display.print(":");
  if(currentMin < 10){
    display.print('0');
    display.print(currentMin);
    } else {display.print(currentMin);}
  display.print(":");
  if(currentSec < 10){
    display.print('0');
    display.print(currentSec);
    } else {display.print(currentSec);}
  display.display();            // mostra na tela

// *************************************************** //

  int rstTimer = WDT_TIMEOUT/1000;                          // 120000 miliseconds de WDT_TIMEOUT = 2 minutos
  while (BotaoRESET == 1) {                                 // BotaoRESET = 1 força e entrada na rotina do watchdog
        Serial.println("Botão de RESET no APP pressionado...");
        Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " App força RESET em ", rstTimer);
          if ((WDT_TIMEOUT/1000) - rstTimer == 2) {
            Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Calibrando o relógio interno...");
            setRTC();
            //ESP.restart();
            }
        rstTimer --;
        delay(900);   // faz um loop aqui... ATENÇÃO não alimenta o software watchdog de propósito!
        }

                     // até aqui usa aproximadamente 1300ms
  sendLogReset();    // depois de executar uma vez manda o log 
  getDataHora();     // usa aproximadamente  10ms
  ComandoOutput();   // usa aproximadamente 500ms

  //Envio dos dados que serão armazenados. Tenta enviar por até 60 segundos.
  if (currentMin == minAtualiza && currentSec > 0){
    Blynk.beginGroup();                             // https://docs.blynk.io/en/blynk-library-firmware-api/virtual-pins
      Blynk.virtualWrite(V100, UmiExt);             
      Blynk.virtualWrite(V101, average_UmiExt);
    Blynk.endGroup();
    //Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " LOG de Temp. e Umidade enviado");
    minAtualiza = minAtualiza + 5;                  // soma 15 a cada 15 minutos
    if (minAtualiza > 56) {minAtualiza = 0;}        // minAtualiza usado para enviar a cada 15 minutos
  }
  
  /*
    //Envio dos dados que serão armazenados a cada 60 segundos.
    if (currentSec == 0){
      Blynk.beginGroup();                             // https://docs.blynk.io/en/blynk-library-firmware-api/virtual-pins
        Blynk.virtualWrite(V0, UmiExt);             
        Blynk.virtualWrite(V1, average_UmiExt);
      Blynk.endGroup();
    }
    */

  Serial.printf("Período (ms) do Main2: %u\n", (millis() - tempo_start)); // cálculo do tempu utilizado até aqui

}

void MODBUS_Sensor(){
  Blynk.virtualWrite(V4, 255);                        // liga LED de sinalização no app
  /*
  // SENSOR EXTERNO 4x1 = ExtSensor4x1
  uint8_t resultEXT = ExtSensor.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor Externo ");
  Serial.print("Error = "); Serial.println( resultEXT );   // 0: ok, 226: falha
  Blynk.virtualWrite(V50, resultEXT);                      // envia Status do Sensor MODBUS (0 = OK, 226 = falha)
  // Gera os alarmes de push e no display no app - Blynk
  if (resultEXT != 0) {Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " FALHA NO SENSOR EXTERNO");
                     Blynk.logEvent("falha_de_sensor");
                     Blynk.virtualWrite(V102, 255);        // envia 255 para o led de sinalização
                     }

  if (resultEXT == ExtSensor.ku8MBSuccess){
    UmiExt    = (ExtSensor.getResponseBuffer(0)/10);
    TempExt   = (ExtSensor.getResponseBuffer(1)/10);

    Serial.print("Umidade Ext.:          "); Serial.print(UmiExt); Serial.println(" %");

    average_UmiExt = moving_average();                   // busca a média móvel da UmiExt
    Serial.print("Média Umidade Ext.:    "); Serial.print(average_UmiExt); Serial.println(" %");
    Serial.print("Temperatura Ext.:      "); Serial.print(TempExt); Serial.println(" C");

    Blynk.virtualWrite(V51, average_UmiExt);             // Envia ao Blynk a informação
    Blynk.virtualWrite(V52, TempExt);
    //Blynk.virtualWrite(V53, UmiExt);                   // V53 é do contador de RESET's - REMOVER PÓS TESTES
    Blynk.virtualWrite(V102, 0);                         // envia 0 para o led de sinalização

  } Serial.print("\n"); delay(2);
  */
  // Sensor_01
  uint8_t result_01 = Sensor_01.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor 01 ");
  Serial.print("Error -> (0 = Sensor OK): "); Serial.println( result_01 );   // 0: ok, 226: falha

  // Gera os alarmes de push e no display no app - Blynk
  if (result_01 != 0) {Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " FALHA NO SENSOR 01");
                     Blynk.logEvent("falha_de_sensor");
                     //Blynk.virtualWrite(V10, 255);        // envia 255 para o led de sinalização
                     //#D3435C - Blynk RED 
                     Blynk.setProperty(V10, "color", "#D03A20");
                     }
                     
  if (result_01 == Sensor_01.ku8MBSuccess){
    Umi_01    = (Sensor_01.getResponseBuffer(0)/10);
    Temp_01   = (Sensor_01.getResponseBuffer(1)/10);

    Serial.print("Umidade 01:          "); Serial.print(Umi_01); Serial.println(" %");
    //average_Umi_01 = moving_average();                 // busca a média móvel da Umi_01
    //Serial.print("Média Umidade 01:    "); Serial.print(average_Umi_01); Serial.println(" %");
    Serial.print("Temperatura 01:      "); Serial.print(Temp_01); Serial.println(" C");

    //Blynk.virtualWrite(V10, 0);                          // envia 0 para o led de sinalização
    //#23C48E - Blynk Green 
    Blynk.setProperty(V10, "color", "#64C366");
    Blynk.virtualWrite(V11, Umi_01);                     // Envia ao Blynk a informação
    Blynk.virtualWrite(V12, Temp_01);
    //Blynk.virtualWrite(V2, Umi_01);                    // V2 é do contador de RESET's - REMOVER PÓS TESTES
  } Serial.print("\n"); delay(2);

  // Sensor_02
  uint8_t result_02 = Sensor_02.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor 02 ");
  Serial.print("Error -> (0 = Sensor OK): "); Serial.println( result_02 );   // 0: ok, 226: falha

  // Gera os alarmes de push e no display no app - Blynk
  if (result_02 != 0) {Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " FALHA NO SENSOR 02");
                     Blynk.logEvent("falha_de_sensor");
                     //Blynk.virtualWrite(V15, 255);        // envia 255 para o led de sinalização
                     //#D3435C - Blynk RED 
                     Blynk.setProperty(V15, "color", "#D03A20");
                     }
                     
  if (result_02 == Sensor_02.ku8MBSuccess){
    Umi_02    = (Sensor_02.getResponseBuffer(0)/10);
    Temp_02   = (Sensor_02.getResponseBuffer(1)/10);

    Serial.print("Umidade 02:          "); Serial.print(Umi_02); Serial.println(" %");
    //average_Umi_02 = moving_average();                 // busca a média móvel da Umi_02
    //Serial.print("Média Umidade 02:    "); Serial.print(average_Umi_02); Serial.println(" %");
    Serial.print("Temperatura 02:      "); Serial.print(Temp_02); Serial.println(" C");

    //Blynk.virtualWrite(V15, 0);                          // envia 0 para o led de sinalização
    //#23C48E - Blynk Green 
    Blynk.setProperty(V15, "color", "#64C366");
    Blynk.virtualWrite(V16, Umi_02);                     // Envia ao Blynk a informação
    Blynk.virtualWrite(V17, Temp_02);
    //Blynk.virtualWrite(V2, Umi_02);                    // V2 é do contador de RESET's - REMOVER PÓS TESTES
  } Serial.print("\n"); delay(2);

  // Sensor_03
  uint8_t result_03 = Sensor_03.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor 03 ");
  Serial.print("Error -> (0 = Sensor OK): "); Serial.println( result_03 );   // 0: ok, 226: falha

  // Gera os alarmes de push e no display no app - Blynk
  if (result_03 != 0) {Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " FALHA NO SENSOR 03");
                     Blynk.logEvent("falha_de_sensor");
                     //Blynk.virtualWrite(V20, 255);        // envia 255 para o led de sinalização
                     //#D3435C - Blynk RED 
                     Blynk.setProperty(V20, "color", "#D03A20");
                     }
                     
  if (result_03 == Sensor_03.ku8MBSuccess){
    Umi_03    = (Sensor_03.getResponseBuffer(0)/10);
    Temp_03   = (Sensor_03.getResponseBuffer(1)/10);

    Serial.print("Umidade 03:          "); Serial.print(Umi_03); Serial.println(" %");
    //average_Umi_03 = moving_average();                 // busca a média móvel da Umi_03
    //Serial.print("Média Umidade 03:    "); Serial.print(average_Umi_03); Serial.println(" %");
    Serial.print("Temperatura 03:      "); Serial.print(Temp_03); Serial.println(" C");

    //Blynk.virtualWrite(V20, 0);                          // envia 0 para o led de sinalização
    //#23C48E - Blynk Green 
    Blynk.setProperty(V20, "color", "#64C366");
    Blynk.virtualWrite(V21, Umi_03);                     // Envia ao Blynk a informação
    Blynk.virtualWrite(V22, Temp_03);
    //Blynk.virtualWrite(V2, Umi_03);                    // V2 é do contador de RESET's - REMOVER PÓS TESTES
  } Serial.print("\n"); delay(2);

  // Sensor_04
  uint8_t result_04 = Sensor_04.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor 04 ");
  Serial.print("Error -> (0 = Sensor OK): "); Serial.println( result_04 );   // 0: ok, 226: falha

  // Gera os alarmes de push e no display no app - Blynk
  if (result_04 != 0) {Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " FALHA NO SENSOR 04");
                     Blynk.logEvent("falha_de_sensor");
                     //Blynk.virtualWrite(V25, 255);        // envia 255 para o led de sinalização
                     //#D3435C - Blynk RED 
                     Blynk.setProperty(V25, "color", "#D03A20");
                     }
                     
  if (result_04 == Sensor_04.ku8MBSuccess){
    Umi_04    = (Sensor_04.getResponseBuffer(0)/10);
    Temp_04   = (Sensor_04.getResponseBuffer(1)/10);

    Serial.print("Umidade 04:          "); Serial.print(Umi_04); Serial.println(" %");
    //average_Umi_04 = moving_average();                 // busca a média móvel da Umi_04
    //Serial.print("Média Umidade 04:    "); Serial.print(average_Umi_04); Serial.println(" %");
    Serial.print("Temperatura 04:      "); Serial.print(Temp_04); Serial.println(" C");

    //Blynk.virtualWrite(V25, 0);                          // envia 0 para o led de sinalização
    //#23C48E - Blynk Green 
    Blynk.setProperty(V25, "color", "#64C366");
    Blynk.virtualWrite(V26, Umi_04);                     // Envia ao Blynk a informação
    Blynk.virtualWrite(V27, Temp_04);
    //Blynk.virtualWrite(V2, Umi_04);                    // V2 é do contador de RESET's - REMOVER PÓS TESTES
  } Serial.print("\n"); delay(2);

  Blynk.virtualWrite(V4, 0);                             // desliga LED de sinalização no app

}

long moving_average(){
  for (int i = samples-1; i>0; i--) matriz_samples[i] = matriz_samples[i-1];   // desloca os elementos do vetor
  matriz_samples[0] = UmiExt;                                         // posição inicial recebe o valor inicial
  long acc = 0;                                                       // acumulador para a soma dos valores
  for (int i=0; i<samples; i++) acc = acc + matriz_samples[i];        // soma os valores dos dados do vetor
  return acc/samples;          // retorna a soma de todos os valores lidos dividido pelo número de amostras
}

void sendLogReset(){
  // envia razao do reset para o monitor serial e servidor
  if ((servicoIoTState==4) && (sendBlynk)){
    Serial.print("               BLYNK:  RODANDO COM SUCESSO!");
    esp_reset_reason_t r = esp_reset_reason();
    Serial.printf("\r\nReset reason %i - %s\r\n", r, resetReasonName(r));
    Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "",resetReasonName(r), " FW:", BLYNK_FIRMWARE_VERSION, " /",counterRST);
    Blynk.virtualWrite(V2, counterRST);                        // envia para tela do app
    //Blynk.syncVirtual (V40, V69, V89);                       // sincroniza datastream de agendamentos
    Blynk.virtualWrite(V10, 255);                              // envia 0 para o led de sinalização
    Blynk.virtualWrite(V15, 255);                              // envia 0 para o led de sinalização
    Blynk.virtualWrite(V20, 255);                              // envia 0 para o led de sinalização
    Blynk.virtualWrite(V25, 255);                              // envia 0 para o led de sinalização
    delay(250);
    MODBUS_Sensor();                                            // quando conectar lê sensor e envia os dados
    delay(250);
    // se reiniciar por (1) POWER ON RESET, ou Software reboot
    if (r == 1 || r == 3 ){
      Blynk.virtualWrite(V4, 255);                        // envia 1 para sinalização push no app via automação
      delay(3000);
      Blynk.virtualWrite(V4, 0);                          // envia 0 para "des_sinalizar" na automação
      //Blynk.logEvent("falha_de_energia", String("Teste - Falha de Energia!"));
      Blynk.logEvent("falha_de_energia");                 // registra o evento falha_de_energia no servidor
      }
    sendBlynk = false;                                    // garante que só envia uma vez essas informações
  }
}

void NTPserverTime(){          // habilitar no SETUP se quiser depurar na serial o horário recebido da internet
  struct tm timeinfo;
     //  "%u, %d.%m.%Y %H:%M:%S"  = 1, 17.08.2021 14:33:33
     //  %u = dia da semana em decimal, range 1 a 7, Segunda = 1.
     //  https://www.ibm.com/docs/en/z-workload-scheduler/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
  Serial.print("NTP: ");
  if(!getLocalTime(&timeinfo)){
    Serial.println(" falha ao sincronizar NTP!");
    } else { 
      Serial.print(&timeinfo, "%u, %d.%m.%Y %H:%M:%S");
      Serial.println(" hora recebida da internet.");}
}

void setRTC(){                        // comandos de set no DS1307
struct tm timeinfo;
     //  "%u, %d.%m.%Y %H:%M:%S"  = 1, 17.08.2021 14:33:33
     //  %u = dia da semana em decimal, range 1 a 7, Segunda = 1.
     //  https://www.ibm.com/docs/en/z-workload-scheduler/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
Serial.print("NTP: ");
if(!getLocalTime(&timeinfo)){
    Serial.println(" falha ao sincronizar NTP!");
    for (int i=9; i > 0; i= i-1) {
       delay(960);                    // 9 segundos no laço for considerando tempo de escrita no display
         // mostra no display
       display.clearDisplay(); 
       display.setCursor(25, 15);
       display.setTextSize(2);
       display.print("FALHA");
       display.setCursor(10, 35);      // coluna, linha 
       display.print("INTERNET");
       display.setCursor(110, 5); 
       display.print(i);
       display.display();             // mostra na tela
       }
    } else { 
  Serial.print(&timeinfo, "%u, %d.%m.%Y %H:%M:%S");
  Serial.println(" hora recebida da internet.");

  time_t now;                         // this is the epoch
  tm tm;                              // the structure tm holds time information in a more convient way
  time(&now);                         // read the current time
  //delay(50); 
  localtime_r(&now, &tm);             // update the structure tm with the current time
  //delay(50); 
  int ye = tm.tm_year + 1900;
  int mo = tm.tm_mon + 1;
  int da = tm.tm_mday;
  int ho = tm.tm_hour;
  int mi = tm.tm_min;
  int se = tm.tm_sec +1;

  //delay(50); 
  RTC.adjust(DateTime( ye , mo , da , ho , mi , se )); 
  delay(50); 

  preferences.begin  ("my-app", false);              // inicia 
  preferences.putUInt("counterRST", 0);              // grava em Preferences/My-app/counterRST, counterRST
  counterRST = preferences.getUInt("counterRST", 0); // Le da NVS
  preferences.end();
  delay(50);
  Serial.println("A data e hora foram recalibradas no relógio interno, e o contador de RESETs foi zerado!");
  Blynk.virtualWrite(V1, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Relógio calibrado");
  flagSetRTC = 0;

  for (int i=20; i > 0; i= i-1) {      // i= tempo que fica mostrando no display
       delay(960);                     // ex.: adianta 13 segundos por dia
         // mostra no display          // espera por pelo menos 14 segundos aqui para não calibrar mais de uma vez
       display.clearDisplay();         // quanto maior o tempo mais garantido que não vai repetir a calibração
       display.setCursor(25, 25);
       display.setTextSize(2);
       display.print("CAL..");
       display.setCursor(96, 25);     // coluna, linha 
       display.print(i);
       display.display();             // mostra na tela
       }
  }
}

void failMSG(String HW_status) {
  // mensagem de falha nos disposotivos de entrada e/ou saida (RTC, CIs PFC...)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  display.clearDisplay();                    // limpa o buffer
  display.display();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(45,7);                   // coluna, linha
  display.println("R&M");
  display.setTextSize(1);
  display.setCursor(42,27);
  display.println("Company");
  display.setCursor(15, 50);
  display.print("STOP: ");
  display.println(HW_status); 
  display.display();                         // mostra na tela a falha de hardware ocorrida
  while (1);                                 // PARA A EXECUCAO POR SEGURANCA e irá reiniciar pelo watchdog

}

// -------------------------------------- Recebe do APP as variaveis --------------------------------------

//int BotaoRESET;                               // BotaoRESET = Virtual do APP
BLYNK_WRITE(V39){
  BotaoRESET = param.asInt();                   // função de RESET que força a atuação do watchtdog
  }

// *************************************************** //

void setup(){
  // configuração do watchdog
  rtc_wdt_protect_off();               //Disable RTC WDT write protection
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC);
  rtc_wdt_set_time (RTC_WDT_STAGE0, WDT_TIMEOUT);
  rtc_wdt_enable();                    //Start the RTC WDT timer
  rtc_wdt_protect_on();                //Enable RTC WDT write protection

  pinMode(Heartbeat_PIN,OUTPUT);       // vinculo para setup do pino na biblioteca HeartBeat.h
  pinMode(rearme_PIN,OUTPUT);
  //dacWrite(CANAL_DAC0,   0);         // Saida DAC_01 da KC868-A6 - start ligado
  //dacWrite(CANAL_DAC1, 255);         // Saida DAC_02 da KC868-A6 - start desligado
  // DAC vai pulsar a cada reset ou atualização de software devido ao hardware (CI LM258)
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);        // ajusta os pinos do I2C

  Wire.beginTransmission(0x24);        // inicia a transmissao para o CI PCF8574 de saida endereço 0x24
  Wire.write(output_PLC);              // coloca todas saidas em HIGH, reles desligados (0b11111111)
  int errorCode_OUTPUT = Wire.endTransmission();
  //Serial.println(errorCode_OUTPUT);  // se errorCode_OUTPUT = 0, o dispositivo foi encontrado no endereço
  if (errorCode_OUTPUT != 0)  {
    Serial.println("Falha no dispositivo de saida PCF8574!");
    failMSG("FALHA OUPUT");
    } else { Serial.println("Saida I2C PCF8574     OK");}
  delay(50);

  Wire.beginTransmission(0x22);         // inicia a transmissao para o CI PCF8574 de entrada endereço 0x22
  int errorCode_INPUT = Wire.endTransmission();
  //Serial.println(errorCode_INPUT);    // se errorCode_OUTPUT = 0, o dispositivo foi encontrado no endereço
  if (errorCode_INPUT != 0)  {
    Serial.println("Falha no dispositivo de entrada PCF8574!");
    failMSG("FALHA INPUT");
    } else {Serial.println("Entrada I2C PCF8574   OK");}
  delay(50);

  // ------   Inicia e cria espaco na memoria NVS - namespace:my-app    ------
  preferences.begin("my-app", false);                  // Note: Namespace name is limited to 15 chars.
  //preferences.clear();                               // remove all preferences under the opened namespace
  //preferences.remove("counterRST");                  // or remove the counter key only
  counterRST = preferences.getUInt("counterRST", 0);   // lê da NVS, se counterRST nao existir retorna o valor 0
  counterRST++;                                        // incrementa a cada reset
  preferences.putUInt("counterRST", counterRST);       // grava o novo valor de counterRST em Preferences/My-app/counterRST
  
  preferences.end();                                   // finaliza o uso da memória NVS  
  delay(50);

  ResetReason();                                       // imprime na serial razao do ultimo reset e
                                                       // se iniciar por (1) POWER ON RESET coloca o app em modo manual
  Serial.printf("Quantidade de RESETs: %u\n", counterRST);

      //  Desabilidada a ativacao do motor pelo efeito memória (MemMotorState)
      /*
          if (MemMotorState == false){                               // verificar se necessario forcaLiga1 == 0 ?!?!            
          //digitalWrite(OUT1_PIN, HIGH);
          output_PLC = output_PLC & 0b1111110;    // faz AND, apenas bit 0 = 0
          Wire.beginTransmission(0x24);           // escreve na saida do PLC
          Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          forcaLiga1 = 0;
          Serial.printf("Motor ativado pela memória! \n");
          delay (1000);
          //digitalWrite(OUT1_PIN, LOW);
          output_PLC = output_PLC | 0b00000001;   // faz OU, apenas bit 0 = 1
          Wire.beginTransmission(0x24);           // escreve na saida do PLC
          Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          cicloOFF_1 = 0;
          } else if (MemMotorState = true){ 
                     //digitalWrite(OUT2_PIN, HIGH);
                     output_PLC = output_PLC & 0b1111101;    // faz AND, apenas bit 1 = 0
                     Wire.beginTransmission(0x24);           // escreve na saida do PLC
                     Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
                     Wire.endTransmission();
                     forcaDESLiga1 = 0;
                     Serial.printf("Motor desativado pela memória! \n");
                     delay (1000);
                     //digitalWrite(OUT2_PIN, LOW);
                     output_PLC = output_PLC | 0b00000010;   // faz OU, apenas bit 1 = 1
                     Wire.beginTransmission(0x24);           // escreve na saida do PLC
                     Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
                     Wire.endTransmission();
                     cicloON_1 = 0;                    
                     }
        */

  

  Serial1.begin(9600, SERIAL_8N1, 14, 27);   // porta RS-485 do hardware KC-868-A6
  //ExtSensor.begin  (Slave_ID_EXT, Serial1);         // Slave address: 20H  Sensor 4x1
  Sensor_01.begin  (Slave_ID_01,  Serial1);         // Slave address: 01
  Sensor_02.begin  (Slave_ID_02,  Serial1);         // Slave address: 02
  Sensor_03.begin  (Slave_ID_03,  Serial1);         // Slave address: 03
  Sensor_04.begin  (Slave_ID_04,  Serial1);         // Slave address: 04
  //ExtSensorCWT.begin  ( 1, Serial1);       // Slave address: 01H  Sensor CWT-TH04

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  timerStart();                              // rotina de logomarca temporizada (minimo 10 segundos para RTC start)
  /*
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  display.clearDisplay();                    // limpa o buffer
  display.setTextSize(2);                    // tamanho do texto
  display.setTextColor(SSD1306_WHITE);       // cor do texto
  display.setCursor(5, 25);                  // coluna, linha 
  display.println("Iniciando!");             // informação
  display.display();                         // mostra na tela
  delay(10000);                              // delay necessário para o RTC DS1307 inicializar
  */
 
  MODBUS_Sensor();                           // lê sensores MODBUS
  for (int i = samples-1; i>-1; i--) 
        matriz_samples[i] = UmiExt; //100;   // inicia todos os elementos do vetor com o valor de Umidade
   
  average_UmiExt = moving_average();
  Serial.print("Primeira Média de Umidade Ext.: "); Serial.print(average_UmiExt); Serial.println(" %");

  if (! RTC.begin()) {
     Serial.println("Não foi possível encontrar o RTC!");
     failMSG("FALHA RTC");
     } else { delay(100);                     // Serial.println("RTC identificado com sucesso!")
              DateTime now = RTC.now();
              char currentTime[64];
              //Cria uma string formatada da estrutura "timeinfo"
              sprintf(currentTime, "%02d.%02d.%04d - %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(),now.second());
              Serial.println("Leitura do RTC DS1307 OK");
              Serial.print  ("Leitura da Data e Hora do sistema: ");
              Serial.println(currentTime);
              delay(100);
              }

  //rtc.adjust(DateTime(__DATE__, __TIME__));                 // seta o RTC com os parametros da data e hora da compilação
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);   // inicia e busca as infos de data e hora (NTP)

  edgentTimer.setInterval(  1000L, Main2);                     // rotina se repete a cada XXXXL (milisegundos)
  edgentTimer.setInterval(  5000L, timerButtonAPP);            // timer para receber os comandos do APP
  edgentTimer.setInterval(180000L, MODBUS_Sensor);             // lê sensores MODBUS a cada XXXXL (milisegundos) 300000L
  BlynkEdgent.begin();
  delay(100);
  Serial.println("--------------------------- SETUP Concluido ---------------------------");
}

// ----------------------------- temporizador para ler os botoes do app --------------------------------------
void timerButtonAPP(){                        // timer de botao pressionado no app
 // *************************************************************************** //
}

void getDataHora(){
  // *************************************************************************** //
}

void ComandoOutput() {
}

//int timerON = 0;            // executa uma vez timer de X segundos configurados em tempoStart a cada reinicio
void timerStart() {
  if (timerON != 1){
  Serial.println("Temporizando início do sistema...  "); 
  for (tempoStart; tempoStart > -1; tempoStart--) {
       heartBeat();     delay(900);               // +/- 1000ms considerando o tempo de escrita no display e heartbeat
       //Serial.println(tempoStart);
       display.clearDisplay();                    // limpa o buffer do display
       display.setTextSize(2);
       display.setTextColor(SSD1306_WHITE);
       display.setCursor(45,10);
       display.println("R&M");
       display.setTextSize(1);
       display.setCursor(42,30);
       display.println("Company");
       display.setCursor(70, 55);
       display.print("FW:");                      //  versao BLYNK_FIRMWARE_VERSION
       display.println(BLYNK_FIRMWARE_VERSION);   //  F1 = falha do sensor
       display.setCursor(5, 55);
       display.print("RST: "); 
       display.println(counterRST);               // quantidade de reset's lido da memória NVS

       //Mostra timer de inicialização
       display.setTextSize(2);
       display.setCursor(96, 24);    // coluna, linha 
       display.print(tempoStart);    // informação
       display.display();            // mostra na tela
       }
    timerON = 1;
  }
}

void loop() {
 heartBeat();                        // rotina na biblioteca HeartBeat.h
 //timerStart();                     // rotina de logomarca temporizada
 BlynkEdgent.run();

 // testa se o botão USER foi pressionado OU 
 // se esta na hora de ajustar e vai para rotina de set RTC, tenta por 9 segundos
 if ((flagSetRTC == 1) ||
     /*(currentWday = 1) && */(currentHour == 2) && (currentMin == 10) && (currentSec < 10)) {
     setRTC();}

 if (flagIoTStatus == 1){
  /*
   display.setTextSize(1); 
   display.setCursor(68,57); 
   display.print(StrStateBlynk);
   display.display();               // mostra na tela
   */
   flagIoTStatus = 0;}
}