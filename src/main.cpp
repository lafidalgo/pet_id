/*Biblioteca do Arduino*/
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "RTClib.h"  
#include <SPI.h>

/*Bibliotecas FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/*mapeamento de pinos*/ 
#define RXD2 16
#define TXD2 17
#define RFIDResetPin 19
#define btnMasterPin 23
#define btnCoverPin 5
#define btnDispenserPin 18
#define RGBRedPin 32
#define RGBGreenPin 33
#define RGBBluePin 25
#define weightCoverDTPin 27
#define weightCoverSCKPin 14
#define weightDispenserDTPin 26
#define weightDispenserSCKPin 13
#define dispenserPin 2
#define openCoverPin 4
#define closeCoverPin 15
#define ackStepperPin 34

RTC_DS3231 rtc;

WiFiClient espClient;
PubSubClient client(espClient);

#define config_file "/config.json"
#define logs_cover "/logsCover.txt"
#define logs_dispenser "/logsDispenser.txt"
#define logs_errors "/logsErrors.txt"

// Tamanho do Objeto JSON
const   size_t    JSON_SIZE  = 1200; //https://arduinojson.org/v6/assistant/

// Variáveis Globais ------------------------------------
char device_id[10];
char wifi_ssid[30];
char wifi_password[30];
char mqtt_server[30];
int mqtt_port;
char mqtt_topic[30];
char mqtt_topic_heartbeat[40];
char mqtt_topic_logs[40];
char mqtt_topic_logs_dispenser[40];
char mqtt_topic_logs_errors[40];
char mqtt_topic_commands[40];
char mqtt_topic_messages[40];
char mqtt_topic_dispenser[40];
char mqtt_topic_schedule[40];
char mqtt_topic_config[40];
char ntp_server[30]; 

/* Variáveis para armazenamento do handle das tasks*/
TaskHandle_t taskServoTampaHandle = NULL;
TaskHandle_t taskServoDispenserHandle = NULL;
TaskHandle_t taskRGBHandle = NULL;
TaskHandle_t taskRFIDHandle = NULL;
TaskHandle_t taskDispenserScheduleCheckHandle = NULL;
TaskHandle_t taskSyncTimeHandle = NULL;
TaskHandle_t taskConnectionsCheckHandle = NULL;
TaskHandle_t taskHeartbeatHandle = NULL;
TaskHandle_t taskRFIDResetHandle = NULL;

QueueHandle_t xFilaRGBRed;
QueueHandle_t xFilaRGBGreen;
QueueHandle_t xFilaRGBBlue;
QueueHandle_t xFilaCardNumber;
QueueHandle_t xFilaActivationType;
QueueHandle_t xFilaActivationTypeDispenser;
QueueHandle_t xFilaDispenserTime;

SemaphoreHandle_t xSemaphoreOpenCover;
SemaphoreHandle_t xSemaphoreCloseCover;
SemaphoreHandle_t xSemaphoreOpenDispenser;
SemaphoreHandle_t xSemaphoreMasterMode;
SemaphoreHandle_t xSemaphoreACKStepper;

TimerHandle_t xTimerClose;
TimerHandle_t xTimerMasterMode;
TimerHandle_t xTimerDispenserSchedule;
TimerHandle_t xTimerDispenserScheduleCheck;
TimerHandle_t xTimerSyncTime;
TimerHandle_t xTimerConnectionsCheck;
TimerHandle_t xTimerHeartbeat;

/*protótipos das Tasks*/
void vTaskServoTampa(void *pvParameters);
void vTaskServoDispenser(void *pvParameters);
void vTaskRGB(void *pvParameters);
void vTaskRFID(void *pvParameters);
void vTaskDispenserScheduleCheck(void *pvParameters);
void vTaskSyncTime(void *pvParameters);
void vTaskConnectionsCheck(void *pvParameters);
void vTaskHeartbeat(void *pvParameters);
void vTaskRFIDReset(void *pvParameters);

void callBackTimerClose(TimerHandle_t xTimer);
void callBackTimerMasterMode(TimerHandle_t xTimer);
void callBackTimerDispenserSchedule(TimerHandle_t xTimer);
void callBackTimerDispenserScheduleCheck(TimerHandle_t xTimer);
void callBackTimerSyncTime(TimerHandle_t xTimer);
void callBackTimerConnectionsCheck(TimerHandle_t xTimer);
void callBackTimerHeartbeat(TimerHandle_t xTimer);

char checkSum (char message_payload[10]);

void  configReset();
boolean configRead();
boolean configSave();
boolean registeredCardsRead(char card_number[10]);
boolean registeredCardsWrite(char card_number[10]);
boolean registeredCardsDelete(char card_number[10]);
boolean dispenserScheduleRead();
boolean dispenserScheduleWrite(char *timestamp, int food_weight);
boolean dispenserScheduleClear();
boolean updateLastUpdate();

char * getTime();
char * getTimeISO();
int timestampDayToInt(char * time);
int timestampHourToInt(const char * time);
void syncTime();
char * getTimeRTC();
char * getTimeISORTC();

void setupWiFi(char ssid[30], char pwd[30], int max_tries, int delay);

void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnectionCheck(int max_tries, int delay);
void mqttPublishLog(char *id, char *time_stamp, long duration_open, long food_weight, char *card_payload, bool activation_type);
void mqttPublishLogDispenser(char *id, char *time_stamp, long food_weight, bool activation_type);
void mqttPublishLogError(char *id, const char *error);
boolean mqttPublishConfig();
void mqttPublishMessage(char * id, const char * message);
void mqttPublishHeartbeat(char * id);

void btnMasterISRCallBack();
void btnCoverISRCallBack();
void btnDispenserISRCallBack();
void ackStepperISRCallBack();

bool writeFile(String values, String pathFile, bool appending);
String readFile(String pathFile);
void readSendFile(String pathFile, char * mqtt_topic);

void setRGBColor(int red, int green, int blue);
void blinkRGBColor(int red, int green, int blue);

void openCover();
void closeCover();
void rotateDispenser();

/*função setup*/
void setup() {
  Serial.begin(115200); //configura comunicação serial com baudrate de 9600
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(RFIDResetPin, OUTPUT);
  digitalWrite(RFIDResetPin, HIGH);

  pinMode(btnMasterPin,INPUT_PULLUP);
  pinMode(btnCoverPin,INPUT_PULLUP);
  pinMode(btnDispenserPin,INPUT_PULLUP);

  pinMode(RGBRedPin, OUTPUT); //DEFINE O PINO COMO SAÍDA
  pinMode(RGBGreenPin, OUTPUT); //DEFINE O PINO COMO SAÍDA
  pinMode(RGBBluePin, OUTPUT); //DEFINE O PINO COMO SAÍDA 

  pinMode(dispenserPin, OUTPUT); //DEFINE O PINO COMO SAÍDA
  digitalWrite(dispenserPin, HIGH);
  pinMode(openCoverPin, OUTPUT); //DEFINE O PINO COMO SAÍDA
  digitalWrite(openCoverPin, HIGH);
  pinMode(closeCoverPin, OUTPUT); //DEFINE O PINO COMO SAÍDA
  digitalWrite(closeCoverPin, HIGH);
  pinMode(ackStepperPin, INPUT); //DEFINE O PINO COMO SAÍDA  

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }

  if (!rtc.begin()) {
    mqttPublishLogError(device_id, "Não foi possível encontrar RTC.");
    ESP.restart();
  }
  if(rtc.lostPower()){
    Serial.println("DS3231 OK!");
  }

  configRead();
  readFile(logs_cover);
  readFile(logs_dispenser);
  readFile(logs_errors);

  setupWiFi(wifi_ssid, wifi_password, 10, 500);

  configTime(-3*3600, 0, ntp_server);
  syncTime();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  mqttConnectionCheck(10, 5000);
  mqttPublishMessage(device_id, "Pote Ligou!");

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    Serial.println("Enviando logs salvos.");
    readSendFile(logs_cover, mqtt_topic_logs);
    readSendFile(logs_dispenser, mqtt_topic_logs_dispenser);
    readSendFile(logs_errors, mqtt_topic_logs_errors);
  }

  xFilaRGBRed = xQueueCreate(1, sizeof(int));
  xFilaRGBGreen = xQueueCreate(1, sizeof(int));
  xFilaRGBBlue = xQueueCreate(1, sizeof(int));
  xFilaCardNumber = xQueueCreate(1, sizeof(char[10]));
  xFilaActivationType = xQueueCreate(1, sizeof(bool));
  xFilaActivationTypeDispenser = xQueueCreate(2, sizeof(bool));
  xFilaDispenserTime = xQueueCreate(2, sizeof(int));

  attachInterrupt(digitalPinToInterrupt(btnMasterPin), btnMasterISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnCoverPin), btnCoverISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnDispenserPin), btnDispenserISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(ackStepperPin), ackStepperISRCallBack, RISING);

  xSemaphoreMasterMode = xSemaphoreCreateBinary();

  if(xSemaphoreMasterMode == NULL){
   mqttPublishLogError(device_id, "Não foi possível criar o semaforo!");
   ESP.restart();
  }

  xSemaphoreOpenCover = xSemaphoreCreateBinary();

  if(xSemaphoreOpenCover == NULL){
   mqttPublishLogError(device_id, "Não foi possível criar o semaforo!");
   ESP.restart();
  }

  xSemaphoreCloseCover = xSemaphoreCreateBinary();

  if(xSemaphoreCloseCover == NULL){
   mqttPublishLogError(device_id, "Não foi possível criar o semaforo!");
   ESP.restart();
  }

  xSemaphoreOpenDispenser = xSemaphoreCreateBinary();

  if(xSemaphoreOpenDispenser == NULL){
   mqttPublishLogError(device_id, "Não foi possível criar o semaforo!");
   ESP.restart();
  }

  xSemaphoreACKStepper = xSemaphoreCreateBinary();

  if(xSemaphoreACKStepper == NULL){
   mqttPublishLogError(device_id, "Não foi possível criar o semaforo!");
   ESP.restart();
  }

  xTimerClose = xTimerCreate("TIMER CLOSE",pdMS_TO_TICKS(5000),pdTRUE,0,callBackTimerClose);
  xTimerMasterMode = xTimerCreate("TIMER Master Mode",pdMS_TO_TICKS(5000),pdTRUE,0,callBackTimerMasterMode);
  xTimerDispenserSchedule = xTimerCreate("TIMER Dispenser Schedule",pdMS_TO_TICKS(5000),pdTRUE,0,callBackTimerDispenserSchedule);
  xTimerDispenserScheduleCheck = xTimerCreate("TIMER Dispenser Schedule Check",pdMS_TO_TICKS(600000),pdTRUE,0,callBackTimerDispenserScheduleCheck);
  xTimerStart(xTimerDispenserScheduleCheck, 0);
  xTimerSyncTime = xTimerCreate("TIMER Sync Time",pdMS_TO_TICKS(600000),pdTRUE,0,callBackTimerSyncTime);
  xTimerStart(xTimerSyncTime, 0);
  xTimerConnectionsCheck = xTimerCreate("TIMER Connections Check",pdMS_TO_TICKS(60000),pdTRUE,0,callBackTimerConnectionsCheck);
  xTimerStart(xTimerConnectionsCheck, 0);
  xTimerHeartbeat = xTimerCreate("TIMER Heartbeat",pdMS_TO_TICKS(15000),pdTRUE,0,callBackTimerHeartbeat);
  xTimerStart(xTimerHeartbeat, 0);

  /*criação das tasks*/
  if(xTaskCreatePinnedToCore(vTaskServoTampa,"TASK SERVO TAMPA",configMINIMAL_STACK_SIZE + 4096,NULL,1,&taskServoTampaHandle, APP_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Servo Tampa");
    ESP.restart();
  }
  vTaskSuspend(taskServoTampaHandle);

  if(xTaskCreatePinnedToCore(vTaskServoDispenser,"TASK SERVO DISPENSER",configMINIMAL_STACK_SIZE + 4096,NULL,1,&taskServoDispenserHandle, APP_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Servo Dispenser");
    ESP.restart();
  }
  vTaskSuspend(taskServoDispenserHandle);

  if(xTaskCreatePinnedToCore(vTaskRGB,"TASK RGB",configMINIMAL_STACK_SIZE + 2048,NULL,1,&taskRGBHandle, APP_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task RGB");
    ESP.restart();
  }
  vTaskSuspend(taskRGBHandle);

  if(xTaskCreatePinnedToCore(vTaskRFID,"TASK RFID",configMINIMAL_STACK_SIZE + 6144,NULL,1,&taskRFIDHandle, APP_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task RFID");
    ESP.restart();
  }

  if(xTaskCreatePinnedToCore(vTaskDispenserScheduleCheck,"TASK DISPENSER SCHEDULE CHECK",configMINIMAL_STACK_SIZE+4096,NULL,1,&taskDispenserScheduleCheckHandle, PRO_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Dispenser Schedule Check");
    ESP.restart();
  }

  if(xTaskCreatePinnedToCore(vTaskSyncTime,"TASK SYNC TIME",configMINIMAL_STACK_SIZE+4096,NULL,1,&taskSyncTimeHandle, PRO_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Sync Time");
    ESP.restart();
  }

  if(xTaskCreatePinnedToCore(vTaskConnectionsCheck,"TASK CONNECTIONS CHECK",configMINIMAL_STACK_SIZE+2048,NULL,1,&taskConnectionsCheckHandle, PRO_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Connections Check");
    ESP.restart();
  }

  if(xTaskCreatePinnedToCore(vTaskHeartbeat,"TASK HEARTBEAT",configMINIMAL_STACK_SIZE+2048,NULL,1,&taskHeartbeatHandle, PRO_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task Heartbeat");
    ESP.restart();
  }

  if(xTaskCreatePinnedToCore(vTaskRFIDReset,"TASK RFID RESET",configMINIMAL_STACK_SIZE+2048,NULL,1,&taskRFIDResetHandle, PRO_CPU_NUM) == pdFAIL){
    mqttPublishLogError(device_id, "Não foi possível criar a Task RFID Reset");
    ESP.restart();
  }
  vTaskSuspend(taskRFIDResetHandle);


  mqttPublishConfig();
}

/*função loop*/
void loop() {
  if(client.connected()){
    client.loop();
  }
}

//.......................Tasks.............................
void vTaskServoTampa(void *pvParameters)
{
  long duration;
  char *timestamp;
  char card_number[11];
  bool activationtype;
    while (1)
    {
      xQueueReceive(xFilaCardNumber, &card_number, portMAX_DELAY);
      xQueueReceive(xFilaActivationType, &activationtype, portMAX_DELAY);
      xSemaphoreTake(xSemaphoreOpenCover,portMAX_DELAY);
      blinkRGBColor(0, 255, 0);
      mqttPublishMessage(device_id, "Abrindo tampa.");
      duration = millis();
      timestamp = getTimeISORTC();
      openCover();
      xTimerStart(xTimerClose,0);

      xSemaphoreTake(xSemaphoreCloseCover,portMAX_DELAY);
      xTimerStop(xTimerClose,0);
      duration = millis() - duration;
      closeCover();

      mqttPublishLog(device_id, timestamp, duration, 123, card_number, activationtype);
      free(timestamp);
      mqttPublishMessage(device_id, "Fechando tampa.");
      vTaskSuspend(taskRFIDResetHandle);
      vTaskSuspend(taskServoTampaHandle);
    }
}

void vTaskServoDispenser(void *pvParameters)
{
  int dispenser_time;
  char *timestamp;
  bool activationtype;
    while (1)
    {
      xQueueReceive(xFilaDispenserTime, &dispenser_time, portMAX_DELAY);
      xQueueReceive(xFilaActivationTypeDispenser, &activationtype, portMAX_DELAY);
      xSemaphoreTake(xSemaphoreOpenDispenser,portMAX_DELAY);
      mqttPublishMessage(device_id, "Abrindo dispenser.");
      timestamp = getTimeISORTC();

      openCover();
      rotateDispenser();
      vTaskDelay(pdMS_TO_TICKS(dispenser_time));
      closeCover();
    
      mqttPublishLogDispenser(device_id, timestamp, dispenser_time, activationtype);
      free(timestamp);
      mqttPublishMessage(device_id, "Fechando o dispenser.");
      vTaskResume(taskDispenserScheduleCheckHandle);
      vTaskSuspend(taskServoDispenserHandle);
    }
}

void vTaskRGB(void *pvParameters)
{
  int red, green, blue;

    while (1)
    {
      xQueueReceive(xFilaRGBRed, &red, portMAX_DELAY);
      xQueueReceive(xFilaRGBGreen, &green, portMAX_DELAY);
      xQueueReceive(xFilaRGBBlue, &blue, portMAX_DELAY);

      setRGBColor(red, green, blue);
      vTaskDelay(pdMS_TO_TICKS(1000));
      setRGBColor(0, 0, 0);

      vTaskSuspend(taskRGBHandle);
    }
}

void vTaskRFID(void *pvParameters)
{
  char message[14];
  int message_index = 0;

  int serial_read;

  char start_code, check_sum, end_code;
  char payload[11];
  int payload_index = 0;

  char check_sum_xor;
  bool activationtype;

    while (1)
    {
      if(Serial2.available()){ // wait for the serial port to send data
        serial_read = Serial2.read();
        if(message_index == 0){
          if(serial_read == 0x02){
            message[message_index] = serial_read;
            message_index ++;
          }
        }
        else{
          message[message_index] = serial_read;
          message_index ++;
        }
      }

      if(message_index == 13){ //Message buffer complete
      message[13] = '\0'; 
      message_index = 0;
      
      //Format values
      for(payload_index = 0; payload_index < 10; payload_index ++){
        payload[payload_index] = message[payload_index+1];
      }
      payload[10] = '\0'; 
      
      start_code = message[0];
      check_sum = message[11];
      end_code = message[12];
    
      //Check start code
      if(start_code != 0x02){
        Serial.println("Start code not valid!");
      }
      
      //Check checksum
      check_sum_xor = checkSum(payload);
      if(check_sum != check_sum_xor){
        Serial.println("Checksum not valid!");
      }
      
      //Check end code
      if(end_code != 0x03){
        Serial.println("End code not valid!");
      }

      if(start_code == 0x02 && check_sum == check_sum_xor && end_code == 0x03){
        Serial.println(payload);

        if(xSemaphoreTake(xSemaphoreMasterMode,pdMS_TO_TICKS(10)) == pdTRUE){
          if(registeredCardsRead(payload) == true){
            registeredCardsDelete(payload);
            blinkRGBColor(255, 0, 0);
          }
          else{
            registeredCardsWrite(payload);
            blinkRGBColor(0, 255, 0);
          }
          mqttPublishMessage(device_id, "Saindo do modo mestre.");
          xTimerStop(xTimerMasterMode,0);
        }
        else{ 
          if(registeredCardsRead(payload) == true){
            activationtype = true;
            xQueueOverwrite(xFilaCardNumber, &payload);
            xQueueOverwrite(xFilaActivationType, &activationtype);
            vTaskResume(taskServoTampaHandle);
            xSemaphoreGive(xSemaphoreOpenCover);
            vTaskResume(taskRFIDResetHandle);
          }
          else{
            blinkRGBColor(255, 0, 0);
          }
        }
      }
      else{
        int i = 0;
        for(i = 0; i < 14; i ++){
          Serial.print(message[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        mqttPublishLogError(device_id, "Leitura inválida do cartão.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskDispenserScheduleCheck(void *pvParameters)
{
    while (1)
    {
      Serial.println("Checando dispenser schedule.");
      dispenserScheduleRead();
      vTaskSuspend(taskDispenserScheduleCheckHandle);
    }
}

void vTaskSyncTime(void *pvParameters)
{
    while (1)
    {
      Serial.println("Sincronizando RTC com NTP.");
      syncTime();
      vTaskSuspend(taskSyncTimeHandle);
    }
}

void vTaskConnectionsCheck(void *pvParameters)
{
    while (1)
    {
      Serial.println("Checando WiFi e MQTT.");
      if (WiFi.status() != WL_CONNECTED)
      { 
        mqttPublishLogError(device_id, "Erro ao conectar no WiFi.");                     
        setupWiFi(wifi_ssid, wifi_password, 10, 500);
      }
      
      if(WiFi.status() == WL_CONNECTED && !client.connected()){
        mqttPublishLogError(device_id, "Erro ao conectar no MQTT.");
        Serial.println("Reiniciando ESP");
        ESP.restart();
      }
      vTaskSuspend(taskConnectionsCheckHandle);
    }
}

void vTaskHeartbeat(void *pvParameters)
{
    while (1)
    {
      Serial.println("Enviando heartbeat.");
      mqttPublishHeartbeat(device_id);
      vTaskSuspend(taskHeartbeatHandle);
    }
}

void vTaskRFIDReset(void *pvParameters)
{
    while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(2000));
      digitalWrite(RFIDResetPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(RFIDResetPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(RFIDResetPin, HIGH);
    }
}

//.......................Timers.............................
void callBackTimerClose(TimerHandle_t xTimer){
  xSemaphoreGive(xSemaphoreCloseCover);
  xSemaphoreTake(xSemaphoreOpenCover,pdMS_TO_TICKS(10));
}

void callBackTimerMasterMode(TimerHandle_t xTimer){
  xSemaphoreTake(xSemaphoreMasterMode,pdMS_TO_TICKS(10));
  Serial.println("Tempo modo mestre esgotado.");
  xTimerStop(xTimerMasterMode,0);
}

void callBackTimerDispenserSchedule(TimerHandle_t xTimer){
  vTaskResume(taskServoDispenserHandle);
  xSemaphoreGive(xSemaphoreOpenDispenser);
  xTimerStop(xTimerDispenserSchedule, 0);
}

void callBackTimerDispenserScheduleCheck(TimerHandle_t xTimer){
  vTaskResume(taskDispenserScheduleCheckHandle);
}

void callBackTimerSyncTime(TimerHandle_t xTimer){
  vTaskResume(taskSyncTimeHandle);
}

void callBackTimerConnectionsCheck(TimerHandle_t xTimer){
  vTaskResume(taskConnectionsCheckHandle);
}

void callBackTimerHeartbeat(TimerHandle_t xTimer){
  if(WiFi.status() == WL_CONNECTED && client.connected()){
    vTaskResume(taskHeartbeatHandle);
  }
}

//......................ISR.............................

void btnMasterISRCallBack(){
  BaseType_t xHighPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(xSemaphoreMasterMode,&xHighPriorityTaskWoken);
  blinkRGBColor(0, 0, 255);
  Serial.println("Entrando no modo mestre.");
  xTimerStart(xTimerMasterMode,0);
}

void btnCoverISRCallBack(){
  BaseType_t xHighPriorityTaskWoken = pdFALSE;
  char card_number[] = "0000000000";
  bool activationtype = false;

  xQueueOverwrite(xFilaCardNumber, &card_number);
  xQueueOverwrite(xFilaActivationType, &activationtype);
  vTaskResume(taskServoTampaHandle);
  xSemaphoreGiveFromISR(xSemaphoreOpenCover, &xHighPriorityTaskWoken);
  xTimerStart(xTimerClose,0);
}

void btnDispenserISRCallBack(){
  BaseType_t xHighPriorityTaskWoken = pdFALSE;
  int dispenser_time = 5000;
  bool activationtype = false;

  xQueueSendToFrontFromISR(xFilaDispenserTime, &dispenser_time, &xHighPriorityTaskWoken);
  xQueueSendToFrontFromISR(xFilaActivationTypeDispenser, &activationtype, &xHighPriorityTaskWoken);
  vTaskResume(taskServoDispenserHandle);
  xSemaphoreGiveFromISR(xSemaphoreOpenDispenser, &xHighPriorityTaskWoken);
}

void ackStepperISRCallBack(){
  BaseType_t xHighPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(xSemaphoreACKStepper, &xHighPriorityTaskWoken);
}

//.......................RFID.............................
char checkSum (char message_payload[10]){
  char data_ascii_nibble[10];
  int i;
  int j = 0;
  char sum = 0x00;
  int data_ascii[5];
  
  for(i = 0; i<10; i++){
      if(message_payload[i]<=0x39){
        data_ascii_nibble[j] = (message_payload[i] - 0x30);
      }
      else {
        data_ascii_nibble[j] = (message_payload[i] - 0x37);
      }
      j++;
  }
    
  j = 0;
  for(i = 0; i<5; i++){
    data_ascii[i] = (data_ascii_nibble[j] << 4) | (data_ascii_nibble[j+1]);
    j += 2;
  }

  for(i = 0; i < 5; i++){
    sum ^= data_ascii[i];
  }

  return sum;
}

//.......................JSON.............................
void  configReset() {
  // Define configuração padrão
  strlcpy(device_id, "0", sizeof(device_id)); 
  strlcpy(wifi_ssid, "", sizeof(wifi_ssid)); 
  strlcpy(wifi_password, "", sizeof(wifi_password));
  strlcpy(mqtt_server, "", sizeof(mqtt_server)); 
  mqtt_port = 1883;
  strlcpy(mqtt_topic, "", sizeof(mqtt_topic));
  strlcpy(mqtt_topic_heartbeat, "", sizeof(mqtt_topic_heartbeat));
  strlcpy(mqtt_topic_logs, "", sizeof(mqtt_topic_logs));
  strlcpy(mqtt_topic_logs_dispenser, "", sizeof(mqtt_topic_logs_dispenser));
  strlcpy(mqtt_topic_logs_errors, "", sizeof(mqtt_topic_logs_errors));
  strlcpy(mqtt_topic_commands, "", sizeof(mqtt_topic_commands));
  strlcpy(mqtt_topic_messages, "", sizeof(mqtt_topic_messages));
  strlcpy(mqtt_topic_dispenser, "", sizeof(mqtt_topic_dispenser));
  strlcpy(mqtt_topic_schedule, "", sizeof(mqtt_topic_schedule));
  strlcpy(mqtt_topic_config, "", sizeof(mqtt_topic_config));

  Serial.println("Parâmetros de configuração resetados.");
}
 
boolean configRead() {
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    // Sucesso na leitura
    strlcpy(device_id, jsonConfig["device_id"]      | "", sizeof(device_id)); 
    strlcpy(wifi_ssid, jsonConfig["wifi_ssid"]  | "", sizeof(wifi_ssid)); 
    strlcpy(wifi_password, jsonConfig["wifi_password"]      | "", sizeof(wifi_password));
    strlcpy(mqtt_server, jsonConfig["mqtt_server"]      | "", sizeof(mqtt_server)); 
    mqtt_port = jsonConfig["mqtt_port"];
    strlcpy(mqtt_topic, jsonConfig["mqtt_topic"]      | "", sizeof(mqtt_topic));
    strcat(mqtt_topic_heartbeat, mqtt_topic);
    strcat(mqtt_topic_heartbeat, device_id);
    strcat(mqtt_topic_heartbeat, jsonConfig["mqtt_topic_heartbeat"]);
    strcat(mqtt_topic_logs, mqtt_topic);
    strcat(mqtt_topic_logs, device_id);
    strcat(mqtt_topic_logs, jsonConfig["mqtt_topic_logs"]);    
    strcat(mqtt_topic_logs_dispenser, mqtt_topic);
    strcat(mqtt_topic_logs_dispenser, device_id);
    strcat(mqtt_topic_logs_dispenser, jsonConfig["mqtt_topic_logs_dispenser"]);
    strcat(mqtt_topic_logs_errors, mqtt_topic);
    strcat(mqtt_topic_logs_errors, device_id);
    strcat(mqtt_topic_logs_errors, jsonConfig["mqtt_topic_logs_errors"]); 
    strcat(mqtt_topic_commands, mqtt_topic);
    strcat(mqtt_topic_commands, device_id);
    strcat(mqtt_topic_commands, jsonConfig["mqtt_topic_commands"]);
    strcat(mqtt_topic_messages, mqtt_topic);
    strcat(mqtt_topic_messages, device_id);
    strcat(mqtt_topic_messages, jsonConfig["mqtt_topic_messages"]);
    strcat(mqtt_topic_dispenser, mqtt_topic);
    strcat(mqtt_topic_dispenser, device_id);
    strcat(mqtt_topic_dispenser, jsonConfig["mqtt_topic_dispenser"]);
    strcat(mqtt_topic_schedule, mqtt_topic);
    strcat(mqtt_topic_schedule, device_id);
    strcat(mqtt_topic_schedule, jsonConfig["mqtt_topic_schedule"]);
    strcat(mqtt_topic_config, mqtt_topic);
    strcat(mqtt_topic_config, device_id);
    strcat(mqtt_topic_config, jsonConfig["mqtt_topic_config"]);                  
    strlcpy(ntp_server, jsonConfig["ntp_server"]      | "", sizeof(ntp_server));

    file.close();

    Serial.println(F("\nLendo config:"));
    serializeJsonPretty(jsonConfig, Serial);
    Serial.println("");

    return true;
  }
}

boolean configSave() {
  // Grava configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;

  File file = SPIFFS.open(F(config_file), "w+");
  if (file) {
    // Atribui valores ao JSON e grava
    jsonConfig["device_id"]    = device_id;
    jsonConfig["wifi_ssid"]   = wifi_ssid;
    jsonConfig["wifi_password"]  = wifi_password;
    jsonConfig["mqtt_server"]  = mqtt_server;
    jsonConfig["mqtt_port"]    = mqtt_port;
    jsonConfig["mqtt_topic"]    = mqtt_topic;
    jsonConfig["mqtt_topic_heartbeat"]    = mqtt_topic_heartbeat;
    jsonConfig["mqtt_topic_logs"]    = mqtt_topic_logs;
    jsonConfig["mqtt_topic_logs_dispenser"]    = mqtt_topic_logs_dispenser;
    jsonConfig["mqtt_topic_logs_errors"]    = mqtt_topic_logs_errors;
    jsonConfig["mqtt_topic_commands"]    = mqtt_topic_commands;
    jsonConfig["mqtt_topic_messages"]    = mqtt_topic_messages;
    jsonConfig["mqtt_topic_dispenser"]    = mqtt_topic_dispenser;
    jsonConfig["mqtt_topic_schedule"]    = mqtt_topic_schedule;
    jsonConfig["mqtt_topic_config"]    = mqtt_topic_config;
    jsonConfig.createNestedArray("registered_cards");

    serializeJsonPretty(jsonConfig, file);
    file.close();

    Serial.println(F("\nGravando config:"));
    serializeJsonPretty(jsonConfig, Serial);
    Serial.println("");

    return true;
  }
  return false;
}

boolean registeredCardsRead(char card_number[10]) {
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  char copy_payload[10];

  strncpy(copy_payload, card_number, 10);

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonArray registered_cards = jsonConfig["registered_cards"];

    for(JsonVariant v : registered_cards) {
      if(strcmp(copy_payload, v.as<const char *>()) == 0){
        file.close();
        return true;
      }
    }

      mqttPublishMessage(device_id, "Cartão não cadastrado.");
      file.close();
      return false;
  } 
}

boolean registeredCardsWrite(char card_number[10]) {
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  StaticJsonDocument<384> intermediate;

  char copy_payload[10];

  strncpy(copy_payload, card_number, 10);

  if(registeredCardsRead(copy_payload) == true){
    mqttPublishMessage(device_id, "Cartão já cadastrado.");
    return true;
  }

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonArray old_registered_cards = jsonConfig["registered_cards"];
    JsonArray intermediate_registered_cards = intermediate.to<JsonArray>();

    for(JsonVariant v : old_registered_cards) {
      intermediate_registered_cards.add(v.as<const char *>());
    }

    JsonArray new_registered_cards = jsonConfig.createNestedArray("registered_cards");

    for(JsonVariant v : intermediate_registered_cards) {
      new_registered_cards.add(v.as<const char *>());
    }
    new_registered_cards.add(copy_payload);

    serializeJsonPretty(jsonConfig, Serial);
    file.close();

    File file = SPIFFS.open(F(config_file), "w+");
    serializeJsonPretty(jsonConfig, file);
    file.close();

    mqttPublishMessage(device_id, "Cartão cadastrado com sucesso.");

    updateLastUpdate();
    mqttPublishConfig();
    return true;
  } 
}

boolean registeredCardsDelete(char card_number[10]) {
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  StaticJsonDocument<384> intermediate;

  char copy_payload[10];

  strncpy(copy_payload, card_number, 10);

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonArray old_registered_cards = jsonConfig["registered_cards"];
    JsonArray intermediate_registered_cards = intermediate.to<JsonArray>();

    for(JsonVariant v : old_registered_cards) {
      if(strcmp(copy_payload, v.as<const char *>()) != 0){
        intermediate_registered_cards.add(v.as<const char *>());
      }
      else{
        mqttPublishMessage(device_id, "Cartão deletado.");
      }
    }

    JsonArray new_registered_cards = jsonConfig.createNestedArray("registered_cards");

    for(JsonVariant v : intermediate_registered_cards) {
      new_registered_cards.add(v.as<const char *>());
    }

    file.close();

    File file = SPIFFS.open(F(config_file), "w+");
    serializeJsonPretty(jsonConfig, file);
    file.close();

    updateLastUpdate();
    mqttPublishConfig();
    return true;
  } 
}

boolean dispenserScheduleRead() {
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  char * timestamp;
  int total_seconds_timestamp, total_seconds_schedule;
  int duration;
  int minimum_positive_duration = 86400;
  int minimum_negative_duration = 0;
  int timer_period;
  int food_weight = 0;
  bool activationtype = true;

  timestamp = getTimeRTC();
  total_seconds_timestamp = timestampDayToInt(timestamp);
  
  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonObject dispenser_schedule = jsonConfig["dispenser_schedule"];
    
    for (JsonPair kv : dispenser_schedule) {
        total_seconds_schedule = timestampHourToInt(kv.key().c_str());
        duration = total_seconds_schedule - total_seconds_timestamp;
        if(duration >= 0){
          if(duration <= minimum_positive_duration){
            minimum_positive_duration = duration;
            food_weight = kv.value().as<int>();
          }
        }
        else if(duration < 0 && minimum_positive_duration == 86400){
          if(duration <= minimum_negative_duration){
            minimum_negative_duration = duration;
            food_weight = kv.value().as<int>();
          }
        }
    }

    if(minimum_positive_duration < 86400){
      timer_period = minimum_positive_duration;
    }
    else{
      timer_period = 86400 + minimum_negative_duration;
    }

    if(timer_period <= 600){
      Serial.println("Programando abertura dispenser.");
      Serial.println(timer_period);
      Serial.println(food_weight);
      timer_period = timer_period * 1000;
      xTimerChangePeriod(xTimerDispenserSchedule, pdMS_TO_TICKS(timer_period), 0);
      xTimerStart(xTimerDispenserSchedule, 0);
      xQueueReset(xFilaDispenserTime);
      xQueueReset(xFilaActivationTypeDispenser);
      xQueueSendToBack(xFilaDispenserTime, &food_weight, 0);
      xQueueSendToBack(xFilaActivationTypeDispenser, &activationtype, 0);
    }
    else{
      Serial.println("Tempo de abertura do dispenser superior ao limite.");
      Serial.println(timer_period);
      Serial.println(food_weight);
    }

    free(timestamp);
    file.close();
    return true;
  } 
}

boolean dispenserScheduleWrite(char *timestamp, int food_weight){
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  StaticJsonDocument<384> intermediate;

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonObject old_dispenser_schedule = jsonConfig["dispenser_schedule"];
    JsonObject intermediate_dispenser_schedule = intermediate.to<JsonObject>();

    for (JsonPair kv : old_dispenser_schedule) {
        intermediate_dispenser_schedule[kv.key().c_str()] = kv.value().as<int>();
    }

    JsonObject new_dispenser_schedule = jsonConfig.createNestedObject("dispenser_schedule");

    for(JsonPair kv : intermediate_dispenser_schedule) {
      new_dispenser_schedule[kv.key().c_str()] = kv.value().as<int>();
    }
    new_dispenser_schedule[timestamp] = food_weight;

    serializeJsonPretty(new_dispenser_schedule, Serial);
    file.close();

    File file = SPIFFS.open(F(config_file), "w+");
    serializeJsonPretty(jsonConfig, file);
    file.close();

    updateLastUpdate();
    mqttPublishConfig();
    return true;
  } 
}

boolean dispenserScheduleClear(){
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonConfig;

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    JsonObject new_dispenser_schedule = jsonConfig.createNestedObject("dispenser_schedule");

    serializeJsonPretty(new_dispenser_schedule, Serial);
    file.close();

    File file = SPIFFS.open(F(config_file), "w+");
    serializeJsonPretty(jsonConfig, file);
    file.close();

    updateLastUpdate();
    mqttPublishConfig();
    return true;
  } 
}

boolean updateLastUpdate(){
  char * timestamp;
  StaticJsonDocument<JSON_SIZE> jsonConfig;

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    timestamp = getTimeRTC();

    jsonConfig["last_update"] = timestamp;

    free(timestamp);
    file.close();

    File file = SPIFFS.open(F(config_file), "w+");
    serializeJsonPretty(jsonConfig, file);
    file.close();

    return true;
  }
}

//.......................SNTP.............................
char * getTime(){
  struct tm time;
  char * buffer = (char *) malloc (sizeof(char) * 21);

  if(!getLocalTime(&time)){
    mqttPublishLogError(device_id, "Could not obtain time info");
    strcpy(buffer, "00/00/00 00:00:00");
  }
  else{
    strftime(buffer, sizeof(char) * 20, "%d/%m/%Y %H:%M:%S", &time);
  }

  return buffer;
}

char * getTimeISO(){
  struct tm time;
  char * buffer = (char *) malloc (sizeof(char) * 21);

  if(!getLocalTime(&time)){
    mqttPublishLogError(device_id, "Could not obtain time info");
    strcpy(buffer, "2021-01-01T00:00:00Z");
  }
  else{
    strftime(buffer, sizeof(char) * 21, "%Y-%m-%dT%H:%M:%SZ", &time);
  }

  return buffer;
}

int timestampDayToInt(char * time){
  int day, month, year;
  int hour, minute, second;
  int total_seconds;

  sscanf(time, "%d/%d/%d %d:%d:%d", &day, &month, &year, &hour, &minute, &second);

  total_seconds = hour*3600 + minute*60 + second;

  return total_seconds;
}

int timestampHourToInt(const char * time){
  int hour, minute, second;
  int total_seconds;

  sscanf(time, "%d:%d:%d", &hour, &minute, &second);

  total_seconds = hour*3600 + minute*60 + second;

  return total_seconds;
}

void syncTime(){
  struct tm time;

  if(!getLocalTime(&time)){
    mqttPublishLogError(device_id, "Could not obtain time info");
    return;
  }
  rtc.adjust(DateTime(time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec));
}

char * getTimeRTC(){
  char * timestamp = (char *) malloc (sizeof(char) * 23);
  char buffer[10];

  if(rtc.now().day() <= 9){ strcat(timestamp, "0"); }
  strcpy(timestamp, itoa(rtc.now().day(), buffer, 10));
  strcat(timestamp, "/");
  if(rtc.now().month() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().month(), buffer, 10));
  strcat(timestamp, "/");
  strcat(timestamp, itoa(rtc.now().year(), buffer, 10));
  strcat(timestamp, " ");
  if(rtc.now().hour() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().hour(), buffer, 10));
  strcat(timestamp, ":");
  if(rtc.now().minute() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().minute(), buffer, 10));
  strcat(timestamp, ":");
  if(rtc.now().second() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().second(), buffer, 10));

  return timestamp;
}

char * getTimeISORTC(){
  char * timestamp = (char *) malloc (sizeof(char) * 23);
  char buffer[10];

  strcpy(timestamp, itoa(rtc.now().year(), buffer, 10));
  strcat(timestamp, "-");
  if(rtc.now().month() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().month(), buffer, 10));
  strcat(timestamp, "-");
  if(rtc.now().day() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().day(), buffer, 10));
  strcat(timestamp, "T");
  if(rtc.now().hour() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().hour(), buffer, 10));
  strcat(timestamp, ":");
  if(rtc.now().minute() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().minute(), buffer, 10));
  strcat(timestamp, ":");
  if(rtc.now().second() <= 9){ strcat(timestamp, "0"); }
  strcat(timestamp, itoa(rtc.now().second(), buffer, 10));
  strcat(timestamp, "Z");

  return timestamp;
}

//.......................WiFi.............................
void setupWiFi(char ssid[30], char pwd[30], int max_tries, int delay){
  int tries_connect;

  WiFi.begin(ssid, pwd);
  for(tries_connect = 0; tries_connect <= max_tries; tries_connect ++){
    if(WiFi.status() == WL_CONNECTED){

      Serial.println("Conectado");
      Serial.println("IP: ");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("Conectando ao WiFi..");
    vTaskDelay(pdMS_TO_TICKS(delay)); 
  }
  
  mqttPublishLogError(device_id, "Falha ao conectar ao WiFi.");
  return;
}

//.......................MQTT.............................
void mqttCallback(char* topic, byte* payload, unsigned int length){
  char card_number[] = "0000000000";
  bool activationtype = false;
  char payload_char[30];

  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  strlcpy(payload_char, (const char*) payload, length + 1);

  if (strcmp(topic, mqtt_topic_commands)==0){
    if (strcmp(payload_char, "open_cover")==0){
      xQueueOverwrite(xFilaCardNumber, &card_number);
      xQueueOverwrite(xFilaActivationType, &activationtype);
      vTaskResume(taskServoTampaHandle);
      xSemaphoreGive(xSemaphoreOpenCover);
    }
    else if(strcmp(payload_char, "close_cover")==0){
      xSemaphoreGive(xSemaphoreCloseCover);
      xSemaphoreTake(xSemaphoreOpenCover,pdMS_TO_TICKS(10));
    }
    else if(strcmp(payload_char, "master_mode_on")==0){
      xSemaphoreGive(xSemaphoreMasterMode);
      blinkRGBColor(0, 0, 255);
      mqttPublishMessage(device_id, "Entrando no modo mestre.");
      xTimerStart(xTimerMasterMode,0);
    }
    else if(strcmp(payload_char, "restart")==0){
      mqttPublishMessage(device_id, "Reiniciando pote.");
      vTaskDelay(1000);
      ESP.restart();
    }
    else if(strcmp(payload_char, "send_config")==0){
      mqttPublishConfig();
    }
  }
  else if(strcmp(topic, mqtt_topic_dispenser)==0){
      int dispenser_time = atoi((const char*)payload);
      bool activationtype = false;

      xQueueSendToFront(xFilaDispenserTime, &dispenser_time, 0);
      xQueueSendToFront(xFilaActivationTypeDispenser, &activationtype, 0);          
      vTaskResume(taskServoDispenserHandle);
      xSemaphoreGive(xSemaphoreOpenDispenser);
  }
  else if(strcmp(topic, mqtt_topic_schedule)==0){
    if ((char)payload[0] == '0'){
      dispenserScheduleClear();
    }
    else{
      StaticJsonDocument<48> doc;
      int food_weight = 0;
      char time[9];
      deserializeJson(doc, payload);

      strncpy(time, doc["timestamp"], sizeof(time));
      food_weight = doc["food_weight"];

      dispenserScheduleWrite(time, food_weight);
      vTaskResume(taskDispenserScheduleCheckHandle);
    }
  }
}

void mqttConnectionCheck(int max_tries, int delay){     
int tries_connect;

  for(tries_connect = 0; tries_connect <= max_tries; tries_connect ++){              
    if(client.connected()){
        return;
    }
    
    Serial.print("Conectando ao MQTT...");

    if (client.connect("ESP32Client")) {
      Serial.println("Conectado");        
      client.subscribe(mqtt_topic_commands);
      client.subscribe(mqtt_topic_dispenser);
      client.subscribe(mqtt_topic_schedule);
    }
    else {
      Serial.print("Erro:");
      Serial.print(client.state());
      Serial.println(" reconectando em 5 segundos");
      
      vTaskDelay(pdMS_TO_TICKS(delay)); 
    }
  }
  
  mqttPublishLogError(device_id, "Falha ao conectar ao MQTT.");
  ESP.restart();
  return;
}

void mqttPublishLog(char *id, char *time_stamp, long duration_open, long food_weight, char *card_payload, bool activation_type){
  char message[170];

  StaticJsonDocument<210> doc;

  doc["device_id"] = id;
  doc["timestamp"] = time_stamp;
  doc["duration"] = duration_open;
  doc["food_weight"] = food_weight;
  doc["card_number"] = card_payload;
  doc["activation_type"] = activation_type;

  size_t n = serializeJson(doc, message);
  Serial.println(message);

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    client.publish(mqtt_topic_logs, message, n);
  }
  else{
    writeFile(message, logs_cover, true);
  }
}

void mqttPublishLogDispenser(char *id, char *time_stamp, long food_weight, bool activation_type){
  char message[170];

  StaticJsonDocument<210> doc;

  doc["device_id"] = id;
  doc["timestamp"] = time_stamp;
  doc["food_weight"] = food_weight;
  doc["activation_type"] = activation_type;

  size_t n = serializeJson(doc, message);
  Serial.println(message);

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    client.publish(mqtt_topic_logs_dispenser, message, n);
  }
  else{
    writeFile(message, logs_dispenser, true);
  }
}

void mqttPublishLogError(char *id, const char *error){
  char message[170];
  char *timestamp;

  StaticJsonDocument<210> doc;

  timestamp = getTimeISORTC();

  doc["device_id"] = id;
  doc["timestamp"] = timestamp;
  doc["error"] = error;

  size_t n = serializeJson(doc, message);
  Serial.println(error);

  free(timestamp);

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    client.publish(mqtt_topic_logs_errors, message, n);
  }
  else{
    writeFile(message, logs_errors, true);
  }
}

boolean mqttPublishConfig(){
  // Lê configuração
  char message[800];
  StaticJsonDocument<JSON_SIZE> jsonConfig;
  StaticJsonDocument<400> config;
  char last_update_config[30];
  char device_id_config[30];

  File file = SPIFFS.open(F(config_file), "r");
  if (deserializeJson(jsonConfig, file)) {
    // Falha na leitura, assume valores padrão
    configReset();
    mqttPublishLogError(device_id, "Falha lendo CONFIG, assumindo valores padrão.");
    return false;
  } else {
    strlcpy(last_update_config, jsonConfig["last_update"]      | "", sizeof(last_update_config));
    strlcpy(device_id_config, jsonConfig["device_id"]      | "", sizeof(device_id_config));
    config["last_update"] = last_update_config;
    config["device_id"] = device_id_config;

    JsonArray registered_cards = jsonConfig["registered_cards"];
    JsonArray registered_cards_config = config.createNestedArray("registered_cards");

    for(JsonVariant v : registered_cards) {
        registered_cards_config.add(v.as<const char *>());
    }

    JsonObject dispenser_schedule = jsonConfig["dispenser_schedule"];
    JsonObject dispenser_schedule_config = config.createNestedObject("dispenser_schedule");

    for (JsonPair kv : dispenser_schedule) {
        dispenser_schedule_config[kv.key().c_str()] = kv.value().as<int>();
    }

    size_t n = serializeJson(config, message);
    Serial.println(message);

    client.publish(mqtt_topic_config, message, n);

    file.close();
    return true;
  }
}

void mqttPublishMessage(char * id, const char * message){
  char mqtt_message[170];
  char *timestamp;

  StaticJsonDocument<210> doc;

  timestamp = getTimeISORTC();

  doc["device_id"] = id;
  doc["timestamp"] = timestamp;
  doc["message"] = message;

  size_t n = serializeJson(doc, mqtt_message);

  free(timestamp);
  Serial.println(message);

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    client.publish(mqtt_topic_messages, mqtt_message, n);
  }
}

void mqttPublishHeartbeat(char * id){
  char message[170];
  char *timestamp;

  StaticJsonDocument<210> doc;

  timestamp = getTimeISORTC();

  doc["device_id"] = id;
  doc["timestamp"] = timestamp;

  size_t n = serializeJson(doc, message);

  free(timestamp);

  if(WiFi.status() == WL_CONNECTED && client.connected()){
    client.publish(mqtt_topic_heartbeat, message, n);
  }
}

//.......................TXT.............................
bool writeFile(String values, String pathFile, bool appending) {
  char *mode = "w"; //open for writing (creates file if it doesn't exist). Deletes content and overwrites the file.
  if (appending) mode = "a"; //open for appending (creates file if it doesn't exist)
  Serial.println("- Writing file: " + pathFile);
  Serial.println("- Values: " + values);
  SPIFFS.begin(true);
  File wFile = SPIFFS.open(pathFile, mode);
  if (!wFile) {
    mqttPublishLogError(device_id, "- Failed to write file.");
    return false;
  } else {
    wFile.println(values);
    Serial.println("- Written!");
  }
  wFile.close();
  return true;
}

String readFile(String pathFile) {
  Serial.println("- Reading file: " + pathFile);
  SPIFFS.begin(true);
  File rFile = SPIFFS.open(pathFile, "r");
  String values;
  if (!rFile) {
    mqttPublishLogError(device_id, "- Failed to open file.");
  } else {
    while (rFile.available()) {
      values += rFile.readString();
    }
    Serial.println("- File values: " + values);
  }
  rFile.close();
  return values;
}

void readSendFile(String pathFile, char * mqtt_topic) {
  Serial.println("- Reading file: " + pathFile);
  SPIFFS.begin(true);
  File rFile = SPIFFS.open(pathFile, "r");
  String values;
  char message [170];
  if (rFile) {
    while (rFile.available()) {
      values = rFile.readStringUntil('\n');
      values.toCharArray(message, 170);
      client.publish(mqtt_topic, message);
      vTaskDelay(100);
    }
  }
  rFile.close();

  if (SPIFFS.remove(pathFile)) {
    Serial.println("- File deleted!");
  }
}

//.......................RGB.............................
void setRGBColor(int red, int green, int blue){
  //analogWrite(RGBRedPin, red);
  //analogWrite(RGBGreenPin, green);
  //analogWrite(RGBBluePin, blue);
}

void blinkRGBColor(int red, int green, int blue){
  vTaskResume(taskRGBHandle);
  xQueueOverwrite(xFilaRGBRed, &red);
  xQueueOverwrite(xFilaRGBGreen, &green);
  xQueueOverwrite(xFilaRGBBlue, &blue);
}

//.......................Stepper.............................
void openCover(){
  digitalWrite(openCoverPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(openCoverPin, HIGH);
  xSemaphoreTake(xSemaphoreACKStepper,portMAX_DELAY);
}

void closeCover(){
  digitalWrite(closeCoverPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(closeCoverPin, HIGH);
  xSemaphoreTake(xSemaphoreACKStepper,portMAX_DELAY);
}

void rotateDispenser(){
  digitalWrite(dispenserPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(dispenserPin, HIGH);
  xSemaphoreTake(xSemaphoreACKStepper,portMAX_DELAY);
}