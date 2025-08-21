// Code for Node 2 - Secondary Room that takes in input from the Main Node 

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <Arduino.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <Wire.h>
#include <string.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#define SERVO_PIN_ON 4 //NO CHANGE
#define SERVO_PIN_OFF 5 //NO CHANGE

#define ON_POS_ACTIVE   180
#define ON_POS_REST      80
#define OFF_POS_ACTIVE  170
#define OFF_POS_REST     80

Servo switchOn;
Servo switchOff;

QueueHandle_t commandQueue = NULL;
QueueHandle_t qRadioTx;
QueueHandle_t qRadioRx;
enum Cmd { CMD_ON = 1, CMD_OFF = 2};
uint8_t ROOM1_MAC[6] = {};
static const int CHANNEL = 6;    
volatile bool isLightOn = false;
enum : uint8_t { MT_CMD=0, MT_ACK=1 };
enum : uint8_t { CMD_LIGHT=0 };
enum : uint8_t { ACT_OFF=0, ACT_ON=1 };



typedef struct __attribute__((packed)) {
  uint8_t  mtype;    // MT_CMD | MT_ACK
  uint8_t  src;      // 1=Room1, 2=Room2
  uint8_t  dst;      // target room id
  uint8_t  cmd;      // CMD_LIGHT
  uint8_t  action;   // ACT_ON/ACT_OFF
  uint8_t  reason;   // REASON_EXIT/ENTRY
  uint32_t ts_ms;    // millis() at send time
} Msg;

typedef struct  {
  uint8_t mac[6]; //destination idef
  Msg msg; //payload
} TxItem;

typedef struct {
  uint8_t mac[6];
  Msg msg;
  int8_t  rssi;
} RxItem;

static inline void radio_enqueue_send(const uint8_t mac[6], const Msg& m) {
  TxItem t;
  memcpy(t.mac, mac, 6);
  t.msg = m;
  xQueueSend(qRadioTx, &t, 0);
}

void radioTxTask(void *pvParameters) {
  TxItem t;
  for(;;) {
    if(xQueueReceive(qRadioTx, &t, portMAX_DELAY)){
      esp_now_send(t.mac, (const uint8_t*)&t.msg, sizeof(Msg));
    } 
  }
}

void radioRxTask(void *pvParameters) {
  RxItem r;
  for(;;) {
    if(xQueueReceive(qRadioRx, &r, portMAX_DELAY)) {
      if(r.msg.mtype==MT_CMD && r.msg.cmd==CMD_LIGHT) {
        Cmd c = (r.msg.action == ACT_ON) ? CMD_ON : CMD_OFF;
        xQueueSend(commandQueue, &c, 0);
      }
    }
  }
}

// toggles lights on and servos return to default position 
void doLightsOnOnce(void *pvParameters) {
  switchOn.write(ON_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOn.write(ON_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = true;
}

// toggles lights off and servos return to default position 
void doLightsOffOnce(void *pvParameters) {
  switchOff.write(OFF_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOff.write(OFF_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = false;
}

void onOffSystem(void *pvParameters) {
  Cmd cmd;
  while(1) {
    if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd)
      {
      case CMD_ON:
        if (!isLightOn){ 
          doLightsOnOnce(NULL);
        }
        break;
      case CMD_OFF:
        if(isLightOn) {
          doLightsOffOnce(NULL);
        }
        break;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true); //unlocks channel config
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE); //sets channel, same for other esp
  esp_wifi_set_promiscuous(false); //locks channel config
  qRadioTx = xQueueCreate(16, sizeof(TxItem));
  qRadioRx = xQueueCreate(16, sizeof(RxItem));
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init fail"); 
  while(true){}}

  //callback registers to setup receving and sending data
  esp_now_register_recv_cb([](const uint8_t* mac, const uint8_t* data, int len) {
    if(len != sizeof(Msg)) {
      return;
    }
    RxItem r;
    memcpy(r.mac, mac, 6);
    memcpy(&r.msg, data, sizeof(Msg));
    r.rssi = -127;
    BaseType_t hpw = pdFALSE;  
    xQueueSendFromISR(qRadioRx, &r, &hpw);
    if (hpw) {
      portYIELD_FROM_ISR();
    }
  });

  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t s){});

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, ROOM1_MAC, 6);
  p.channel = CHANNEL;
  p.encrypt = false;
  esp_now_add_peer(&p);


  switchOn.setPeriodHertz(50);             // Set PWM frequency to 50Hz (standard for most servos)
  switchOff.setPeriodHertz(50);
  switchOn.attach(SERVO_PIN_ON, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths
  switchOff.attach(SERVO_PIN_OFF, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths


  switchOn.write(180);
  switchOff.write(80);                       
  vTaskDelay(pdMS_TO_TICKS(500));


  commandQueue = xQueueCreate(8, sizeof(Cmd));                              
  xTaskCreatePinnedToCore(onOffSystem, "OnOff", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(radioTxTask, "radioTx", 3072, NULL, 3, NULL, 1); // pulls from qRadioTx → esp_now_send
  xTaskCreatePinnedToCore(radioRxTask, "radioRx", 3072, NULL, 2, NULL, 1); // pulls from qRadioRx → maps to commandQueue

}

void loop() {}