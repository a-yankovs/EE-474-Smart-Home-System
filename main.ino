Fin

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include <Wire.h>
#include <string.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include "freertos/queue.h"
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// Generate random Service and Characteristic UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID        "76aba4a6-eb39-4a35-85b2-a6fbb20999cf"
#define CHARACTERISTIC_UUID "2cdd5742-0b1a-4758-8096-cbc516b21eac"

// pin definitions 
#define LCD_ADDR 0x27                 ///< I2C address of the LCD
#define ENABLE_BIT 0x04               ///< LCD enable bit for command latch
#define SDA 8                         ///< I2C SDA pin
#define SCL 9                         ///< I2C SCL pin
#define SERVO_PIN_ON 4 
#define SERVO_PIN_OFF 5 
#define TRIG_PIN 17
#define ECHO_PIN 16
#define BUTTON_PIN 2

// Servo position/angle definitions 
#define ON_POS_ACTIVE   180
#define ON_POS_REST      80
#define OFF_POS_ACTIVE  170
#define OFF_POS_REST     80

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);   ///< LCD object for 16x2 screen
Servo switchOn;
Servo switchOff;

QueueHandle_t commandQueue = NULL;
QueueHandle_t espNowTx;
QueueHandle_t espNowRx;
TimerHandle_t alarmTimer = NULL;


static const int CHANNEL = 6;
static const uint8_t ROOM1_ID = 1; 
static const uint8_t ROOM2_ID = 2;


volatile bool person_present = false; 

BaseType_t lightBurst = NULL;

// task handles for tasks 
TaskHandle_t lightOnOffHandle = NULL; 
TaskHandle_t usSenseHandle = NULL;
TaskHandle_t rxHandle = NULL; 
TaskHandle_t txHandle = NULL; 
TaskHandle_t burstLightHandle = NULL;  

enum Cmd { 
  CMD_ON = 1, 
  CMD_OFF = 2
};


enum : uint8_t {
  MT_CMD = 0, 
  MT_ACK = 1
};

enum : uint8_t {
  CMD_LIGHT = 0
};

enum : uint8_t {
  ACT_OFF = 0, 
  ACT_ON = 1
};

enum {
  REASON_EXIT = 0, 
  REASON_ENTRY = 1, 
  REASON_ALARM = 2, 
  REASON_MANUAL = 3
};
  
volatile bool isLightOn = false;
static bool bleLightOn = false; // don't need this 
volatile bool clockSet = false;
volatile int8_t clkHours = -1; 
volatile int8_t clkMinutes = -1; 
volatile int8_t clkSeconds = 0;
volatile int8_t alarmHours = -1; 
volatile int8_t alarmMinutes = -1;
volatile int32_t lastFiredMinute = -1;

uint8_t ROOM2_MAC[6] = {0xFC, 0x01, 0x2C, 0xDB, 0xF6, 0x70};

BLECharacteristic *globalMsg = NULL;
BLECharacteristic *alarmChar = NULL;

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

//enqueues esp-now message to send it 
static void espNowSend(const uint8_t mac[6], const Msg& m) {
  TxItem text;
  memcpy(text.mac, mac, 6);
  text.msg = m;
  xQueueSend(espNowTx, &text, 0);
}

void Task_espNowSend(void *pvParameters) {
  TxItem t;
  while(1) {
    if(xQueueReceive(espNowTx, &t, portMAX_DELAY)){
      esp_now_send(t.mac, (const uint8_t*)&t.msg, sizeof(Msg));
    } 
  }
}

void Task_espNowReceive(void *pvParameters) {
  RxItem r;
  while(1) {
    if(xQueueReceive(espNowRx, &r, portMAX_DELAY)) {
      if(r.msg.mtype == MT_CMD && r.msg.cmd == CMD_LIGHT) {
        Cmd c = (r.msg.action == ACT_ON) ? CMD_ON : CMD_OFF; // change this 
        xQueueSend(commandQueue, &c, 0);
      }
    }
  }
}

// toggles lights on once and servos return to default position 
void lightsOnOnce(void *pvParameters) {
  switchOn.write(ON_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOn.write(ON_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = true;
}

// toggles lights off once and servos return to default position 
void lightsOffOnce(void *pvParameters) {
  switchOff.write(OFF_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOff.write(OFF_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = false;
}

// toggles lights on/off 5 times 
void burstLight(void *pvParameters) {
  for(int i = 0; i < 5; i++) {
    Cmd command = CMD_ON;
    if(commandQueue) {
      xQueueSend(commandQueue, &command, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(400));
    command = CMD_OFF;
    if(commandQueue) {
      xQueueSend(commandQueue, &command, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  vTaskDelete(NULL); // ?
}


void Task_lightControl(void *pvParameters) {
  Cmd cmd;
  while(1) {
    if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd){
        case CMD_ON:
          if (!isLightOn){ 
            lcd.clear();
            lcd.setCursor(0,0); 
            lcd.print("Status:");
            lcd.setCursor(0,1); 
            lcd.print("Lights ON");
            lightsOnOnce(NULL);
          }
          if(globalMsg){
            globalMsg -> setValue("ACK:ON");
            globalMsg -> notify();
          }
          break;
        case CMD_OFF:
          if(isLightOn) {
            lcd.clear();
            lcd.setCursor(0,0); 
            lcd.print("Status:");
            lcd.setCursor(0,1); 
            lcd.print("Lights OFF");
            lightsOffOnce(NULL);
          }
          if(globalMsg) {
            globalMsg -> setValue("ACK:OFF");
            globalMsg -> notify();
          }
          break;
      }
    }
  }
}

void Task_ultrasonicSense(void *pvParameters){ 
  while(1){ 
    digitalWrite(TRIG_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    digitalWrite(TRIG_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // max wait time 30ms
    float distance_cm = duration * 0.0343 / 2; // 0.0343cm/µs = speed of sound 

    if (duration == 0) {
      Serial.println(F("Failed to capture distance"));
    } else {
      Serial.printf("Distance: %.2f cm\n", distance_cm);
      if (distance_cm <= 50 ) { 

        bool wasInRoom1 = person_present; // redundant 
        bool nowInRoom1 = !person_present;
        if(wasInRoom1 && !nowInRoom1) { //Exiting Room1 going to Room2
          Serial.println("Person exiting room 1, entering room 2");
          Cmd c = CMD_OFF;
          xQueueSend(commandQueue, &c, 0);
          Msg m = {MT_CMD, ROOM1_ID, ROOM2_ID, CMD_LIGHT, ACT_ON, REASON_EXIT, millis()};
          espNowSend(ROOM2_MAC, m);
        } else if(!wasInRoom1 && nowInRoom1) { //Exiting Room2 entering Room1
          Serial.println("Person exiting room 2, entering room 1");
          Cmd c = CMD_ON;
          xQueueSend(commandQueue, &c, 0);
          Msg m = {MT_CMD, ROOM1_ID, ROOM2_ID, CMD_LIGHT, ACT_OFF, REASON_ENTRY, millis()};
          espNowSend(ROOM2_MAC, m);
        }
        person_present = nowInRoom1; 
        vTaskDelay(pdMS_TO_TICKS(2000)); 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}

void setAlarm(TimerHandle_t) {
  if(!clockSet) {
    return;
  }
  clkSeconds++;

  if(clkSeconds >= 60) {
    clkSeconds = 0;
    clkMinutes++;
  } 

  if(clkMinutes >= 60) {
    clkMinutes = 0;
    clkHours++;
  }

  if(clkHours >= 24) {
    clkHours = 0;
  }

  //Alarm matching
  if(alarmHours >= 0 && alarmMinutes >= 0) {
    int currMinuteIndex = clkHours * 60 + clkMinutes;

    if(currMinuteIndex != lastFiredMinute && clkHours == alarmHours && clkMinutes == alarmMinutes) {
      lastFiredMinute = currMinuteIndex;

      lightBurst = xTaskCreatePinnedToCore(burstLight, "Burst", 2048, NULL, 2, &burstLightHandle, 1);

      if (alarmChar) {
        char buf[32];
        snprintf(buf, sizeof(buf), "ALARM:FIRE %02d:%02d", clkHours, clkMinutes);
        alarmChar->setValue(buf);
        alarmChar->notify();
      }
    }
  }
}


class MyCallbacks : public BLECharacteristicCallbacks {
  // Function that takes in a BLE signal from the LightBlue app, and if the message is 
  void onWrite(BLECharacteristic *pCharacteristic) override { 
    String value = pCharacteristic->getValue(); // get the value that was written on the lightBlue app 
    if (value.length() == 0){ // error failsafe 
      return;
    }
    Cmd cmd;
    if (value == "ON" || value == "on") {
      cmd = CMD_ON; // trun on lifhts 
      bleLightOn = true;
    } else if (value == "OFF" || value == "off") {
      cmd = CMD_OFF;
      bleLightOn = false;
    } else { 
      return;
    }
    
    if (commandQueue){ 
      xQueueSend(commandQueue, &cmd, 0); // send command to queue 
    }
  }
};

class AlarmCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String value = c->getValue(); value.trim();
    if (!value.length()) { 
      return;
    }

    //Set Time - prints error on BLE app if time is out of bounds 
    if (value.startsWith("TIME ")) {
      int space = value.indexOf(' '); 
      String t = value.substring(space + 1);
      int c1 = t.indexOf(':'), c2 = t.indexOf(':', c1+1);
      if (c1 < 0) { 
        c->setValue("TIME:ERR"); 
        c->notify(); 
        return; 
      }
      int hours = t.substring(0, c1).toInt();
      int mins = (c2 < 0) ? t.substring(c1+1).toInt() : t.substring(c1 + 1, c2).toInt(); // what tf 
      int secs = (c2 < 0) ? 0 : t.substring(c2 + 1).toInt(); // same here 
      if (hours < 0||hours > 23||mins < 0||mins > 59||secs < 0||secs > 59) { 
        c->setValue("TIME:ERR"); 
        c->notify(); 
        return; 
      }
      clkHours = hours; clkMinutes = mins; clkSeconds = secs; clockSet = true;
      c->setValue("TIME:OK"); c->notify();
      return;
    }

    //Set Alarm
    if (value.startsWith("ALARM ")) {
      String t = value.substring(6); t.trim();
      int colon = t.indexOf(':');
      if (colon < 0) { 
        c->setValue("ALARM:ERR"); 
        c->notify(); 
        return; 
      }
      int hours = t.substring(0, colon).toInt();
      int mins = t.substring(colon + 1).toInt();
      if (hours <0||hours >23||mins <0||mins >59) { 
        c->setValue("ALARM:ERR"); 
        c->notify(); 
        return; 
      }
      alarmHours = hours; 
      alarmMinutes = mins;
      lastFiredMinute = -1;   // allow next fire at that minute
      c->setValue("ALARM:SET"); 
      c->notify();
      char buf[17];
      snprintf(buf, sizeof(buf), "Alarm %02d:%02d", hours, mins);
      return;
    }

    //Clear Alarm
    if (value.equalsIgnoreCase("ALARM CLEAR")) {
      alarmHours = -1; 
      alarmMinutes = -1;
      lastFiredMinute = -1;
      c->setValue("ALARM:CLEARED"); 
      c->notify();
      return;
    }

    c->setValue("UNKNOWN"); 
    c->notify();
  }
};

void Task_ButtonSleep(void *pvParameters){ 
// add all of the code for the button here 
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true); //unlocks channel config
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE); //sets channel, same for other esp
  esp_wifi_set_promiscuous(false); //locks channel config
  espNowTx = xQueueCreate(16, sizeof(TxItem));
  espNowRx = xQueueCreate(16, sizeof(RxItem));
  if (esp_now_init() != ESP_OK) { 
    Serial.println(F("ESP-NOW init fail")); 
  // while(true){}
  }

  //callback registers to setup receving and sending data
  esp_now_register_recv_cb([](const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if(len != sizeof(Msg)) {
      return;
    }
    RxItem r;
    // send mac adress to the info struct 
    memcpy(r.mac, info->src_addr, 6);
    // copy raw bytes from data into msg field
    memcpy(&r.msg, data, sizeof(Msg));
    r.rssi = -127; // RSSI placeholder - Received Signal Strength Indicator - how strong the signal received is 
    BaseType_t hpw = pdFALSE;  // hpw = high priority task woken 
    xQueueSendFromISR(espNowRx, &r, &hpw);
    if (hpw) {
      portYIELD_FROM_ISR();
    }
  });

  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t s){});
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, ROOM2_MAC, 6);
  p.channel = CHANNEL;
  p.encrypt = false;
  esp_now_add_peer(&p);

  // ultrasonic sensor setup 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // servo setup 
  switchOn.setPeriodHertz(50);             // Set PWM frequency to 50Hz 
  switchOff.setPeriodHertz(50);
  switchOn.attach(SERVO_PIN_ON, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths
  switchOff.attach(SERVO_PIN_OFF, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths
  switchOn.write(180);
  switchOff.write(80);                       
  vTaskDelay(pdMS_TO_TICKS(500));

  // BLE setup 
  BLEDevice::init("SmartLights");

  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  globalMsg = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  globalMsg->setCallbacks(new MyCallbacks());
  globalMsg->setValue("Ready!");

  // Alarm characteristic
  alarmChar = pService->createCharacteristic(
    "eab9017e-93ba-431e-a051-3d071fe781ed",
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  alarmChar->setCallbacks(new AlarmCallbacks());
  alarmChar->setValue("ALARM:READY");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

// lcd setup 
  Wire.begin(SDA, SCL);
  lcd.init();
  lcd.backlight();

  // setting up alarm 
  alarmTimer = xTimerCreate("Wake Up Alarm", pdMS_TO_TICKS(1000), pdTRUE, NULL, setAlarm);
  xTimerStart(alarmTimer, 0);

  // creating queue for lights on/off commands 
  commandQueue = xQueueCreate(8, sizeof(Cmd));   
  // creating all of our tasks                            
  xTaskCreatePinnedToCore(Task_lightControl, "OnOff", 3072, NULL, 2, &lightOnOffHandle, 1);
  xTaskCreatePinnedToCore(Task_ultrasonicSense, "UltraSense", 3072, NULL, 1, &usSenseHandle, 0);
  xTaskCreatePinnedToCore(Task_espNowSend, "radioTx", 3072, NULL, 3, &rxHandle, 1); // pulls from espNowTx→ esp_now_send
  xTaskCreatePinnedToCore(Task_espNowReceive, "radioRx", 3072, NULL, 2, &txHandle, 1); // pulls from espNowRx → maps to commandQueue
  xTaskCreate(Task_ButtonSleep, "Button Task", 3072, NULL, 5, NULL); 

}

// empty loop 
void loop() {}
