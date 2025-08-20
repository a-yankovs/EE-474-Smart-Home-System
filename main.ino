// Main Node that receives BLE signal from user's phone and sends ESP-NOW signal to Secondary Node  

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
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_sleep.h"

// Generate random Service and Characteristic UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID        "76aba4a6-eb39-4a35-85b2-a6fbb20999cf"
#define CHARACTERISTIC_UUID "2cdd5742-0b1a-4758-8096-cbc516b21eac"

// pin definitions 
#define LCD_ADDR 0x27                 ///< I2C address of the LCD
#define ENABLE_BIT 0x04               ///< LCD enable bit for command latch
#define SDA 8                         ///< I2C SDA pin
#define SCL 9                         ///< I2C SCL pin
#define SERVO_PIN_ON 4 //NO CHANGE
#define SERVO_PIN_OFF 5 //NO CHANGE
#define TRIG_PIN 17
#define ECHO_PIN 16
#define MOTION_PIN 12
#define BUTTON_PIN 2

#define ON_POS_ACTIVE   180
#define ON_POS_REST      80
#define OFF_POS_ACTIVE  170
#define OFF_POS_REST     80

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);   ///< LCD object for 16x2 screen
Servo switchOn;
Servo switchOff;
// static TaskHandle_t sensorTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;
TimerHandle_t alarmTmr = NULL;

volatile bool person_present = false; 
volatile bool system_sleep = true; 


enum Cmd {
  CMD_ON = 1,
  CMD_OFF = 2};

volatile bool isLightOn = false;
volatile bool clockSet = false;
volatile int8_t clk_h = -1; 
volatile int8_t clk_m = -1; 
volatile int8_t clk_s = 0;
volatile int8_t alarm_h = -1, alarm_m = -1;
volatile int32_t lastFiredMinute = -1;

BLECharacteristic *globalMsg = NULL;
BLECharacteristic *alarmChar = NULL;

static void deepSleep(){
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
  vTaskDelay(pdMS_TO_TICKS(50)); 
  person_present = true; 
  esp_deep_sleep_start(); 
}

// find a better way to do this?
  // if system_sleep == true, send system to deep sleep 
  // wait for the button to be pressed, then wake system up 
  // if system_sleep == false, and button is pressed, send system to deep sleep 
void Task_buttonSleep(void *pvParameters){ 
  static uint32_t last = 0;
  while(1){ 
    if ((system_sleep == false) && (digitalRead(BUTTON_PIN) == LOW)){ 
      if ((millis() - last) > 60) {
        system_sleep = true; 
        deepSleep();  
      }
      last = millis(); 
    } 
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}
  
// toggles lights on and servos return to default position 
void doLightsOnOnce(void *arg) {
  switchOn.write(ON_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOn.write(ON_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = true;
}

// toggles lights off and servos return to default position 
void doLightsOffOnce(void *arg) {
  switchOff.write(OFF_POS_ACTIVE);
  vTaskDelay(pdMS_TO_TICKS(2000));
  switchOff.write(OFF_POS_REST);
  vTaskDelay(pdMS_TO_TICKS(2000));
  isLightOn = false;
}

// 
void burstLight(void *arg) {
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
    vTaskDelay(pdMS_TO_TICKS(400));
  }
  vTaskDelete(NULL);
}


void onOffSystem(void *arg) {
  Cmd cmd;
  while(1) {
    if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd)
      {
      case CMD_ON:
        if (!isLightOn){ 
          doLightsOnOnce(NULL);
        }
        if(globalMsg){
          globalMsg -> setValue("ACK:ON");
          globalMsg -> notify();
        }
        break;
      case CMD_OFF:
        if(isLightOn) {
          doLightsOffOnce(NULL);
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
    float distance_cm = duration * 0.0343 / 2; // 0.0343cm/µs​ = speed of sound 

    if (duration == 0) {
      Serial.println("Failed to capture distance");
    } else {
      Serial.printf("Distance: %.2f cm\n", distance_cm);
      if (distance_cm <= 50 ) { 
        if(person_present){ 
          Serial.println("Person exiting room"); 
        } else{ 
          Serial.println("Person entered room"); 
        }
        person_present = !(person_present); 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }
}

void alarmTimerCallback(TimerHandle_t) {
  if(!clockSet) return;
  clk_s++;

  if(clk_s >= 60) {
    clk_s = 0;
    clk_m++;
  } 

  if(clk_m >= 60) {
    clk_m = 0;
    clk_h++;
  }

  if(clk_h >= 24) {
    clk_h = 0;
  }

  //Alarm matching
  if(alarm_h >= 0 && alarm_m >= 0) {
    int currMinuteIndex = clk_h * 60 + clk_m;

    if(currMinuteIndex != lastFiredMinute && clk_h == alarm_h && clk_m == alarm_m) {
      lastFiredMinute = currMinuteIndex;

      xTaskCreatePinnedToCore(burstLight, "Burst", 2048, NULL, 2, NULL, 1);

      if (alarmChar) {
        char buf[32];
        snprintf(buf, sizeof(buf), "ALARM:FIRE %02d:%02d", clk_h, clk_m);
        alarmChar->setValue(buf);
        alarmChar->notify();
      }
    }
  }
}

// BLE stuff - if you write BLE on the app, you turn it on 
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String v = pCharacteristic->getValue();
    if (v.length() == 0) return;
    Cmd cmd;
    if (v == "ON" || v == "on")  cmd = CMD_ON;
    else if (v == "OFF" || v == "off") cmd = CMD_OFF;
    else return;
    
    if (commandQueue) xQueueSend(commandQueue, &cmd, 0);
  }
};


class AlarmCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue(); v.trim();
    if (!v.length()) return;

    //Set Time - prints error on BLE app if time is out of bounds 
    if (v.startsWith("TIME ")) {
      int sp = v.indexOf(' ');
      String t = v.substring(sp+1);
      int c1 = t.indexOf(':'), c2 = t.indexOf(':', c1+1);
      if (c1 < 0) { c->setValue("TIME:ERR"); c->notify(); return; }
      int h = t.substring(0, c1).toInt();
      int m = (c2 < 0) ? t.substring(c1+1).toInt() : t.substring(c1+1, c2).toInt();
      int s = (c2 < 0) ? 0 : t.substring(c2+1).toInt();
      if (h<0||h>23||m<0||m>59||s<0||s>59) { c->setValue("TIME:ERR"); c->notify(); return; }
      clk_h=h; clk_m=m; clk_s=s; clockSet=true;
      c->setValue("TIME:OK"); c->notify();
      return;
    }

    //Set Alarm
    if (v.startsWith("ALARM ")) {
      String t = v.substring(6); t.trim();
      int colon = t.indexOf(':');
      if (colon < 0) { c->setValue("ALARM:ERR"); c->notify(); return; }
      int h = t.substring(0, colon).toInt();
      int m = t.substring(colon+1).toInt();
      if (h<0||h>23||m<0||m>59) { c->setValue("ALARM:ERR"); c->notify(); return; }
      alarm_h = h; alarm_m = m;
      lastFiredMinute = -1;   // allow next fire at that minute
      c->setValue("ALARM:SET"); c->notify();
      return;
    }

    //Clear Alarm
    if (v.equalsIgnoreCase("ALARM CLEAR")) {
      alarm_h = -1; alarm_m = -1;
      lastFiredMinute = -1;
      c->setValue("ALARM:CLEARED"); c->notify();
      return;
    }

    c->setValue("UNKNOWN"); c->notify();
  }
};



void setup() {
  Serial.begin(115200);
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    system_sleep = false;
  }
  pinMode(MOTION_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  switchOn.setPeriodHertz(50);             // Set PWM frequency to 50Hz (standard for most servos)
  switchOff.setPeriodHertz(50);
  switchOn.attach(SERVO_PIN_ON, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths
  switchOff.attach(SERVO_PIN_OFF, 500, 2400);    // Attach the servo object to a pin with min/max pulse widths
  BLEDevice::init("SmartHome1");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Ctrl characteristic
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

  // Start service, then advertise
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Wire.begin(SDA, SCL);
  lcd.init();
  lcd.backlight();

  alarmTmr = xTimerCreate("Wake Up Alarm", pdMS_TO_TICKS(1000), pdTRUE, NULL, alarmTimerCallback);
  xTimerStart(alarmTmr, 0);

  switchOn.write(180);
  switchOff.write(80);                       
  vTaskDelay(pdMS_TO_TICKS(500));
  
  // pinMode(US_SENSOR, INPUT_PULLDOWN);
  // pinMode(MOTION_SENSOR, INPUT_PULLDOWN);

  commandQueue = xQueueCreate(8, sizeof(Cmd));                              
  xTaskCreatePinnedToCore(onOffSystem, "OnOff", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_ultrasonicSense, "UltraSense", 3072, NULL, 1, NULL, 0);
  xTaskCreate(Task_buttonSleep, "Button Sleep Task", 1024, NULL, 1, NULL); 
  if (system_sleep) {
    deepSleep();
  }
}

void loop() {}
