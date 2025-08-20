// Code for Node 2 - Secondary Room that takes in input from the Main Node 

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


volatile bool present = false; 

// when ESP_NOW is received, wake from deep sleep 
// need to change ESP_NOW wakeup input 
void deepSleepEspNow(){ 
    esp_sleep_enable_ext0_wakeup((gpio_num_t)ESP_NOW, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    esp_deep_sleep_start(); 
}

void setup(){ 

}

void loop(){}