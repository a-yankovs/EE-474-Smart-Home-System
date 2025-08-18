#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "driver/gpio.h" 
#include "esp_sleep.h"

#define TRIG_PIN 10
#define ECHO_PIN 11
#define MOTION_PIN 12

volatile bool person_present = false; 
static TaskHandle_t motionTaskHandle = NULL;

void IRAM_ATTR motionISR() {
  BaseType_t taskWoken = pdFALSE;
  if (motionTaskHandle) {
    vTaskNotifyGiveFromISR(motionTaskHandle, &taskWoken);
    if (taskWoken) { 
      portYIELD_FROM_ISR();
    }
  }
}

void motionTask(void *pvParameters) {
  int lastState = -1; // unknown at boot

  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int currentState = digitalRead(PIR_PIN);
    if (currentState != lastState) {
      if (currentState == HIGH) {
        Serial.println("Motion started");
      } else {
        Serial.println("Motion ended");
      }
      lastState = currentState;
    }
  }
}


void ultrasonicTask(void *pvParameters) {
  while (1) {
    digitalWrite(TRIG_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(10); 
    digitalWrite(TRIG_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10); 
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // max wait time 30ms
    float distance_cm = duration * 0.0343 / 2; // 0.0343cm/µs​ = speed of sound 

    if (duration == 0) {
      Serial.println("Failed to capture distance");
    } else {
      Serial.printf("Distance: %.2f cm\n", distance_cm);
      if (distance_cm <= 50 ) { 
        if (person_present == false){ // need to adjust logic so that we can do this for more than one person 
          Serial.println("Entrance detected!"); 
        } else { 
          Serial.println("Exit detected!"); 
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTION_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  xTaskCreate(motionTask, "Motion Task", 2048, NULL, 1 ,&motionTaskHandle);
  xTaskCreate(ultrasonicTask, "Ultrasonic Task", 2048, NULL, 1, NULL);

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionISR, CHANGE);
}


void loop() {}

