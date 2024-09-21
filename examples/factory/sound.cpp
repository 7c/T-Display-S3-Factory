#include <Arduino.h>
#include "sound.h"
#include "esp32-hal-ledc.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/*
  this was ported from framework-arduinoespressif32@3.20014.231204 › cores › esp32 > Tone.cpp (https://github.com/espressif/arduino-esp32/blob/2.0.14/cores/esp32/Tone.cpp)
  the original code is licensed under LGPLv2.1, Copyright (c) 2017-2021 Arduino SA
  to debug why tone() causes display to not show, I've added Serial prints to the original code
  and essentially removed 
  ledcWriteTone(_channel, 0);
  which is the line that actually causing the display to not show
*/
 
static TaskHandle_t _tone_task = NULL;
static QueueHandle_t _tone_queue = NULL;
static uint8_t _channel = 0;

typedef enum{
  TONE_START,
  TONE_END,
  TONE_SET_CHANNEL
} tone_cmd_t;

typedef struct{
  tone_cmd_t tone_cmd;
  uint8_t pin;
  unsigned int frequency;
  unsigned long duration;
  uint8_t channel;
} tone_msg_t;



void tone_task(void*){
  tone_msg_t tone_msg;
  Serial.println("Tone task started\n");
  while(1){
    xQueueReceive(_tone_queue, &tone_msg, portMAX_DELAY);
    Serial.printf("Tone task received message\n");
    switch(tone_msg.tone_cmd){
      case TONE_START:
        Serial.printf("Task received from queue TONE_START: _pin=%d, frequency=%u Hz, duration=%d ms\n", tone_msg.pin, tone_msg.frequency, (int)tone_msg.duration);

        Serial.printf("Setup LED controll on channel %d\n", _channel);
        ledcAttachPin(tone_msg.pin, _channel);
        ledcWriteTone(_channel, tone_msg.frequency);

        if(tone_msg.duration){
          Serial.printf("Delaying for %d ms\n", tone_msg.duration);
          delay(tone_msg.duration);
            ledcDetachPin(tone_msg.pin);
            // ledcWriteTone(_channel, 0); // this line causes the display to not show
        }
        Serial.printf("Tone task finished\n");
        break;

      case TONE_END:
        Serial.printf("Task received from queue TONE_END: pin=%d\n", tone_msg.pin);
        ledcDetachPin(tone_msg.pin);
        // ledcWriteTone(_channel, 0); // this line causes the display to not show
        break;

      case TONE_SET_CHANNEL:
        Serial.printf("Task received from queue TONE_SET_CHANNEL: channel=%d", tone_msg.channel);
        _channel = tone_msg.channel;
        break;

      default: ; // do nothing
    } // switch
  } // infinite loop
}


void my_setToneChannel(uint8_t channel){
  Serial.printf("Setting tone channel to %d\n", channel);
  if(tone_init()){
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_SET_CHANNEL,
      .pin = 0, // Ignored
      .frequency = 0, // Ignored
      .duration = 0, // Ignored
      .channel = channel
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
    Serial.printf("Tone channel set to %d\n", channel);
  }
}

int tone_init(){
  if(_tone_queue == NULL){
    Serial.println("Creating tone queue");
    _tone_queue = xQueueCreate(128, sizeof(tone_msg_t));
    if(_tone_queue == NULL){
      log_e("Could not create tone queue");
      return 0; // ERR
    }
    Serial.println("Tone queue created");
  }

  if(_tone_task == NULL){
    Serial.println("Creating tone task");
    xTaskCreate(
      tone_task, // Function to implement the task
      "my_toneTask", // Name of the task
      3500,  // Stack size in words
      NULL,  // Task input parameter
      1,  // Priority of the task
      &_tone_task  // Task handle.
      );
    if(_tone_task == NULL){
      log_e("Could not create tone task");
      return 0; // ERR
    }
    Serial.println("Tone task created");
  }
  return 1; // OK
}


void my_noTone(uint8_t _pin){
  log_d("noTone was called");
  if(tone_init()){
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_END,
      .pin = _pin,
      .frequency = 0, // Ignored
      .duration = 0, // Ignored
      .channel = 0 // Ignored
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
  }
}

// parameters:
// _pin - pin number which will output the signal
// frequency - PWM frequency in Hz
// duration - time in ms - how long will the signal be outputted.
//   If not provided, or 0 you must manually call noTone to end output
void my_tone(uint8_t _pin, unsigned int frequency, unsigned long duration){
  Serial.printf("TONE: _pin=%d, frequency=%u Hz, duration=%lu ms\n", _pin, frequency, duration);
  if(tone_init()){
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_START,
      .pin = _pin,
      .frequency = frequency,
      .duration = duration,
      .channel = 0 // Ignored
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
  }
}
 