#include <Arduino.h>
#define LCD_MODULE_CMD_1
#include "OneButton.h" 
#include "lvgl.h"      
#include "factory_gui.h"
#include "pin_config.h"
#include "sound.h"
#include "setup.h"

void setup()
{
    startSetup();
    button1.attachClick([]() {
        Serial.println("Button 1 clicked");
        // original tone() function from platformio > packages > framework-arduinoespressif32@3.20014.231204 › cores › esp32 > Tone.cpp
        tone(SPEAKER_PIN, 5000, 1000);
        // after tone has finished playing, the display is kept black but all the events seems to be continue to work
        Serial.println("tone() played");
    });

    button2.attachClick([]() {
        Serial.println("Button 2 clicked");
        my_tone(SPEAKER_PIN, 5000, 1000);
        Serial.println("my_tone() played");
    });

    Serial.println("Setup complete");
}

static uint32_t last_tick;

void loop()
{
    lv_timer_handler();
    button1.tick();
    button2.tick();
    delay(3);
    if (millis() - last_tick > 100)
    {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo))
        {
            lv_msg_send(MSG_NEW_HOUR, &timeinfo.tm_hour);
            lv_msg_send(MSG_NEW_MIN, &timeinfo.tm_min);
        }
        last_tick = millis();
    }
}
