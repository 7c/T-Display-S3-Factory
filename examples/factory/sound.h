#pragma once

#include "pin_config.h"

int tone_init();
void my_tone(uint8_t _pin, unsigned int frequency, unsigned long duration);
void my_setToneChannel(uint8_t channel);
void my_noTone(uint8_t _pin);
