#include <Arduino.h>
#include "Buzzer.h"

void Buzzer::startupTone(int pin)
{
    tone(pin, 523, 150);
    delay(175);
    tone(pin, 659, 150);
    delay(175);
}

void Buzzer::errorTone(int pin)
{
    tone(pin, 200, 500);
}