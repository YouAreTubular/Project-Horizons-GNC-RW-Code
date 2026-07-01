#ifndef BUZZER_H
#define BUZZER_H

class Buzzer {
public:
    static void startupTone(int pin);
    static void errorTone(int pin);
};

#endif