#ifndef ACTIVE_BUZZER_H
#define ACTIVE_BUZZER_H

#include "mcqueen_hw/abstract/IBuzzer.h"

#include <gpiod.h>

class ActiveBuzzer : public IBuzzer {
public:
    explicit ActiveBuzzer(int pin);
    ~ActiveBuzzer();

    void on() override;
    void off() override;

private:
    int _pin;
    gpiod_chip* _chip;
    gpiod_line* _line;
};

#endif // ACTIVE_BUZZER_H
