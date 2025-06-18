#ifndef PASSIVE_BUZZER_H
#define PASSIVE_BUZZER_H

#include "mcqueen_hw/abstract/IBuzzer.h"

#include <pigpio.h>

class PassiveBuzzer : public IBuzzer {
public:
    explicit PassiveBuzzer(int pin);
    ~PassiveBuzzer();

    void on() override;
    void off() override;

private:
    int _pin;
    bool _pigpioInitialized;
};

#endif // PASSIVE_BUZZER_H
