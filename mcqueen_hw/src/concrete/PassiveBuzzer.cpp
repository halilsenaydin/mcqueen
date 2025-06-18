#include "mcqueen_hw/concrete/PassiveBuzzer.h"

#include <stdexcept>

PassiveBuzzer::PassiveBuzzer(int pin) : _pin(pin), _pigpioInitialized(false) {
    // Start pigpio libarary
    if (gpioInitialise() < 0) {
        throw std::runtime_error("pigpio initialization failed!");
    }

    _pigpioInitialized = true;
    
    gpioSetMode(_pin, PI_OUTPUT); // Set pin as output
    gpioPWM(_pin, 0); // Keep it closed at startup
}

PassiveBuzzer::~PassiveBuzzer() {
    if (_pigpioInitialized) {
        gpioPWM(_pin, 0); // PWM off
        gpioTerminate(); // End pigpio
    }
}

void PassiveBuzzer::on() {
    if (!_pigpioInitialized) return;

    gpioPWM(_pin, 128); // Audio output with PWM (duty cycle 128, 0-255, half power)
}

void PassiveBuzzer::off() {
    if (!_pigpioInitialized) return;

    gpioPWM(_pin, 0); // Stop PWM
}
