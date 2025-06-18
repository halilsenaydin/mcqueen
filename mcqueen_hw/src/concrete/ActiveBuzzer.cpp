#include "mcqueen_hw/concrete/ActiveBuzzer.h"

#include <chrono>
#include <gpiod.h>
#include <stdexcept>
#include <thread>

ActiveBuzzer::ActiveBuzzer(int pin) : _pin(pin), _chip(nullptr), _line(nullptr) {
    _chip = gpiod_chip_open_by_name("gpiochip0");

    if (!_chip) throw std::runtime_error("GPIO chip could not be opened!");

    _line = gpiod_chip_get_line(_chip, _pin);

    if (!_line) {
        gpiod_chip_close(_chip); // Clean up

        throw std::runtime_error("Failed to get GPIO pin!");
    }

    if (gpiod_line_request_output(_line, "ActiveBuzzer", 0) < 0) {
        gpiod_chip_close(_chip); // Clean up

        throw std::runtime_error("Failed to set GPIO pin as output!");
    }
}

ActiveBuzzer::~ActiveBuzzer() {
    if (_line) {
        gpiod_line_release(_line);
    }
        
    if (_chip) {
        gpiod_chip_close(_chip);
    }
}

void ActiveBuzzer::on() {
    if (gpiod_line_set_value(_line, 1) < 0) {
        throw std::runtime_error("Pin HIGH failed!");
    }
}

void ActiveBuzzer::off() {
    if (gpiod_line_set_value(_line, 0) < 0) {
        throw std::runtime_error("Pin LOW failed!");
    }
}
