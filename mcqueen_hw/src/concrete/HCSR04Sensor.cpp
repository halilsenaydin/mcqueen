#include "mcqueen_hw/concrete/HCSR04Sensor.h"

#include <iostream>

HCSR04Sensor::HCSR04Sensor(const char* chipname, int trigPin, int echoPin)
    : _chipname(chipname), _trigPin(trigPin), _echoPin(echoPin) {}

HCSR04Sensor::~HCSR04Sensor() {
    shutdown();
}

void HCSR04Sensor::initialize() {
    _chip = gpiod_chip_open_by_name(_chipname);
    _trig = gpiod_chip_get_line(_chip, _trigPin);
    _echo = gpiod_chip_get_line(_chip, _echoPin);

    gpiod_line_request_output(_trig, "ultrasonic", 0);
    gpiod_line_request_input(_echo, "ultrasonic");
}

std::vector<double> HCSR04Sensor::getDistance() {
    using namespace std::chrono;
    using namespace std::this_thread;

    gpiod_line_set_value(_trig, 0);
    sleep_for(microseconds(2));
    gpiod_line_set_value(_trig, 1);
    sleep_for(microseconds(10));
    gpiod_line_set_value(_trig, 0);

    auto start = high_resolution_clock::now();
    while (gpiod_line_get_value(_echo) == 0);
    start = high_resolution_clock::now();
    while (gpiod_line_get_value(_echo) == 1);
    auto end = high_resolution_clock::now();

    duration<double> elapsed = end - start;
    double distanceCM = elapsed.count() * 34300 / 2; // Speed of sound is 34300 cm/s
    double distanceM = distanceCM / 100.0; // Convert to meters

    return std::vector<double>{ distanceM }; 
}

std::vector<double> HCSR04Sensor::getIntensity() {
    return std::vector<double>{ 0.0 }; 
}

void HCSR04Sensor::shutdown() {
    gpiod_chip_close(_chip);
}
