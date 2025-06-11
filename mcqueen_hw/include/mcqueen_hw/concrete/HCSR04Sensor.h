#ifndef HCSR04_SENSOR_H
#define HCSR04_SENSOR_H

#include "mcqueen_hw/abstract/IDistanceSensor.h"

#include <chrono>
#include <gpiod.h>
#include <thread>

class HCSR04Sensor : public IDistanceSensor {
public:
    explicit HCSR04Sensor(const char* chipname, int trigPin, int echoPin);
    ~HCSR04Sensor();

    void initialize() override;
    std::vector<double> getDistance() override;
    std::vector<double> getIntensity() override;
    void shutdown() override;

private:
    gpiod_chip *_chip;
    gpiod_line *_trig, *_echo;
    const char* _chipname;
    int _trigPin, _echoPin;
};

#endif // HCSR04_SENSOR_H
