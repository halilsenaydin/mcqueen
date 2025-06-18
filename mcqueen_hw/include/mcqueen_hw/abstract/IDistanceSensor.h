#ifndef IDISTANCE_SENSOR_H
#define IDISTANCE_SENSOR_H

#include <vector>

class IDistanceSensor {
public:
    virtual ~IDistanceSensor();

    virtual std::vector<double> getDistance() = 0;
    virtual std::vector<double> getIntensity() = 0;
    virtual void initialize() = 0;
    virtual void shutdown() = 0;
};

#endif // IDISTANCE_SENSOR_H
