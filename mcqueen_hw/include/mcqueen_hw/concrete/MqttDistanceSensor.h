#ifndef MQTT_DISTANCE_SENSOR_H
#define MQTT_DISTANCE_SENSOR_H

#include "mcqueen_hw/abstract/IDistanceSensor.h"
#include "mcqueen_hw/abstract/ICommDevice.h"

#include <mutex>
#include <memory>
#include <string>

class MqttDistanceSensor : public IDistanceSensor {
public:
    explicit MqttDistanceSensor(const std::string& connectionString, const std::string& clientId, const std::string& subscribeTopic);
    ~MqttDistanceSensor();

    void initialize() override;
    std::vector<double> getDistance() override;
    std::vector<double> getIntensity() override;
    void shutdown() override;

private:
    std::string _connectionString;
    std::string _clientId;
    std::string _subscribeTopic;
    std::unique_ptr<ICommDevice> _mqttComm;

    std::mutex _distanceMutex;
    double _lastDistance;
};

#endif // MQTT_DISTANCE_SENSOR_H
