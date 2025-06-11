#include "mcqueen_hw/concrete/MqttDistanceSensor.h"
#include "mcqueen_hw/concrete/MqttComm.h"

#include <iostream>

MqttDistanceSensor::MqttDistanceSensor(const std::string& connectionString, const std::string& clientId, const std::string& subscribeTopic) 
    : _connectionString(connectionString), _clientId(clientId), _subscribeTopic(subscribeTopic), _mqttComm(nullptr) {
    _mqttComm = std::make_unique<MqttComm>(_connectionString, _clientId);

    if (!_mqttComm->open()) {
        throw std::runtime_error("Failed to connect to MQTT broker!");
    }
}

MqttDistanceSensor::~MqttDistanceSensor() {
    shutdown();
}

void MqttDistanceSensor::initialize() {
    if (auto mqttCommPtr = dynamic_cast<MqttComm*>(_mqttComm.get())) {
        mqttCommPtr->subscribe(_subscribeTopic);

        mqttCommPtr->set_message_callback([this](const std::string& topic, const std::string& message) {
            if (topic == _subscribeTopic) {
                try {
                    double dist = std::stod(message);
                    std::lock_guard<std::mutex> lock(_distanceMutex);

                    _lastDistance = dist;
                } catch (const std::exception& e) {
                    std::cerr << "Distance parsing error: " << e.what() << std::endl;
                }
            }
        });
    }
}

std::vector<double> MqttDistanceSensor::getDistance() {
    std::lock_guard<std::mutex> lock(_distanceMutex);

    return std::vector<double>(1, _lastDistance);
}

std::vector<double> MqttDistanceSensor::getIntensity() {
    return std::vector<double>{ 0.0 }; 
}

void MqttDistanceSensor::shutdown() {}
