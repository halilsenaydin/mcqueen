#include "mcqueen_hw/concrete/MqttNextionScreen.h"
#include "mcqueen_hw/concrete/MqttComm.h"

#include <stdexcept>

MqttNextionScreen::MqttNextionScreen(const std::string& connectionString, const std::string& clientId, const std::string& targetTopic) 
    : _connectionString(connectionString), _clientId(clientId), _targetTopic(targetTopic), _mqttComm(nullptr) {
    _mqttComm = std::make_unique<MqttComm>(_connectionString, _clientId);

    if (!_mqttComm->open()) {
        throw std::runtime_error("Failed to connect to MQTT broker!");
    }
}

MqttNextionScreen::~MqttNextionScreen() {}

void MqttNextionScreen::mirrorScreen(const std::string& payload) {

    _mqttComm->write(payload, _targetTopic);
}
