#include "mcqueen_hw/concrete/MqttBuzzer.h"
#include "mcqueen_hw/concrete/MqttComm.h"

MqttBuzzer::MqttBuzzer(const std::string& connectionString, const std::string& clientId, const std::string& targetTopic) 
    : _connectionString(connectionString), _clientId(clientId), _targetTopic(targetTopic), _mqttComm(nullptr) {
    _mqttComm = std::make_unique<MqttComm>(_connectionString, _clientId);

    if (!_mqttComm->open()) {
        throw std::runtime_error("Failed to connect to MQTT broker!");
    }
}

MqttBuzzer::~MqttBuzzer() = default;

void MqttBuzzer::on() {
    if (_mqttComm) {
        _mqttComm->write("1", _targetTopic);
    }
}

void MqttBuzzer::off() {
    if (_mqttComm) {
        _mqttComm->write("0", _targetTopic);
    }
}
