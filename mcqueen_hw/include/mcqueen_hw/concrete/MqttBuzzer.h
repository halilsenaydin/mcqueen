#ifndef MQTT_BUZZER_H
#define MQTT_BUZZER_H

#include "mcqueen_hw/abstract/IBuzzer.h"
#include "mcqueen_hw/abstract/ICommDevice.h"

#include <memory>
#include <string>

class MqttBuzzer : public IBuzzer {
public:
    explicit MqttBuzzer(const std::string& connectionString, const std::string& clientId, const std::string& targetTopic);
    ~MqttBuzzer();

    void on() override;
    void off() override;

private:
    std::string _connectionString;
    std::string _clientId;
    std::string _targetTopic;
    std::unique_ptr<ICommDevice> _mqttComm;
};

#endif // MQTT_BUZZER_H
