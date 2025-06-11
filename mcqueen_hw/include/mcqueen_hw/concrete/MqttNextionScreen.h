#ifndef MQTT_NEXTION_SCREEN_H
#define MQTT_NEXTION_SCREEN_H

#include "mcqueen_hw/abstract/IScreen.h"
#include "mcqueen_hw/abstract/ICommDevice.h"

#include <memory>
#include <string>

class MqttNextionScreen : public IScreen {
public:
    explicit MqttNextionScreen(const std::string& connectionString, const std::string& clientId, const std::string& targetTopic);
    ~MqttNextionScreen();

    void mirrorScreen(const std::string& payload) override;

private:
    std::string _connectionString;
    std::string _clientId;
    std::string _targetTopic;
    std::unique_ptr<ICommDevice> _mqttComm;
};

#endif // MQTT_NEXTION_SCREEN_H
