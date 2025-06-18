#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include "mcqueen_hw/abstract/ICommDevice.h"

#include <condition_variable>
#include <map>
#include <memory>
#include <mqtt/async_client.h>
#include <mutex>
#include <set>
#include <string>
#include <functional> 

class MqttComm : public ICommDevice {
public:
    using MessageCallback = std::function<void(const std::string& topic, const std::string& message)>;

    MqttComm(const std::string& serverURI, const std::string& clientID);
    ~MqttComm() override;

    bool open() override;
    void close() override;
    
    bool subscribe(const std::string& topic);
    bool write(const std::string& data, const std::string& target = "") override;
    std::string read(size_t maxBytes = 256, const std::string& target = "") override;

    void set_message_callback(MessageCallback cb);
    
private:
    std::string _serverURI;
    std::string _clientID;

    std::unique_ptr<mqtt::async_client> _client;
    mqtt::connect_options _connOpts;

    std::map<std::string, std::string> _lastMessages;
    std::mutex _msgMutex;
    std::condition_variable _msgCond;
    std::set<std::string> _subscribedTopics;

    class Callback;
    std::shared_ptr<Callback> _callback;

    MessageCallback _userCallback = nullptr;
};

#endif // MQTT_COMM_H
