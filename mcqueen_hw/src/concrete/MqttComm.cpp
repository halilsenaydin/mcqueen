#include "mcqueen_hw/concrete/MqttComm.h"

#include <iostream>
#include <mqtt/callback.h>
#include <mutex>

class MqttComm::Callback : public virtual mqtt::callback {
public:
    Callback(MqttComm* parent) : _parent(parent) {}

    void message_arrived(mqtt::const_message_ptr msg) override {
        {
            std::lock_guard<std::mutex> lock(_parent->_msgMutex);
            _parent->_lastMessages[msg->get_topic()] = msg->to_string();
        }
        
        _parent->_msgCond.notify_all();

        // If a user callback is set, invoke it with the received message
        if (_parent->_userCallback) {
            _parent->_userCallback(msg->get_topic(), msg->to_string());
        }
    }
private:
    MqttComm* _parent;
};

MqttComm::MqttComm(const std::string& serverURI, const std::string& clientID)
    : _serverURI(serverURI), _clientID(clientID) {
    _client = std::make_unique<mqtt::async_client>(_serverURI, _clientID);
    _connOpts.set_keep_alive_interval(20);
    _connOpts.set_clean_session(true);

    _callback = std::make_shared<Callback>(this);
    _client->set_callback(*_callback);
}

MqttComm::~MqttComm() {
    close();
}

bool MqttComm::open() {
    try {
        _client->connect(_connOpts)->wait();
        _client->start_consuming();

        return true;
    } catch (const mqtt::exception& e) {
        std::cerr << "MQTT open() failed: " << e.what() << std::endl;

        return false;
    }
}

void MqttComm::close() {
    try {
        for (const auto& topic : _subscribedTopics)
            _client->unsubscribe(topic)->wait();

        _client->stop_consuming();
        _client->disconnect()->wait();
    } catch (const mqtt::exception& e) {
        std::cerr << "MQTT close() error: " << e.what() << std::endl;
    }
}

bool MqttComm::subscribe(const std::string& topic) {
    if (_subscribedTopics.count(topic)) {
        return true; // Already subscribed to this topic
    }

    try {
        _client->subscribe(topic, 1)->wait();
        _subscribedTopics.insert(topic);

        return true;
    } catch (const mqtt::exception& e) {
        std::cerr << "MQTT subscribe() failed for topic " << topic << ": " << e.what() << std::endl;

        return false;
    }
}

bool MqttComm::write(const std::string& data, const std::string& target) {
    if (target.empty()) {
        std::cerr << "MQTT write() failed: target topic is empty!" << std::endl;
        
        return false;
    }

    try {
        auto msg = mqtt::make_message(target, data);

        msg->set_qos(1);
        _client->publish(msg)->wait();
        return true;
    } catch (const mqtt::exception& e) {
        std::cerr << "MQTT write() failed: " << e.what() << std::endl;

        return false;
    }
}

std::string MqttComm::read(size_t maxBytes, const std::string& target) {
    std::unique_lock<std::mutex> lock(_msgMutex);

    if (!_msgCond.wait_for(lock, std::chrono::seconds(5), [&] { 
        return !_lastMessages[target].empty(); 
    })) {
        std::cerr << "MQTT read() timeout: no message received for topic '" << target << "'\n";
        return "";
    }

    std::string msg = _lastMessages[target];
    _lastMessages[target].clear();

    if (msg.size() > maxBytes)
        msg = msg.substr(0, maxBytes);
    
    return msg;
}

void MqttComm::set_message_callback(MessageCallback cb) {
    _userCallback = std::move(cb);
}
