#include "mcqueen_hw/nodes/buzzer_service.h"
#include "mcqueen_hw/concrete/MqttBuzzer.h"

BuzzerService::BuzzerService(std::shared_ptr<IBuzzer> buzzer)
: Node("buzzer_service"), _buzzer(buzzer)
{
    // Get config parameters with ROS parameters server
    std::string serviceName;

    this->get_parameter_or("sensor.buzzer.service_name", serviceName, std::string("buzzer"));

    _service = this->create_service<custom_interfaces::srv::Buzzer>(
        serviceName,
        std::bind(&BuzzerService::handleService, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "BuzzerService ready.");
}

void BuzzerService::handleService(
    const std::shared_ptr<custom_interfaces::srv::Buzzer::Request> request,
    std::shared_ptr<custom_interfaces::srv::Buzzer::Response> response)
{
    if (request->state) {
        _buzzer->on();
        response->message = "Buzzer turned ON";
    } else {
        _buzzer->off();
        response->message = "Buzzer turned OFF";
    }

    response->success = true;

    RCLCPP_INFO(this->get_logger(), response->message.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("buzzer_service_node");

    std::string mqttAddress, clientId, topicName;

    node->get_parameter_or("comm.mqtt.url", mqttAddress, std::string("tcp://192.168.0.28:1883"));
    node->get_parameter_or("sensor.buzzer.comm.client_id", clientId, std::string("mcqueen_buzzer_server"));
    node->get_parameter_or("sensor.buzzer.comm.target_topic_name", topicName, std::string("sensor/buzzer"));

    auto buzzer = std::make_shared<MqttBuzzer>(mqttAddress, clientId, topicName);
    auto serviceNode = std::make_shared<BuzzerService>(buzzer);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.add_node(serviceNode);
    exec.spin();
    
    rclcpp::shutdown();

    return 0;
}