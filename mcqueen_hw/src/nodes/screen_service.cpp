#include "mcqueen_hw/nodes/screen_service.h"
#include "mcqueen_hw/concrete/MqttNextionScreen.h"
#include "mcqueen_hw/concrete/MqttComm.h"

ScreenService::ScreenService(std::shared_ptr<IScreen> screen)
: Node("screen_service"), _screen(screen)
{
    // Get config parameters with ROS parameters server
    std::string serviceName;

    this->get_parameter_or("output.screen.service_name", serviceName, std::string("screen"));

    _service = this->create_service<custom_interfaces::srv::Screen>(
        serviceName,
        std::bind(&ScreenService::handleService, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "ScreenService ready.");
}

void ScreenService::handleService(
    const std::shared_ptr<custom_interfaces::srv::Screen::Request> request,
    std::shared_ptr<custom_interfaces::srv::Screen::Response> response)
{
    std::string payload = request->payload;

    _screen->mirrorScreen(payload);

    response->success = true;
    response->message = "Command received: " + payload;

    RCLCPP_INFO(this->get_logger(), response->message.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("screen_service_node");

    std::string mqttAddress, clientId, topicName;

    node->get_parameter_or("comm.mqtt.url", mqttAddress, std::string("tcp://192.168.0.28:1883"));
    node->get_parameter_or("output.screen.comm.client_id", clientId, std::string("mcqueen_screen_server"));
    node->get_parameter_or("output.screen.comm.target_topic_name", topicName, std::string("output/screen"));

    auto screen = std::make_shared<MqttNextionScreen>(mqttAddress, clientId, topicName);
    auto serviceNode = std::make_shared<ScreenService>(screen);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.add_node(serviceNode);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}