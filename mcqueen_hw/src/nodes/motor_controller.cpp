#include "mcqueen_hw/nodes/motor_controller.h"
#include "mcqueen_hw/concrete/MqttComm.h"

#include <algorithm>

MotorController::MotorController(std::shared_ptr<ICommDevice> commDevice)
: Node("motor_controller"), _commDevice(commDevice)
{
    // Get config parameters with ROS parameters server
    std::string topicName, targetTopicName, screenServiceName;

    this->get_parameter_or("sensor.motor.topic_name", topicName, std::string("/cmd_vel"));
    this->get_parameter_or("sensor.motor.comm.target_topic_name", targetTopicName, std::string("motor/command"));
    this->get_parameter_or("output.screen.service_name", screenServiceName, std::string("screen"));

    _targetTopicName = targetTopicName;
    _screenServiceClient = this->create_client<custom_interfaces::srv::Screen>(screenServiceName);

    if (!_commDevice->open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open communication device");
        throw std::runtime_error("Failed to open communication device");
    }

    _twistSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
        topicName, 10,
        std::bind(&MotorController::twistCallback, this, std::placeholders::_1));
}

MotorController::~MotorController() = default;

void MotorController::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    int steeringMotor = 1; // Motor 1 is steering
    int driveMotor = 2; // Motor 2 is drive

    // drive motor only linear.x
    double driveSpeed = msg->linear.x;

    // steering motor only angular.z
    double steering = msg->angular.z;

    // Update the screen based on the drive speed and steering
    // std::string picId = encodeScreenCommand(driveSpeed, steering);

    // if (picId != _lastPicId) {
    //     std::string payload = generatePayloadForChangePicture(picId);

    //     sendScreenUpdate(payload);

    //     _lastPicId = picId;
    // }

    RCLCPP_INFO(this->get_logger(), "driveSpeed=%.2f steering=%.2f picId=%s", driveSpeed, steering, _lastPicId.c_str());

    // If driveSpeed and steering are zero, we must stop the motors
    if (driveSpeed == 0.0 && steering == 0.0 && !_isStop) {
        _isStop = true;
        _lastDriveCommand = "";
        _lastSteeringCommand = "";

        std::string stopCmd = encodeStopMotorCommand();

        if (!_commDevice->write(stopCmd, _targetTopicName)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write stop motor command");
        }

        return; // Stop processing if need to stop
    }

    // Encode drive motor command
    std::string driveCmd = encodeMotorCommand(driveMotor, driveSpeed);
    std::string driveMotorDir = driveCmd.substr(0, 3);

    if (driveMotorDir != _lastDriveCommand && std::abs(driveSpeed) > 0.0) {
        _lastDriveCommand = driveMotorDir;

        if (!_commDevice->write(driveCmd, _targetTopicName)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write drive motor command");
        } else {
            _isStop = false;
        }
    }

    // Encode steering motor command
    std::string steeringCmd = encodeMotorCommand(steeringMotor, steering);
    std::string steeringMotorDir = steeringCmd.substr(0, 3);

    if (steeringMotorDir != _lastSteeringCommand && std::abs(steering) > 0.0) {
        _lastSteeringCommand = steeringMotorDir;

        if (!_commDevice->write(steeringCmd, _targetTopicName)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write steering motor command");
        } else {
            _isStop = false;
        }
    }
}

std::string MotorController::encodeMotorCommand(int motorNum, double speed)
{
    char direction = 'F'; // Default to forward
    int absSpeed = static_cast<int>(std::min(std::abs(speed) * 255.0, 255.0));

    if (speed < 0) {
        direction = 'B';
    }

    return "M" + std::to_string(motorNum) + direction + std::to_string(absSpeed) + "\n";
}

std::string MotorController::encodeStopMotorCommand()
{
    return "MSTOP\n";
}

void MotorController::sendScreenUpdate(const std::string& payload)
{
    if (!_screenServiceClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Screen service not available");
        return;
    }

    auto request = std::make_shared<custom_interfaces::srv::Screen::Request>();
    request->payload = payload;

    _screenServiceClient->async_send_request(request);
}

std::string MotorController::encodeScreenCommand(double driveSpeed, double steering)
{
    std::string picId = "0";

    if (driveSpeed == 0.0) {
        if (steering == 0.0) {
            picId = "0" ; // Opened
        } else if (steering > 0.0) {
            picId = "2"; // Opened Look Right
        } else {
            picId = "3"; // Opened Look Left
        }
    } else {
        if (steering == 0.0) {
            picId = "1" ; // Blink
        } else if (steering > 0.0) {
            picId = "4"; // Blink Look Right
        } else {
            picId = "5"; // Blink Look Left
        }
    }

    return picId;
}

std::string MotorController::generatePayloadForChangePicture(std::string picId)
{
    return "component=p0&property=pic&value=" + picId;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("motor_controller_node");

    std::string mqttAddress, clientId;

    node->get_parameter_or("comm.mqtt.url", mqttAddress, std::string("tcp://192.168.0.28:1883"));
    node->get_parameter_or("sensor.motor.comm.client_id", clientId, std::string("mcqueen_motor_controller"));

    auto comm = std::make_shared<MqttComm>(mqttAddress, clientId);
    auto publisherNode = std::make_shared<MotorController>(comm);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.add_node(publisherNode);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
