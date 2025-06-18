#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mcqueen_hw/abstract/ICommDevice.h"
#include "custom_interfaces/srv/screen.hpp"

class MotorController : public rclcpp::Node
{
public:
    explicit MotorController(std::shared_ptr<ICommDevice> commDevice);
    ~MotorController();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    std::string encodeMotorCommand(int motorNum, double speed);
    std::string encodeStopMotorCommand();
    void sendScreenUpdate(const std::string& payload);
    std::string encodeScreenCommand(double driveSpeed, double steering);
    std::string generatePayloadForChangePicture(std::string picId);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twistSubscription;
    std::shared_ptr<ICommDevice> _commDevice;
    std::string _targetTopicName;

    rclcpp::Client<custom_interfaces::srv::Screen>::SharedPtr _screenServiceClient;
    std::string _lastPicId = "0";

    bool _isStop = true;
    std::string _lastDriveCommand = "";
    std::string _lastSteeringCommand = "";
};

#endif // MOTOR_CONTROLLER_H
