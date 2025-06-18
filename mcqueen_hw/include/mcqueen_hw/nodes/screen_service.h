#ifndef SCREEN_SERVICE_H
#define SCREEN_SERVICE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mcqueen_hw/abstract/IScreen.h"
#include "custom_interfaces/srv/screen.hpp"

class ScreenService : public rclcpp::Node {
public:
    explicit ScreenService(std::shared_ptr<IScreen> screen);

private:
    void handleService(
        const std::shared_ptr<custom_interfaces::srv::Screen::Request> request,
        std::shared_ptr<custom_interfaces::srv::Screen::Response> response
    );

    rclcpp::Service<custom_interfaces::srv::Screen>::SharedPtr _service;
    std::shared_ptr<IScreen> _screen;
};

#endif // SCREEN_SERVICE_H
