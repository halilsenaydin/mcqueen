#ifndef BUZZER_SERVICE_H
#define BUZZER_SERVICE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mcqueen_hw/abstract/IBuzzer.h"
#include "custom_interfaces/srv/buzzer.hpp"

class BuzzerService : public rclcpp::Node {
public:
    explicit BuzzerService(std::shared_ptr<IBuzzer> buzzer);

private:
    void handleService(
        const std::shared_ptr<custom_interfaces::srv::Buzzer::Request> request,
        std::shared_ptr<custom_interfaces::srv::Buzzer::Response> response
    );

    rclcpp::Service<custom_interfaces::srv::Buzzer>::SharedPtr _service;
    std::shared_ptr<IBuzzer> _buzzer;
};

#endif // BUZZER_SERVICE_H
