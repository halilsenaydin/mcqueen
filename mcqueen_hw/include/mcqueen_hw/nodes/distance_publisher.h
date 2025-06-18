#ifndef DISTANCE_PUBLISHER_H
#define DISTANCE_PUBLISHER_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "mcqueen_hw/abstract/IDistanceSensor.h"

class DistancePublisher : public rclcpp::Node
{
public:
    explicit DistancePublisher(std::shared_ptr<IDistanceSensor> sensor);

private:
    void timerCallback();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<IDistanceSensor> _sensor;

    sensor_msgs::msg::LaserScan _scanMsg;
};

#endif  // DISTANCE_PUBLISHER_H
