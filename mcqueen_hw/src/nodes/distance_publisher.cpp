#include "mcqueen_hw/nodes/distance_publisher.h"
#include "mcqueen_hw/concrete/MqttDistanceSensor.h"

using namespace std::chrono_literals;

DistancePublisher::DistancePublisher(std::shared_ptr<IDistanceSensor> sensor)
: Node("distance_publisher"), _sensor(sensor)
{
    if (!_sensor) {
        RCLCPP_ERROR(this->get_logger(), "Distance sensor is not initialized");
        throw std::runtime_error("Distance sensor is not initialized");
    }

    _sensor->initialize();

    // Get config parameters with ROS parameters server
    std::string topicName, frameId;
    double angleMin, angleMax, angleIncrement, timeIncrement, scanTime, rangeMin, rangeMax;
    int numReadings;

    this->get_parameter_or("sensor.laser.topic_name", topicName, std::string("/scan"));
    this->get_parameter_or("sensor.laser.frame_id", frameId, std::string("laser_frame"));
    this->get_parameter_or("sensor.laser.angle_min", angleMin, 0.0);
    this->get_parameter_or("sensor.laser.angle_max", angleMax, 0.0);
    this->get_parameter_or("sensor.laser.angle_increment", angleIncrement, 0.0);
    this->get_parameter_or("sensor.laser.time_increment", timeIncrement, 0.0);
    this->get_parameter_or("sensor.laser.scan_time", scanTime, 0.1);
    this->get_parameter_or("sensor.laser.range_min", rangeMin, 0.02);
    this->get_parameter_or("sensor.laser.range_max", rangeMax, 4.0);
    this->get_parameter_or("sensor.laser.num_readings", numReadings, 1);

    // Publisher topic with LaserScan msg type
    _publisher = this->create_publisher<sensor_msgs::msg::LaserScan>(topicName, 10);

    // Timer to periodically publish sensor data
    _timer = this->create_wall_timer(
        100ms, std::bind(&DistancePublisher::timerCallback, this));

    // Initialize LaserScan message
    _scanMsg.header.frame_id = frameId;
    _scanMsg.angle_min = angleMin;
    _scanMsg.angle_max = angleMax;
    _scanMsg.angle_increment = angleIncrement;
    _scanMsg.time_increment = timeIncrement;
    _scanMsg.scan_time = scanTime;
    _scanMsg.range_min = rangeMin;
    _scanMsg.range_max = rangeMax;
    _scanMsg.ranges.resize(numReadings);
    _scanMsg.intensities.resize(numReadings);
}

void DistancePublisher::timerCallback()
{
    std::vector<double> distances = _sensor->getDistance();
    std::vector<double> intensities = _sensor->getIntensity();
    std::vector<float> float_distances(distances.begin(), distances.end());
    std::vector<float> float_intensities(intensities.begin(), intensities.end());

    _scanMsg.ranges = float_distances;
    _scanMsg.intensities = float_intensities;
    _scanMsg.header.stamp = this->now();

    _publisher->publish(_scanMsg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("distance_publisher_node");

    std::string mqttAddress, clientId, topicName;

    node->get_parameter_or("comm.mqtt.url", mqttAddress, std::string("tcp://192.168.0.28:1883"));
    node->get_parameter_or("sensor.laser.comm.client_id", clientId, std::string("mcqueen_distance_publisher"));
    node->get_parameter_or("sensor.laser.comm.target_topic_name", topicName, std::string("sensor/distance"));

    auto distanceSensor = std::make_shared<MqttDistanceSensor>(mqttAddress, clientId, topicName);
    auto publisherNode = std::make_shared<DistancePublisher>(distanceSensor);
    
    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.add_node(publisherNode);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}