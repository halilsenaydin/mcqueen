#ifndef MCQUEEN_SYSTEM_H
#define MCQUEEN_SYSTEM_H

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <vector>
#include <string>
#include <memory>

#include "mcqueen_hw/concrete/MqttComm.h"

class McQueenSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(McQueenSystem)

  McQueenSystem();
  ~McQueenSystem();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Target wheel velocities and steering angles
  double _rear_left_velocity_command;
  double _rear_right_velocity_command;
  double _front_left_steering_command;
  double _front_right_steering_command;

  // Communication object for MQTT
  std::shared_ptr<MqttComm> _mqtt_comm;

  // MQTT parameters
  std::string _mqtt_url;
  std::string _mqtt_client_id;
  std::string _mqtt_topic;

  struct JointState
  {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    double command = 0.0;
  };

  std::vector<JointState> _joints;
  std::vector<std::string> _joint_names;
};

#endif // MCQUEEN_SYSTEM_H
