#include "mcqueen_hw/system/mcqueen_system.h"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

McQueenSystem::McQueenSystem()
: _rear_left_velocity_command(0.0),
  _rear_right_velocity_command(0.0),
  _front_left_steering_command(0.0),
  _front_right_steering_command(0.0)
{
}

McQueenSystem::~McQueenSystem() = default;

hardware_interface::CallbackReturn McQueenSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto logger = rclcpp::get_logger("McQueenSystem");
  auto get_param = [&](const std::string & key, std::string & value) -> bool {
    auto it = info.hardware_parameters.find(key);

    if (it == info.hardware_parameters.end()) {
      RCLCPP_ERROR(logger, "Parameter '%s' not found in hardware_parameters", key.c_str());

      return false;
    }

    value = it->second;

    return true;
  };

  if (!get_param("comm.mqtt.url", _mqtt_url)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_param("sensor.motor.comm.client_id", _mqtt_client_id)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_param("sensor.motor.comm.target_topic_name", _mqtt_topic)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  _mqtt_comm = std::make_shared<MqttComm>(_mqtt_url, _mqtt_client_id);
  _joint_names.clear();

  for (const auto & joint : info.joints) {
    _joint_names.push_back(joint.name);
  }

  _joints.resize(_joint_names.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn McQueenSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!_mqtt_comm->open()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn McQueenSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  _mqtt_comm->close();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> McQueenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < _joint_names.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      _joint_names[i], hardware_interface::HW_IF_POSITION, &_joints[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      _joint_names[i], hardware_interface::HW_IF_VELOCITY, &_joints[i].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      _joint_names[i], hardware_interface::HW_IF_EFFORT, &_joints[i].effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> McQueenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &_rear_left_velocity_command);
  interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &_rear_right_velocity_command);
  interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &_front_left_steering_command);
  interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &_front_right_steering_command);

  return interfaces;
}

hardware_interface::return_type McQueenSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : _joints) {
    joint.position = 0.0;
    joint.velocity = 0.0;
    joint.effort = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type McQueenSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string command;

  // Determine direction (forward/backward/stop)
  if (std::abs(_rear_left_velocity_command) < 1e-3 && std::abs(_rear_right_velocity_command) < 1e-3) {
    command = "MSTOP";
  } else if (_rear_left_velocity_command > 0.0 || _rear_right_velocity_command > 0.0) {
    if (_front_left_steering_command > 0.1) {
      command = "MFR";  // Front right turn, rear forward
    } else if (_front_left_steering_command < -0.1) {
      command = "MFL";  // Front left turn, rear forward
    } else {
      command = "MFN";  // Rear forward only
    }
  } else if (_rear_left_velocity_command < 0.0 || _rear_right_velocity_command < 0.0) {
    if (_front_left_steering_command > 0.1) {
      command = "MBR";  // Front right turn, rear backward
    } else if (_front_left_steering_command < -0.1) {
      command = "MBL";  // Front left turn, rear backward
    } else {
      command = "MBN";  // Rear backward only
    }
  }

  _mqtt_comm->write(command, _mqtt_topic);

  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(McQueenSystem, hardware_interface::SystemInterface)