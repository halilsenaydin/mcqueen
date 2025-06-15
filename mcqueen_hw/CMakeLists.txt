cmake_minimum_required(VERSION 3.8)
project(mcqueen_hw)

# Enable common compile warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(foxglove_bridge REQUIRED)

# Find Paho MQTT C/C++ libraries
find_path(PAHO_MQTT_CPP_INCLUDE_DIR NAMES mqtt/async_client.h)
find_library(PAHO_MQTT_C_LIBRARY NAMES paho-mqtt3as REQUIRED)
find_library(PAHO_MQTT_CPP_LIBRARY NAMES paho-mqttpp3 REQUIRED)

find_library(PIGPIO_LIBRARY pigpio REQUIRED)
find_path(PIGPIO_INCLUDE_DIR pigpio.h REQUIRED)

# Pluginlib
find_package(pluginlib REQUIRED)

# Whisper.cpp
find_package(PulseAudio REQUIRED)
add_subdirectory(third_party/whisper.cpp)

# Define platform-dependent GPIO support (libgpiod is Linux-only)
if(UNIX AND NOT APPLE)
  message(STATUS "Linux detected: enabling libgpiod support")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GPIOD libgpiod REQUIRED)
  set(USE_GPIOD TRUE)
else()
  message(STATUS "Non-Linux system detected: skipping libgpiod")
  set(USE_GPIOD FALSE)
endif()

# Add include directories
include_directories(
  include
  third_party/whisper.cpp/include
)

if(USE_GPIOD)
  include_directories(${GPIOD_INCLUDE_DIRS})
  link_directories(${GPIOD_LIBRARY_DIRS})
endif()

# Define abstract and concrete class source files
set(ABSTRACT_SOURCES
  src/abstract/IBuzzer.cpp
  src/abstract/ICommDevice.cpp
  src/abstract/IController.cpp
  src/abstract/IDistanceSensor.cpp
  src/abstract/IScreen.cpp
)

set(CONCRETE_SOURCES
  src/concrete/ActiveBuzzer.cpp
  src/concrete/BluetoothComm.cpp
  src/concrete/HCSR04Sensor.cpp
  src/concrete/MqttBuzzer.cpp
  src/concrete/MqttComm.cpp
  src/concrete/MqttDistanceSensor.cpp
  src/concrete/MqttNextionScreen.cpp
  src/concrete/PassiveBuzzer.cpp
  src/concrete/PIDController.cpp
  src/concrete/SerialComm.cpp
)

# Create main library
add_library(mcqueen_hw SHARED
  ${ABSTRACT_SOURCES}
  ${CONCRETE_SOURCES}
  src/system/mcqueen_system.cpp
)

# Enable C++17
target_compile_features(mcqueen_hw PUBLIC cxx_std_17)

# Include directories for the library
target_include_directories(mcqueen_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PAHO_MQTT_CPP_INCLUDE_DIR}
  ${PIGPIO_INCLUDE_DIR}
)

ament_target_dependencies(mcqueen_hw
  hardware_interface
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  custom_interfaces
)

if(USE_GPIOD)
  target_include_directories(mcqueen_hw PUBLIC ${GPIOD_INCLUDE_DIRS})
endif()

# Link libraries
target_link_libraries(mcqueen_hw
  ${PAHO_MQTT_CPP_LIBRARY}
  ${PAHO_MQTT_C_LIBRARY}
  pthread
  bluetooth
  rt
  ${PIGPIO_LIBRARY}
)

if(USE_GPIOD)
  target_link_libraries(mcqueen_hw ${GPIOD_LIBRARIES})
endif()

# Executables

add_executable(buzzer_service src/nodes/buzzer_service.cpp)
target_link_libraries(buzzer_service 
  mcqueen_hw
  ${PAHO_MQTT_C_LIBRARY}
)
ament_target_dependencies(buzzer_service 
  rclcpp 
  custom_interfaces
)

add_executable(distance_publisher src/nodes/distance_publisher.cpp)
target_link_libraries(distance_publisher 
  mcqueen_hw
  ${PAHO_MQTT_C_LIBRARY}
)
ament_target_dependencies(distance_publisher 
  rclcpp 
  sensor_msgs
)

add_executable(motor_controller src/nodes/motor_controller.cpp)
target_link_libraries(motor_controller 
  mcqueen_hw
  ${PAHO_MQTT_C_LIBRARY}
)
ament_target_dependencies(motor_controller 
  rclcpp 
  geometry_msgs
  custom_interfaces
)

add_executable(screen_service src/nodes/screen_service.cpp)
target_link_libraries(screen_service 
  mcqueen_hw
  ${PAHO_MQTT_C_LIBRARY}
)
ament_target_dependencies(screen_service 
  rclcpp 
  custom_interfaces
)

# Installation
install(
  TARGETS 
    mcqueen_hw
    buzzer_service
    distance_publisher
    motor_controller
    screen_service
  EXPORT mcqueen_hwTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES plugin_description/hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface plugin_description/hardware_plugin.xml)

# Export
ament_export_targets(mcqueen_hwTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# Mark as an ament package
ament_package()
