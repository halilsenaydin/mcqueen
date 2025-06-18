cmake_minimum_required(VERSION 3.10)
project(mqtt_comm_standalone)

set(CMAKE_CXX_STANDARD 17)

find_package(PahoMqttCpp REQUIRED)

include_directories(
  ../../include
)

add_executable(mqtt_comm 
  ../../src/abstract/ICommDevice.cpp
  ../../src/concrete/MqttComm.cpp 
  ../../examples/mqtt_comm.cpp 
)

target_link_libraries(mqtt_comm
    PahoMqttCpp::paho-mqttpp3
    /usr/local/lib/libpaho-mqtt3as.dylib
)
