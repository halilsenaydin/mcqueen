cmake_minimum_required(VERSION 3.10)
project(bluetooth_comm_standalone)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  ../../include
)

add_executable(bluetooth_comm
  ../../src/abstract/ICommDevice.cpp
  ../../src/concrete/BluetoothComm.cpp
  ../../examples/bluetooth_comm.cpp

)

target_link_libraries(bluetooth_comm
  bluetooth
)
