cmake_minimum_required(VERSION 3.10)
project(serial_comm_standalone)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  ../../include
)

add_executable(serial_comm
  ../../src/abstract/ICommDevice.cpp
  ../../src/concrete/SerialComm.cpp
  ../../examples/serial_comm.cpp
)
