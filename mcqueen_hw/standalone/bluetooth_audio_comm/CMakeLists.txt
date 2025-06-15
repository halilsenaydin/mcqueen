cmake_minimum_required(VERSION 3.10)
project(bluetooth_audio_comm_standalone)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  ../../include
)

add_executable(bluetooth_comm
  ../../src/abstract/ICommDevice.cpp
  ../../src/concrete/BluetoothAudioComm.cpp
  ../../examples/bluetooth_audio_comm.cpp
)
