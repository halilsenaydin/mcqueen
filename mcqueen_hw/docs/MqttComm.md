# MQTT Communication with MqttComm

MqttComm is a concrete implementation of the `ICommDevice` interface that enables MQTT-based communication using the Eclipse Paho C++ library.

It supports:

- Connecting to an MQTT broker
- Publishing messages to a topic
- Subscribing to multiple topics
- Reading received messages in a thread-safe way
- Seamless integration with ROS 2 or standalone usage

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Dependencies

- Eclipse Paho MQTT C++ library
- C++17 compatible compiler

### Installing Paho MQTT C++ Library

Install Paho MQTT C Library:

```bash
cd ~/
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_WITH_SSL=TRUE -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --build build --target install
```

Install Paho MQTT C++ Library:

```bash
cd ~/
git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_MQTT_C_PATH=/usr/local -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --build build --target install
```

## Integration with ROS 2

If you are using this library inside a ROS 2 package (e.g., mcqueen_hw):

- The package is set up with `ament_cmake`
- You can include and use `MqttComm` inside your ROS 2 nodes
- Just add `mcqueen_hw_lib` as a dependency and link it in your `CMakeLists.txt`
- Your ROS nodes can then create and manage `MqttComm` objects as shown above

### Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Standalone Usage (Without ROS)

For non-ROS projects, you can build and use `MqttComm` standalone:

- Create a simple CMake project linking against Paho MQTT C++ libraries
- Include source files from `mcqueen_hw/src` and headers from include
- See the example `standalone/mqtt_comm/CMakeLists.txt` for guidance

### Usage Example

```cpp
#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/MqttComm.h"

#include <iostream>
#include <memory>

int main() {
    std::unique_ptr<ICommDevice> comm = std::make_unique<MqttComm>("tcp://192.168.0.28:1883", "client_id");

    if (!comm->open()) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    if (auto mqttCommPtr = dynamic_cast<MqttComm*>(comm.get())) {
        mqttCommPtr->subscribe("sensors/temp");
        mqttCommPtr->subscribe("alerts");
    }

    comm->write("23.5", "sensors/temp");
    comm->write("Fire!", "alerts");

    std::string temp = comm->read(256, "sensors/temp");
    std::string alert = comm->read(256, "alerts");

    std::cout << "Temp: " << temp << "\n";
    std::cout << "Alert: " << alert << "\n";

    return 0;
}
```

## Building

### ROS 2 Build

Inside your ROS 2 workspace:

```bash
colcon build --packages-select mcqueen_hw
```

### Standalone Build

If you want to build without ROS 2:

```bash
cd standalone/mqtt_comm
mkdir build && cd build
cmake ..
make
./mqtt_comm
```

Make sure Paho MQTT C++ is installed and discoverable by CMake.
