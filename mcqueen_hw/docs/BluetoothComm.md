# Bluetooth Communication with BluetoothComm

BluetoothComm is a concrete implementation of the `ICommDevice` interface that enables Bluetooth-based communication using BlueZ (Linux Bluetooth stack) and C++.

It supports:

- Discovering and connecting to Bluetooth devices
- Sending data to a paired device
- Receiving data in a thread-safe manner
- Working seamlessly inside ROS 2 nodes or as a standalone component

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Integration with ROS 2

If you are using this library inside a ROS 2 package (e.g., mcqueen_hw):

- The package is set up with `ament_cmake`
- You can include and use `BluetoothComm` inside your ROS 2 nodes
- Just add `mcqueen_hw_lib` as a dependency and link it in your `CMakeLists.txt`
- Create and manage `BluetoothComm` objects in your ROS nodes similar to other communication devices

### Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Standalone Usage (Without ROS)

For non-ROS projects, you can build and use `BluetoothComm` standalone:

- Create a simple CMake project linking against BlueZ libraries
- Include source files from `mcqueen_hw/src` and headers from `include`
- See the example `standalone/bluetooth_comm/CMakeLists.txt` for guidance

### Dependencies

- BlueZ Bluetooth libraries (`libbluetooth-dev` on Debian/Ubuntu)
- C++17 compatible compiler

### Installing BlueZ Development Libraries

On Debian/Ubuntu systems, install BlueZ headers and tools with:

```bash
sudo apt-get update
sudo apt-get install libbluetooth-dev bluetooth
```

### Usage Example

```cpp
#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/BluetoothComm.h"

#include <iostream>
#include <memory>

int main() {
    // Replace with your Bluetooth device's MAC address
    std::unique_ptr<ICommDevice> comm = std::make_unique<BluetoothComm>("00:1A:7D:DA:71:13", 1);

    if (!comm->open()) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    comm->write("Hello Bluetooth!");

    std::string response = comm->read();
    std::cout << "Data received: " << response << std::endl;

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
cd standalone/bluetooth_comm
mkdir build && cd build
cmake ..
make
./bluetooth_comm
```

Make sure BlueZ libraries are installed and discoverable by CMake.
