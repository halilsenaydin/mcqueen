# Serial Communication with SerialComm

`SerialComm` is a concrete implementation of the `ICommDevice` interface that provides serial (UART) communication over POSIX-compliant systems such as macOS and Linux.

It supports:

- Opening and configuring serial ports
- Sending data over serial
- Reading data from a serial device
- Lightweight and dependency-free implementation
- Seamless integration with ROS 2 or standalone usage

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Integration with ROS 2

If you are using this library inside a ROS 2 package (e.g., `mcqueen_hw`):

- The package is set up with `ament_cmake`
- You can include and use `SerialComm` inside your ROS 2 nodes
- Just add `mcqueen_hw_lib` as a dependency and link it in your `CMakeLists.txt`
- Your ROS nodes can then create and manage `SerialComm` objects as shown below

### Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Standalone Usage (Without ROS)

For non-ROS projects, you can build and use `SerialComm` standalone:

- Create a simple CMake project
- Include the source files from `mcqueen_hw/src` and headers from `include`
- See the example `standalone/serial_comm/CMakeLists.txt` for guidance

### Dependencies

- POSIX-compliant operating system (Linux, macOS)
- C++17 compatible compiler

No external libraries are required.

### Usage Example

```cpp
#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/SerialComm.h"

#include <iostream>
#include <memory>

int main() {
    std::string portName = "/dev/cu.usbserial-12345";
    unsigned int baudRate = 9600;

    std::unique_ptr<ICommDevice> comm = std::make_unique<SerialComm>(portName, baudRate);

    if (!comm->open()) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    // Example message to send
    std::string messageToSend = "Hello Serial Device!\n";

    if (!comm->write(messageToSend)) {
        std::cerr << "Failed to write to serial port\n";
    } else {
        std::cout << "Sent: " << messageToSend;
    }

    // Read response from the device
    std::string received = comm->read(256);

    if (!received.empty()) {
        std::cout << "Received: " << received << "\n";
    } else {
        std::cout << "No data received (timeout or empty)\n";
    }

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
cd standalone/serial_comm
mkdir build && cd build
cmake ..
make
./serial_comm
```

No additional setup is required for POSIX systems as long as the serial device is accessible at runtime (e.g., /dev/ttyUSB0, /dev/tty.usbserial-\*).
