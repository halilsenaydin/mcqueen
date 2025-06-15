# Bluetooth Communication with BluetoothAudioComm

`BluetoothAudioComm` is a concrete implementation of the `ICommDevice` interface that enables interaction with Bluetooth speaker devices on Linux using the BlueZ stack. This class provides modular and standardized methods for pairing, trusting, connecting, and playing audio files on a Bluetooth device.

It supports:

- Pairing, trusting, and connecting to Bluetooth speaker devices via BlueZ commands
- Playing audio files (e.g., MP3) on the connected Bluetooth device using system audio players
- Providing a simple interface compatible with `ICommDevice` for integration in larger systems
- Operating as part of ROS 2 nodes or as a standalone audio playback component on Raspberry Pi or similar Linux devices

Note: Reading data from Bluetooth speaker devices is not supported, as these devices primarily serve as audio sinks.

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Integration with ROS 2

If you are using this library inside a ROS 2 package (e.g., mcqueen_hw):

- The package is set up with `ament_cmake`
- You can include and use `BluetoothAudioComm` inside your ROS 2 nodes
- Just add `mcqueen_hw_lib` as a dependency and link it in your `CMakeLists.txt`
- Create and manage `BluetoothAudioComm` objects in your ROS nodes similar to other communication devices

### Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Standalone Usage (Without ROS)

For non-ROS projects, you can build and use `BluetoothAudioComm` as a standalone component:

- Create a simple CMake project including the source files of `BluetoothAudioComm` and the `ICommDevice` interface
- Include necessary headers from your project’s `include` directory
- Use system audio players (e.g., `mpg123`) for playing audio files through the connected Bluetooth device
- No need to link BlueZ libraries explicitly since Bluetooth commands are executed via system calls
- Refer to your standalone example `CMakeLists.txt` for build configuration and integration

### Dependencies

- Linux system with BlueZ installed (used via `bluetoothctl` command-line tool)
- Audio player utility installed (e.g., `mpg123` for playing MP3 files)
- C++17 compatible compiler
- Standard system utilities (`bash`, `system()`) for command execution

### Installing Bluetooth Tools

On Debian/Ubuntu systems, install Bluetooth management tools with:

```bash
sudo apt-get update
sudo apt-get install bluetooth bluez
```

### Usage Example

```cpp
#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/BluetoothAudioComm.h"

#include <iostream>
#include <memory>

int main() {
    // Replace with your Bluetooth device's MAC address
    std::unique_ptr<ICommDevice> comm = std::make_unique<BluetoothAudioComm>("00:1A:7D:DA:71:13");

    if (!comm->open()) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    comm->write("/home/halil/music/senaydin.mp3");
    comm.close();

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
cd standalone/bluetooth_audio_comm
mkdir build && cd build
cmake ..
make
./bluetooth_audio_comm
```
