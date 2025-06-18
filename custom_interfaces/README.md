# Custom Interfaces ROS 2 Package

Custom ROS 2 interfaces (messages and services) used across the McQueen robot system.

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Dependencies

- ROS 2 (recommended [Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html))

## Package Overview

This package defines custom service and message types required for communication between various hardware and software components of the robot.

### Defined Interfaces

- **Services**:
  - `custom_interfaces/srv/Buzzer`: Control the robot's buzzer (on/off + duration).
  - `custom_interfaces/srv/Screen`: Display a string message on the screen.

### Use Case Examples

- `Buzzer` service is used to play sound feedback on event triggers.
- `Screen` service is used to display voice-recognized commands or sensor data.

## Installation

Start by creating a new ROS 2 workspace if you don't already have one:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

Use Git to download the `custom_interfaces` package from the repository:

```bash
git clone https://github.com/halilsenaydin/mcqueen
```

### Build Workspace

Navigate to the root of your workspace and build the package using `colcon`:

```bash
cd ~/workspace
colcon build --symlink-install
```

After building, don’t forget to source the setup file before running any ROS 2 commands:

```bash
source install/setup.bash
```

## Usage

You can call the services from any ROS 2 node:

```bash
ros2 service call /buzzer custom_interfaces/srv/Buzzer "{on: true, duration_ms: 500}"
ros2 service call /screen custom_interfaces/srv/Screen "{text: 'Hello, McQueen!'}"
```

These services should be implemented in the hardware communication layer, such as the [`mcqueen_hw`](../mcqueen_hw/README.md) package.
