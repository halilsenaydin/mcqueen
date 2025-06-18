# McQueen Description ROS 2 Package

A ROS 2 package that defines the robot description (URDF/XACRO) and visualization settings for the McQueen robot.

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Dependencies

- ROS 2 (recommended [Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html))

## Package Overview

Defines the complete robot model for McQueen using URDF and Xacro. Enables seamless integration with ROS 2 visualization and introspection tools.

### URDF/Xacro-Based Robot Model

- Full 3D URDF description with:
  - Base link, wheels, and sensors
  - Joint definitions
  - Inertial and visual properties

### Visualization Support

- Easily visualizable in `rviz2`
- Includes material definitions and RViz configuration files
- Launchable with GUI sliders using `joint_state_publisher_gui`

### ROS 2 Integration

- Publishes robot description to `/robot_description` using `robot_state_publisher`
- Compatible with `joint_state_publisher` and real/simulated feedback

### `ros2_control` Ready

- Includes hardware interface tag for `ros2_control` integration
- Pairs with [`mcqueen_hw`](../mcqueen_hw/README.md) package for real-time control

## Installation

Start by creating a new ROS 2 workspace if you don't already have one:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

Use Git to download the `mcqueen_description` package from the repository:

```bash
git clone https://github.com/halilsenaydin/mcqueen
```

### Install Dependencies

Before building the workspace, make sure all system and ROS 2 package dependencies are installed using rosdep:

```bash
# Navigate to the root of your ROS 2 workspace
cd ~/workspace

# Update rosdep database (recommended)
rosdep update

# Install all dependencies defined in package.xml files
rosdep install --from-paths src --ignore-src -r -y
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

Launches the URDF model along with the necessary nodes for robot visualization and joint publishing:

```bash
ros2 launch mcqueen_description mcqueen.launch.py
```

### Launching RViz2

To visualize the robot with a predefined RViz2 configuration, run the following command:

```bash
rviz2 -d ~/workspace/src/mcqueen_teleop/config/mcqueen.rviz
```

## Screenshots

### Rviz2 Visualization

![Rviz2 Visualization](docs/img/rviz2.png)
