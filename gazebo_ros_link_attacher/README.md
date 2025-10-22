# gazebo_ros_link_attacher

## Overview
The `gazebo_ros_link_attacher` package provides functionality for attaching and detaching links between models in the Gazebo simulation environment. This package is designed to work with ROS2 and allows users to manipulate the physical connections between models programmatically.

This is based on the original plug-in for ROS1 at: https://github.com/pal-robotics/gazebo_ros_link_attacher/

## Installation
To build and install the package, follow these steps:

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd gazebo_ros_link_attacher
   ```

2. **Install dependencies**:
   Make sure you have the necessary dependencies installed. You can use the following command to install them:
   ```bash
   sudo apt install ros-<ros-distro>-gazebo-ros-pkgs
   ```

3. **Build the package**:
   Use `colcon` to build the package:
   ```bash
   colcon build --packages-select gazebo_ros_link_attacher
   ```

4. **Source the setup file**:
   After building, source the setup file to overlay this workspace:
   ```bash
   source install/setup.bash
   ```

## Usage
To use the `gazebo_ros_link_attacher`, you can call the attach and detach services provided by the package. The service names are as follows:

- **Attach Service**: `/attach`
- **Detach Service**: `/detach`

### Example
To attach two links, you can use the following command:
```bash
ros2 service call /attach gazebo_model_1 link_1 gazebo_model_2 link_2
```

To detach the links, use:
```bash
ros2 service call /detach gazebo_model_1 link_1 gazebo_model_2 link_2
```

## Dependencies
This package depends on the following ROS2 packages:
- `gazebo_ros_pkgs`
- `rclcpp`
- `std_msgs`

## Maintainers
- Your Name <pedro.ribeiro@york.ac.uk>

## License
This project is licensed under the BSD License. See the LICENSE file for more details.