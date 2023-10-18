# ros_component_diagram_generator

This is a fork of https://github.com/kenji-miyake/ros_component_diagram_generator .

I have modified and 

## Prerequisites

- ROS 2 Humble

## Installation

```bash
git clone https://github.com/xmfcx/ros_component_diagram_generator.git
```

## Usage

### launch2json

```bash
ros2 run launch_unifier launch2json --ros-args -p launch_command:="ros2 launch {package_name} {launch_file_name} {launch_args}"
```

Example commands using [Autoware](https://github.com/autowarefoundation/autoware):

```bash
# Setup Autoware following https://autowarefoundation.github.io/autoware-documentation/main/installation/

# Build launch_unifier
cd /path/to/launch_unifier
source /path/to/autoware/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run launch2json
source /path/to/launch_unifier/install/setup.bash
ros2 run launch_unifier launch2json --ros-args -p launch_command:="ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit" # Change the args according to your environment

# If you encounter the message "selected interface "lo" is not multicast-capable: disabling multicast", try the following command.
sudo ip link set multicast on lo
```
