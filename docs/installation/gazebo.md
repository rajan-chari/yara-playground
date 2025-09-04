# Gazebo Classic Installation

## Installation Status

✅ **Gazebo Classic 11.15.1 is ready for use with Yara_OVE.**

This guide documents the Gazebo installation configured for robotic simulation.

## What Was Installed

### Core Components
- **Gazebo Classic 11.15.1** - 3D robotic simulation platform
- **Installation Method**: Via ROS repository for seamless integration
- **Location**: `/usr/bin/gazebo`, `/usr/share/gazebo-11/`
- **Integration**: Full ROS Noetic compatibility

### Key Features
- **Physics Engine**: ODE, Bullet, DART support
- **Visualization**: Real-time 3D rendering
- **Sensors**: Camera, lidar, IMU, GPS, and more
- **Plugins**: Extensible architecture for custom functionality
- **Models**: Extensive model library and URDF/SDF support

## ROS-Gazebo Integration Packages

The following packages provide seamless ROS-Gazebo integration:

### Core Integration
- **`gazebo_ros`** - Basic ROS-Gazebo interface
- **`gazebo_plugins`** - Sensor and actuator plugins
- **`gazebo_msgs`** - ROS message definitions for Gazebo
- **`gazebo_ros_control`** - Hardware interface integration

### Available Tools
```bash
# Core executables
gazebo           # Complete Gazebo (server + client)
gzserver         # Gazebo physics server
gzclient         # Gazebo GUI client

# ROS integration tools
spawn_model      # Spawn models via ROS
gz               # Gazebo command-line tools
```

## Installation Process Completed

### 1. Automatic Installation via ROS
Gazebo was installed automatically with ROS Noetic Desktop-Full:

```bash
# This command installed both ROS and Gazebo
sudo apt install ros-noetic-desktop-full
```

### 2. ROS Integration Packages
```bash
# These were included in the Desktop-Full installation
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### 3. Verification Completed
```bash
# Version check
gazebo --version
# Output: Gazebo multi-robot simulator, version 11.15.1

# ROS integration test
roslaunch gazebo_ros empty_world.launch
```

## Directory Structure

```
/usr/share/gazebo-11/
├── media/
│   ├── materials/     # Textures and materials
│   └── models/        # Default model library
├── worlds/            # Pre-built simulation worlds
└── setup.sh          # Gazebo environment setup

/opt/ros/noetic/share/
├── gazebo_ros/        # ROS-Gazebo bridge
├── gazebo_plugins/    # ROS sensor plugins
└── gazebo_msgs/       # ROS message definitions
```

## Environment Variables

Gazebo uses these environment variables (optional for basic usage):

```bash
# Model search paths (not set by default)
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models

# Plugin search paths
GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib

# Resource paths
GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
```

## Verification Commands

Test your Gazebo installation:

```bash
# Check version
gazebo --version

# Test standalone Gazebo (GUI will open)
gazebo

# Test ROS integration
roslaunch gazebo_ros empty_world.launch

# Check available worlds
ls /usr/share/gazebo-11/worlds/

# Test model spawning
rosrun gazebo_ros spawn_model -help
```

## Common Launch Commands

### Basic Gazebo with ROS
```bash
# Empty world
roslaunch gazebo_ros empty_world.launch

# Specific world file
roslaunch gazebo_ros empty_world.launch world_name:=willowgarage.world

# Headless mode (no GUI)
roslaunch gazebo_ros empty_world.launch gui:=false

# With custom parameters
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=true
```

### Model Management
```bash
# Spawn a model from file
rosrun gazebo_ros spawn_model -model my_robot -file model.sdf

# Spawn from ROS parameter
rosrun gazebo_ros spawn_model -model my_robot -param robot_description -urdf

# Delete a model
rosservice call /gazebo/delete_model '{model_name: my_robot}'
```

## Integration with ROS Topics

Gazebo publishes and subscribes to several ROS topics:

```bash
# Check Gazebo-related topics
rostopic list | grep gazebo

# Common topics:
/gazebo/link_states       # All link positions/velocities
/gazebo/model_states      # All model positions/velocities
/gazebo/parameter_updates # Parameter changes
/clock                    # Simulation time (when use_sim_time:=true)
```

## What's Next

- **Simulation Usage**: See [Gazebo Simulation](../usage/gazebo-simulation.md) for practical usage
- **Advanced Workflows**: Try [Advanced Workflows](../usage/advanced-workflows.md) for complex scenarios
- **Troubleshooting**: Check [Gazebo-Specific Issues](../troubleshooting/gazebo-specific.md)
- **Model Development**: Explore robot model creation and custom environments

## Resources

- [Official Gazebo Classic Documentation](http://classic.gazebosim.org/) - Complete reference
- [Gazebo ROS Integration Guide](http://gazebosim.org/tutorials?cat=connect_ros) - ROS integration tutorials
- [Model Database](https://github.com/osrf/gazebo_models) - Community model repository
- [Plugin Development](http://gazebosim.org/tutorials?cat=write_plugin) - Custom functionality development

---

**🚀 Gazebo Classic: Ready for Robotic Simulation!**

*This Gazebo installation provides the simulation environment needed for the Yara_OVE experimental playground.*