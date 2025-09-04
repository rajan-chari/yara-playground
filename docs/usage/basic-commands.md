# Basic Commands

This guide provides essential commands for ROS Noetic, Gazebo simulation, and Miniconda environments used in the Yara_OVE experimental playground.

## ROS Commands

### Core ROS Operations
```bash
# Start ROS master
roscore

# Run a node
rosrun package_name node_name

# Launch multiple nodes
roslaunch package_name launch_file.launch

# Check ROS version
rosversion -d

# Get help for commands
rosrun --help
roslaunch --help
```

### Topic Operations
```bash
# List all topics
rostopic list

# Show topic information
rostopic info /topic_name

# Monitor topic data
rostopic echo /topic_name

# Publish to topic
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 2.0} angular: {z: 0.5}"

# Check topic data rate
rostopic hz /topic_name

# Show message types
rostopic type /topic_name
```

### Service Operations
```bash
# List all services
rosservice list

# Call a service
rosservice call /service_name

# Show service information
rosservice info /service_name

# Show service type
rosservice type /service_name
```

### Node Operations
```bash
# List running nodes
rosnode list

# Show node information
rosnode info /node_name

# Kill a node
rosnode kill /node_name

# Ping a node
rosnode ping /node_name
```

### Parameter Operations
```bash
# List all parameters
rosparam list

# Get parameter value
rosparam get /parameter_name

# Set parameter value
rosparam set /parameter_name value

# Delete parameter
rosparam delete /parameter_name

# Dump all parameters
rosparam dump params.yaml

# Load parameters
rosparam load params.yaml
```

### Package Operations
```bash
# List all packages
rospack list

# Find package location
rospack find package_name

# Show package dependencies
rospack depends package_name

# Create new package
catkin_create_pkg package_name dependencies
```

### Build System Commands
```bash
# Build all packages in workspace
catkin_make

# Build specific package
catkin_make --only-pkg-with-deps package_name

# Clean build
catkin_make clean

# Source workspace
source devel/setup.bash
```

## Gazebo Commands

### Basic Gazebo Operations
```bash
# Start Gazebo GUI
gazebo

# Start simulation server (headless)
gzserver

# Start visualization client
gzclient

# Start with specific world
gazebo worlds/world_name.world

# Get help
gazebo --help
```

### Gazebo with ROS Integration
```bash
# Launch empty world
roslaunch gazebo_ros empty_world.launch

# Launch with specific world
roslaunch gazebo_ros empty_world.launch world_name:=world_file.world

# Launch headless mode
roslaunch gazebo_ros empty_world.launch gui:=false

# Launch paused
roslaunch gazebo_ros empty_world.launch paused:=true
```

### Model Management
```bash
# Spawn model from file
rosrun gazebo_ros spawn_model -model model_name -file model.sdf

# Spawn model from parameter
rosrun gazebo_ros spawn_model -model model_name -param robot_description -urdf

# Remove model from simulation
rosservice call /gazebo/delete_model '{model_name: model_name}'

# List models in simulation
rostopic echo /gazebo/model_states -n 1
```

### Gazebo Services
```bash
# Pause simulation
rosservice call /gazebo/pause_physics

# Unpause simulation
rosservice call /gazebo/unpause_physics

# Reset simulation
rosservice call /gazebo/reset_simulation

# Reset world
rosservice call /gazebo/reset_world
```

### World Management
```bash
# Get world properties
rosservice call /gazebo/get_world_properties

# Set physics properties
rosservice call /gazebo/set_physics_properties
```

## YARA-OVE Sailing Boat Models

### Ocean World Launch
```bash
# Launch YARA-OVE ocean simulation
roslaunch wave_gazebo ocean_world.launch

# Launch with specific parameters
roslaunch wave_gazebo ocean_world.launch paused:=true gui:=true

# Launch headless for performance
roslaunch wave_gazebo ocean_world.launch gui:=false headless:=true
```

### EBoat Model (2.5m Research Vessel)
```bash
# Launch EBoat in ocean environment
roslaunch yara_ove_experiments eboat_ocean.launch

# Spawn EBoat model manually
rosrun gazebo_ros spawn_model -model eboat \
  -file $(rospack find yara_ove_models)/urdf/eboat.urdf \
  -urdf -x 0 -y 0 -z 0.1

# Monitor EBoat state
rostopic echo /eboat/state

# Control EBoat sail
rostopic pub /eboat/sail_cmd std_msgs/Float64 "data: 0.5"

# Control EBoat rudder
rostopic pub /eboat/rudder_cmd std_msgs/Float64 "data: 0.2"
```

### Fortune612 Model (0.99m RC Boat)
```bash
# Launch Fortune612 in ocean environment  
roslaunch yara_ove_experiments fortune612_ocean.launch

# Spawn Fortune612 model manually
rosrun gazebo_ros spawn_model -model fortune612 \
  -file $(rospack find yara_ove_models)/urdf/fortune612.urdf \
  -urdf -x 0 -y 0 -z 0.1

# Monitor Fortune612 state
rostopic echo /fortune612/state

# Control Fortune612 sail
rostopic pub /fortune612/sail_cmd std_msgs/Float64 "data: 0.3"

# Control Fortune612 rudder  
rostopic pub /fortune612/rudder_cmd std_msgs/Float64 "data: -0.1"
```

### Sailing Model Capabilities

#### EBoat (2.5m Research Vessel)
- **Length**: 2.5 meters
- **Physics**: 6-DOF sailing dynamics
- **Sensors**: GPS, IMU, wind sensor, compass
- **Control**: Sail servo, rudder servo, navigation
- **Applications**: Long-distance missions, research deployments
- **Simulation**: Sailing behavior in waves

```bash
# EBoat sensor topics
rostopic echo /eboat/gps/fix        # GPS coordinates
rostopic echo /eboat/imu/data       # Orientation data  
rostopic echo /eboat/wind_sensor    # Wind measurements
rostopic echo /eboat/compass        # Magnetic heading

# EBoat control topics
rostopic pub /eboat/waypoint geometry_msgs/PointStamped "..."
rostopic pub /eboat/sail_angle std_msgs/Float64 "data: 0.7"
rostopic pub /eboat/rudder_angle std_msgs/Float64 "data: 0.2"
```

#### Fortune612 (0.99m RC Boat)
- **Length**: 0.99 meters
- **Physics**: Sailing dynamics
- **Sensors**: GPS, IMU, wind sensor
- **Control**: Sail/rudder servos
- **Applications**: Testing, education, experiments
- **Simulation**: Sailing for development

```bash
# Fortune612 sensor topics
rostopic echo /fortune612/gps/fix      # GPS coordinates
rostopic echo /fortune612/imu/data     # Orientation data
rostopic echo /fortune612/wind_sensor  # Wind measurements

# Fortune612 control topics  
rostopic pub /fortune612/sail_cmd std_msgs/Float64 "data: 0.4"
rostopic pub /fortune612/rudder_cmd std_msgs/Float64 "data: -0.3"
```

### Sailing Maneuvers

#### Basic Sailing Commands
```bash
# Tacking maneuver (turn through wind)
rostopic pub /boat/maneuver std_msgs/String "data: 'tack'"

# Jibing maneuver (turn away from wind)  
rostopic pub /boat/maneuver std_msgs/String "data: 'jibe'"

# Point of sail commands
rostopic pub /boat/point_of_sail std_msgs/String "data: 'close_hauled'"
rostopic pub /boat/point_of_sail std_msgs/String "data: 'beam_reach'"
rostopic pub /boat/point_of_sail std_msgs/String "data: 'broad_reach'"
rostopic pub /boat/point_of_sail std_msgs/String "data: 'running'"
```

#### Autonomous Navigation
```bash
# Set waypoint navigation
rostopic pub /boat/waypoint_nav geometry_msgs/PointStamped \
  "header: {frame_id: 'map'} 
   point: {x: 100.0, y: 50.0, z: 0.0}"

# Follow GPS track
rosrun yara_ove_navigation gps_follower.py track.gpx

# Autonomous sailing mode
rostopic pub /boat/mode std_msgs/String "data: 'autonomous'"

# Manual control mode
rostopic pub /boat/mode std_msgs/String "data: 'manual'"
```

### Wave and Wind Interaction

#### Monitor Environmental Conditions
```bash
# Wave state information  
rostopic echo /ocean/wave_state

# Wind conditions
rostopic echo /ocean/wind_state

# Current simulation parameters
rostopic echo /ocean/parameters
```

#### Sailing Performance Metrics
```bash
# Boat velocity and heading
rostopic echo /boat/velocity

# Sailing efficiency metrics
rostopic echo /boat/performance

# Distance to waypoint
rostopic echo /boat/navigation/distance_to_waypoint

# Sailing angle relative to wind
rostopic echo /boat/sailing_angles
```

### Multi-Boat Scenarios

#### Fleet Management
```bash
# Launch multiple boats
roslaunch yara_ove_experiments multi_boat.launch num_boats:=3

# Individual boat control
rostopic pub /boat_1/sail_cmd std_msgs/Float64 "data: 0.5"
rostopic pub /boat_2/sail_cmd std_msgs/Float64 "data: 0.6"  
rostopic pub /boat_3/sail_cmd std_msgs/Float64 "data: 0.4"

# Fleet coordination
rostopic pub /fleet/formation std_msgs/String "data: 'line_abreast'"
rostopic pub /fleet/maneuver std_msgs/String "data: 'synchronized_tack'"
```

### Troubleshooting Sailing Models

#### Common Issues
```bash
# Check model spawn status
rostopic echo /gazebo/model_states | grep boat

# Verify sailing physics
rosservice call /gazebo/get_model_state '{model_name: "eboat"}'

# Reset boat position
rosservice call /gazebo/set_model_state \
  '{model_state: {model_name: "eboat", pose: {position: {x: 0, y: 0, z: 0.1}}}}'

# Check wind sensor data
rostopic echo /boat/wind_sensor --once

# Verify sail/rudder response
rostopic pub /boat/sail_cmd std_msgs/Float64 "data: 0.0" --once
rostopic pub /boat/rudder_cmd std_msgs/Float64 "data: 0.0" --once
```

#### Performance Optimization
```bash
# Reduce simulation complexity
roslaunch wave_gazebo ocean_world.launch wave_resolution:=25

# Enable fast simulation mode
rosparam set /use_sim_time true
rosparam set /simulation_speedup 10.0

# Monitor simulation performance
rostopic hz /clock
rostopic hz /gazebo/model_states
```


## Miniconda Commands

### Environment Management
```bash
# List environments
conda env list

# Create environment
conda create -n env_name python=3.9

# Create environment with packages
conda create -n env_name python=3.9 numpy matplotlib opencv pytorch

# Activate environment
conda activate env_name

# Deactivate environment
conda deactivate

# Remove environment
conda env remove -n env_name

# Export environment
conda env export > environment.yml

# Create environment from file
conda env create -f environment.yml
```

### Package Management
```bash
# Install package
conda install package_name

# Install from specific channel
conda install -c conda-forge package_name

# Install specific version
conda install package_name=1.2.3

# Update package
conda update package_name

# Update all packages
conda update --all

# Remove package
conda remove package_name

# List installed packages
conda list

# Search for packages
conda search package_name
```

### Pip Integration
```bash
# Install with pip (in conda environment)
pip install package_name

# List pip packages
pip list

# Show package info
pip show package_name

# Install from requirements
pip install -r requirements.txt

# Generate requirements
pip freeze > requirements.txt
```

### Conda Configuration
```bash
# Show configuration
conda config --show

# Add channel
conda config --add channels conda-forge

# Remove channel
conda config --remove channels channel_name

# Set channel priority
conda config --set channel_priority strict
```

### Information Commands
```bash
# Show conda version
conda --version

# Show system information
conda info

# Show environment information
conda info --envs

# Check for updates
conda update conda
```

## Example Workflows

### Basic ROS Development Workflow
```bash
# 1. Start ROS
roscore &

# 2. Build workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# 3. Launch nodes
rosrun package_name node_name &

# 4. Monitor topics
rostopic list
rostopic echo /topic_name

# 5. Cleanup
pkill -f roscore
```

### Simulation Workflow
```bash
# 1. Start ROS + Gazebo
roslaunch gazebo_ros empty_world.launch &

# 2. Spawn models
rosrun gazebo_ros spawn_model -model robot -file robot.urdf -urdf

# 3. Monitor simulation
rostopic echo /gazebo/model_states

# 4. Control simulation
rosservice call /gazebo/pause_physics
rosservice call /gazebo/unpause_physics

# 5. Cleanup
pkill -f roslaunch
```

### Python Development Workflow
```bash
# 1. Create environment
conda create -n my_env python=3.9 numpy matplotlib opencv

# 2. Activate environment
conda activate my_env

# 3. Install packages
conda install pytorch scikit-learn
pip install additional_packages

# 4. Develop code
python my_script.py

# 5. Export environment
conda env export > environment.yml

# 6. Deactivate environment
conda deactivate
```

## Quick Reference Cards

### ROS Essentials
```bash
roscore                    # Start ROS master
rosrun pkg node           # Run single node
roslaunch pkg file.launch # Launch multiple nodes
rostopic list             # List topics
rosnode list              # List nodes
rosservice list           # List services
```

### Gazebo Essentials
```bash
gazebo                                    # Start Gazebo
roslaunch gazebo_ros empty_world.launch  # ROS + Gazebo
rosrun gazebo_ros spawn_model            # Spawn models
rosservice call /gazebo/pause_physics    # Pause sim
```

### Conda Essentials
```bash
conda create -n env python=3.9  # Create environment
conda activate env              # Activate environment
conda install package           # Install package
conda deactivate                # Deactivate environment
conda env list                  # List environments
```

## Getting Help

### ROS Help
```bash
# Command help
rosrun --help
rostopic --help

# Package documentation
rospack find package_name
# Look for README or doc/ folder

# Online resources
# http://wiki.ros.org/
```

### Gazebo Help
```bash
# Command help
gazebo --help
gzserver --help

# Online resources
# http://classic.gazebosim.org/
```

### Conda Help
```bash
# Command help
conda --help
conda install --help

# Online resources
# https://docs.conda.io/
```

For advanced patterns and workflows, see [Advanced Workflows](advanced-workflows.md).

---

**⚡ Commands for Development**

*These commands form the foundation of development workflow in the Yara_OVE experimental playground.*