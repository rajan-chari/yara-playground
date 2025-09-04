# Basic Commands Reference

This guide provides essential commands for working with ROS Noetic, Gazebo, and Miniconda.

## ROS Commands

### Core ROS Operations
```bash
# Start ROS master
roscore

# Run a ROS node
rosrun package_name node_name

# Launch multiple nodes
roslaunch package_name launch_file.launch

# Check ROS version
rosversion -d

# Get help for any ROS command
rosrun --help
roslaunch --help
```

### Topic Operations
```bash
# List all topics
rostopic list

# Show topic information
rostopic info /topic_name

# Echo topic messages
rostopic echo /topic_name

# Publish to a topic
rostopic pub /topic_name message_type "data"

# Get topic message rate
rostopic hz /topic_name

# Show topic message type
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
# Start Gazebo with GUI
gazebo

# Start Gazebo server only (headless)
gzserver

# Start Gazebo client only
gzclient

# Start with specific world
gazebo worlds/willowgarage.world

# Get help
gazebo --help
```

### Gazebo with ROS
```bash
# Launch empty world
roslaunch gazebo_ros empty_world.launch

# Launch with specific world
roslaunch gazebo_ros empty_world.launch world_name:=willowgarage.world

# Launch without GUI
roslaunch gazebo_ros empty_world.launch gui:=false

# Launch paused
roslaunch gazebo_ros empty_world.launch paused:=true
```

### Model Management
```bash
# Spawn model from file
rosrun gazebo_ros spawn_model -model my_robot -file robot.sdf

# Spawn model from parameter
rosrun gazebo_ros spawn_model -model my_robot -param robot_description -urdf

# Delete model
rosservice call /gazebo/delete_model '{model_name: my_robot}'

# List models
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

## Miniconda Commands

### Environment Management
```bash
# List environments
conda env list

# Create environment
conda create -n env_name python=3.9

# Create environment with packages
conda create -n env_name python=3.9 numpy pandas

# Activate environment
conda activate env_name

# Deactivate environment
conda deactivate

# Remove environment
conda env remove -n env_name

# Export environment
conda env export > environment.yml

# Create from file
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

## Common Workflows

### ROS Development Workflow
```bash
# 1. Start ROS
roscore &

# 2. Build workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# 3. Run nodes
rosrun my_package my_node &

# 4. Monitor topics
rostopic list
rostopic echo /my_topic

# 5. Cleanup
pkill -f roscore
```

### Gazebo Simulation Workflow
```bash
# 1. Start ROS + Gazebo
roslaunch gazebo_ros empty_world.launch &

# 2. Spawn robot
rosrun gazebo_ros spawn_model -model my_robot -file robot.urdf -urdf

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
conda create -n my_project python=3.9 numpy matplotlib

# 2. Activate environment
conda activate my_project

# 3. Install additional packages
conda install scikit-learn
pip install some_package

# 4. Work on project
python my_script.py

# 5. Export environment
conda env export > environment.yml

# 6. Deactivate
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

For more advanced usage patterns, see [Advanced Workflows](advanced-workflows.md).