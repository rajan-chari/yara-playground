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

**⚡ Master These Commands for Effective Development!**

*These commands form the foundation of your development workflow in the Yara_OVE experimental playground.*