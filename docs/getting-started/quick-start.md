# Quick Start Guide

Welcome to the **Yara_OVE Experimental Playground**! This guide gets you up and running in 5 minutes.

**Yara_OVE** is an experimental environment for robotics research, featuring advanced physics simulation, accelerated learning capabilities, and comprehensive development tools.

## Prerequisites

- Ubuntu 20.04 LTS (Focal Fossa)
- Internet connection
- At least 5GB free disk space

## Required Components

Before starting, ensure you have installed:

- **ROS Noetic Desktop-Full** - Robotics framework
- **Gazebo Classic 11.15.1** - 3D simulation environment
- **Miniconda 25.7.0** - Python environment for development

*If you need to install these components, see our [installation guides](../installation/).*

## Quick Verification

### Test ROS Installation
```bash
# Check ROS version
rosversion -d
# Expected output: noetic

# Test basic ROS functionality
roscore &
sleep 2
rostopic list
pkill -f roscore
```

### Test Gazebo Simulation
```bash
# Launch Gazebo
roslaunch gazebo_ros empty_world.launch &
sleep 5
pkill -f roslaunch
```

### Test Python Environment
```bash
# Check conda version
conda --version
# Expected output: conda 25.7.0

# List environments
conda env list
```

## 🌊 YARA-OVE Ocean Simulation Quick Launch

Launch the ocean simulation with wave physics.

### Prerequisites Check
```bash
# Verify YARA-OVE is ready
ls yara-ove/  # Should show wave_gazebo, yara_description, etc.

# Check if workspace is built
ls ~/yara_ws/devel/setup.bash  # Should exist if catkin workspace is built
```

### Setup Workspace (One-time Setup)
```bash
# Initialize YARA-OVE submodule (if not done already)
./scripts/setup-yara-ove.sh

# Build the catkin workspace
cd ~/yara_ws
source /opt/ros/noetic/setup.bash
catkin_make

# Source the workspace
source ~/yara_ws/devel/setup.bash
echo "source ~/yara_ws/devel/setup.bash" >> ~/.bashrc
```

### Launch Ocean Simulation
```bash
# Navigate to workspace and source environment
cd ~/yara_ws && source devel/setup.bash

# Launch the ocean world with advanced wave physics
roslaunch wave_gazebo ocean_world.launch
```

**Expected Results:**
- **Gazebo GUI opens** with ocean environment
- **Wave animations** show Gerstner wave physics in real-time
- **Dynamic wave surface** responds to 1000m x 1000m wave field
- **GPU-accelerated rendering** provides visual performance

### Verify Ocean Physics
```bash
# In a new terminal, check wave physics topics
rostopic list | grep gazebo
# Expected: /gazebo/model_states, /gazebo/performance_metrics

# Monitor wave field performance
rostopic echo /gazebo/performance_metrics
# Look for real_time_factor close to 1.0

# Check wave model state
rostopic echo /gazebo/model_states | grep ocean_waves
```

### Quick Troubleshooting
```bash
# If launch fails, check common issues:

# 1. Verify wave_gazebo package
rospack find wave_gazebo
# Should output: /home/user/yara_ws/src/yara-ove/wave_gazebo

# 2. Check for missing dependencies
rosdep check --from-paths . --ignore-src
# Should show "All system dependencies have been satisfied"

# 3. Verify Gazebo can find wave plugins
ls /opt/ros/noetic/lib/ | grep -i wave
# Or check local build: ls ~/yara_ws/devel/lib/ | grep -i wave
```

### Confirmation
If you see:
- **Ocean waves** moving across the surface
- **No error messages** in terminal output
- **Stable frame rate** in Gazebo GUI
- **Topic data flowing** when checking with `rostopic echo`

**YARA-OVE ocean simulation is running.**

### What's Next?
1. **Wave Physics**: See [Wave Physics Analysis](../usage/gazebo-simulation.md#🌊-yara-ove-ocean-simulation)
2. **Different Scenarios**: Check [YARA-OVE Scenarios Guide](../usage/yara-ove-scenarios.md)
3. **Sailing Boats**: Follow [Advanced Workflows](../usage/advanced-workflows.md#🌊-yara-ove-sailing-robotics-workflows)
4. **System Details**: Study [Additional Resources](../resources/additional-resources.md)

## Next Steps

1. **New to robotics?** → Start with [Basic Commands](../usage/basic-commands.md)
2. **Ready to simulate?** → Try [Gazebo Simulation](../usage/gazebo-simulation.md)
3. **Developing with Python?** → Set up [Python Environments](../usage/python-environments.md)
4. **Need help?** → Check [Common Issues](../troubleshooting/common-issues.md)

## Learning Path

For a structured approach to robotics development, see our [Learning Path](../resources/learning-path.md).

---

**Welcome to Yara_OVE - Robotics Experimentation Platform**

*This experimental playground builds upon the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE), adapted for learning and experimentation.*