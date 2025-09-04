# Quick Start Guide

This guide gets you up and running with ROS Noetic, Gazebo, and Miniconda in 5 minutes.

## Prerequisites

- Ubuntu 20.04 LTS (Focal Fossa)
- Internet connection
- At least 5GB free disk space

## Installation Status

✅ **All components are already installed and configured on this system:**

- **ROS Noetic Desktop-Full** - Complete robotics framework
- **Gazebo Classic 11.15.1** - 3D robot simulation
- **Miniconda 25.7.0** - Python environment management

## Quick Verification

### Test ROS
```bash
# Check ROS installation
rosversion -d
# Expected output: noetic

# Test basic ROS functionality
roscore &
sleep 2
rostopic list
pkill -f roscore
```

### Test Gazebo
```bash
# Launch Gazebo (will open GUI)
roslaunch gazebo_ros empty_world.launch &
sleep 5
pkill -f roslaunch
```

### Test Miniconda
```bash
# Check conda installation
conda --version
# Expected output: conda 25.7.0

# List environments
conda env list
```

## Next Steps

1. **New to ROS?** → Start with [Basic Commands](../usage/basic-commands.md)
2. **Ready to simulate?** → Try [Gazebo Simulation](../usage/gazebo-simulation.md)  
3. **Python development?** → Set up [Python Environments](../usage/python-environments.md)
4. **Need help?** → Check [Common Issues](../troubleshooting/common-issues.md)

## Learning Path

For a structured learning approach, see our [Learning Path](../resources/learning-path.md).