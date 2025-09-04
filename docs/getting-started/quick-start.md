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

## Next Steps

1. **New to robotics?** → Start with [Basic Commands](../usage/basic-commands.md)
2. **Ready to simulate?** → Try [Gazebo Simulation](../usage/gazebo-simulation.md)
3. **Developing with Python?** → Set up [Python Environments](../usage/python-environments.md)
4. **Need help?** → Check [Common Issues](../troubleshooting/common-issues.md)

## Learning Path

For a structured approach to robotics development, see our [Learning Path](../resources/learning-path.md).

---

**🚀 Welcome to Yara_OVE - Your Robotics Experimentation Platform!**

*This experimental playground builds upon the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE), adapted for learning and experimentation.*