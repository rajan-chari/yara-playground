# Common Issues and Solutions for Sailing Robotics

This guide covers the most frequently encountered issues when working with ROS, Gazebo, and conda environments in the Yara_OVE experimental playground for autonomous sailing robotics.

## General System Issues

### Permission Errors
**Problem**: Permission denied errors when running commands
```bash
bash: /opt/ros/noetic/setup.bash: Permission denied
```

**Solutions**:
```bash
# Fix file permissions
sudo chmod +x /opt/ros/noetic/setup.bash

# Check if you're in the correct groups
groups $USER

# Add user to necessary groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# Log out and back in for group changes to take effect
```

### Environment Variables Not Set
**Problem**: ROS commands not found or environment not properly configured for sailing robotics
```bash
roscore: command not found
```

**Solutions**:
```bash
# Manually source ROS environment for sailing robotics
source /opt/ros/noetic/setup.bash

# Add to .bashrc permanently for Yara_OVE development
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify sailing robotics environment
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
rospack find yara_ove_simulation  # Verify Yara_OVE packages
```

### Network Configuration Issues
**Problem**: ROS nodes can't communicate between machines (important for sailing robot remote control)
```bash
ROS_MASTER_URI not connecting
```

**Solutions**:
```bash
# Check network configuration for sailing robot deployment
hostname -I
echo $ROS_MASTER_URI
echo $ROS_HOSTNAME

# Set proper network variables for local sailing simulation
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# For multi-machine setup (sailing robot + shore station)
export ROS_MASTER_URI=http://SAILING_ROBOT_IP:11311
export ROS_IP=$(hostname -I | cut -d' ' -f1)

# Test sailing robot connectivity
rostopic list | grep sailing_robot
rosnode list | grep navigation
```

## Installation and Package Issues

### Broken Package Dependencies
**Problem**: Package installation fails due to dependency conflicts
```bash
E: Unable to locate package ros-noetic-*
dpkg: dependency problems prevent configuration
```

**Solutions**:
```bash
# Update package lists
sudo apt update

# Fix broken packages
sudo apt --fix-broken install

# Clean package cache
sudo apt clean
sudo apt autoclean

# Reinstall problematic packages
sudo apt remove --purge package-name
sudo apt install package-name

# Reset ROS repository keys
wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### catkin_make Build Failures
**Problem**: Sailing robotics workspace fails to build
```bash
CMake Error: Could not find package configuration file
```

**Solutions**:
```bash
# Clean sailing robotics build artifacts
cd ~/sailing_ws/  # or ~/catkin_ws/
rm -rf build/ devel/

# Source ROS environment before building Yara_OVE packages
source /opt/ros/noetic/setup.bash
catkin_make

# Check for missing sailing robotics dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Debug specific sailing package
catkin_make --pkg yara_ove_navigation
catkin_make --pkg sailing_control

# Verbose output for debugging sailing robotics
catkin_make --verbose
```

### Missing Python Dependencies
**Problem**: Python packages not found in ROS context
```bash
ModuleNotFoundError: No module named 'rospy'
ImportError: No module named cv_bridge
```

**Solutions**:
```bash
# Install missing ROS Python packages
sudo apt install python3-rospy python3-roslib
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv

# For conda environments
conda activate your_env
pip install rospkg catkin_pkg

# Fix Python path issues
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Check Python path
python3 -c "import sys; print('\\n'.join(sys.path))"
```

## Runtime Issues

### roscore Won't Start
**Problem**: ROS master fails to initialize
```bash
ERROR: unable to start XML-RPC server
Address already in use
```

**Solutions**:
```bash
# Check if roscore is already running
ps aux | grep roscore

# Kill existing roscore
killall -9 roscore rosmaster

# Check port usage
sudo netstat -tulpn | grep 11311
sudo lsof -i :11311

# Kill process using ROS port
sudo kill -9 $(sudo lsof -t -i:11311)

# Start fresh roscore
roscore
```

### Node Communication Problems
**Problem**: Sailing robot nodes can't find each other
```bash
ERROR: service [/sailing_robot/set_waypoint] unavailable
```

**Solutions**:
```bash
# Check running sailing nodes
rosnode list | grep sailing
rosnode info /sailing_robot/navigation

# Check sailing topics
rostopic list | grep sailing_robot
rostopic info /sailing_robot/wind_data
rostopic info /sailing_robot/gps

# Check sailing services
rosservice list | grep sailing
rosservice info /sailing_robot/set_waypoint

# Test sailing communication
rostopic echo /sailing_robot/wind_data
rostopic echo /sailing_robot/sail_angle
rosservice call /sailing_robot/emergency_stop

# Check sailing ROS graph
rqt_graph  # Look for sailing robot connections

# Restart problematic sailing nodes
rosnode kill /sailing_robot/navigation
roslaunch yara_ove_navigation sailing_navigation.launch
```

### High CPU/Memory Usage
**Problem**: System becomes slow or unresponsive
```bash
System hanging, high load average
```

**Solutions**:
```bash
# Monitor system resources
htop
iostat 1

# Check ROS node resource usage
rosnode list | xargs -I {} sh -c 'echo "Node: {}"; ps aux | grep {}'

# Limit node resource usage
nice -n 10 rosrun package_name node_name
cpulimit -l 50 -p PID

# Kill resource-heavy nodes
rosnode kill /heavy_node
pkill -f "python.*heavy_script"

# Monitor bag file recording
rosbag info file.bag
# Stop if too large
ps aux | grep rosbag
kill PID
```

## File System Issues

### Disk Space Problems
**Problem**: No space left on device
```bash
OSError: [Errno 28] No space left on device
```

**Solutions**:
```bash
# Check disk usage
df -h
du -sh ~/catkin_ws/
du -sh ~/.ros/log/

# Clean log files
rm -rf ~/.ros/log/*
find ~/.ros -name "*.log" -delete

# Clean build artifacts
cd ~/catkin_ws/
rm -rf build/ devel/

# Clean conda cache
conda clean --all

# Clean apt cache
sudo apt clean
sudo apt autoremove

# Find large files
find ~/ -type f -size +100M 2>/dev/null | head -10
```

### File Permission Issues
**Problem**: Can't write to workspace or log directories
```bash
Permission denied: '/home/user/catkin_ws/src'
```

**Solutions**:
```bash
# Fix workspace permissions
sudo chown -R $USER:$USER ~/catkin_ws/
chmod -R 755 ~/catkin_ws/

# Fix ROS log permissions
sudo chown -R $USER:$USER ~/.ros/
chmod -R 755 ~/.ros/

# Check current permissions
ls -la ~/catkin_ws/
ls -la ~/.ros/

# Create directories with proper permissions
mkdir -p ~/catkin_ws/src
chmod 755 ~/catkin_ws/src
```

## Launch File Issues

### Launch File Syntax Errors
**Problem**: Launch files fail to parse
```bash
Invalid roslaunch XML syntax
```

**Solutions**:
```bash
# Validate XML syntax
xmllint --noout file.launch

# Check roslaunch syntax
roslaunch --check package_name launch_file.launch

# Common fixes:
# 1. Ensure proper XML structure
<?xml version="1.0"?>
<launch>
  <!-- content -->
</launch>

# 2. Escape special characters
<param name="param" value="&lt;value&gt;"/>

# 3. Check unclosed tags
<node pkg="package" type="node" name="node_name"/>
# not: <node pkg="package" type="node" name="node_name">
```

### Parameter Loading Issues
**Problem**: Parameters not loading correctly from launch files
```bash
WARN: parameter [/param_name] not set
```

**Solutions**:
```bash
# Check parameter loading
rosparam list
rosparam get /param_name

# Debug launch file parameter loading
roslaunch --screen package_name file.launch

# Verify parameter file syntax
rosparam load params.yaml /namespace
rosparam dump /tmp/current_params.yaml

# Common parameter file formats:
# YAML format
param_name: value
nested:
  param: value

# Load parameters manually
rosparam load params.yaml
rosparam set /param_name value
```

## Data Recording and Playback

### Sailing Data Bag File Issues
**Problem**: Sailing session bag files corrupted or won't play
```bash
rosbag play: error reading sailing_session.bag file
```

**Solutions**:
```bash
# Check sailing bag file integrity
rosbag check sailing_session.bag
rosbag info sailing_session.bag

# Repair corrupted sailing bag
rosbag reindex sailing_session.bag
rosbag fix sailing_session.bag sailing_session_fixed.bag

# Filter problematic sailing topics
rosbag filter sailing_session.bag clean_session.bag 'topic != "/sailing_robot/camera/image_raw"'

# Extract specific sailing data
rosbag filter sailing_session.bag wind_data.bag 'topic == "/sailing_robot/wind_data"'
rosbag filter sailing_session.bag gps_data.bag 'topic == "/sailing_robot/gps"'

# Convert sailing bag formats
rosbag compress sailing_session.bag
rosbag decompress sailing_session.bag

# Split sailing session by time
rosbag filter sailing_session.bag morning_sail.bag 't.secs >= 1000 and t.secs <= 5000'
```

### Clock Synchronization Issues
**Problem**: Time synchronization problems during playback
```bash
TF_OLD_DATA ignoring data from the past
```

**Solutions**:
```bash
# Use simulation time during playback
rosparam set use_sim_time true
rosbag play --clock file.bag

# Reset simulation time after playback
rosparam set use_sim_time false

# Check time synchronization
rostopic echo /clock
rostopic echo /tf

# Fix TF timing issues
rosrun tf view_frames
rosrun rqt_tf_tree rqt_tf_tree
```

## Quick Diagnostic Commands

### System Health Check
```bash
#!/bin/bash
# quick_diagnostic.sh - Quick system health check

echo "=== ROS Environment Check ==="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"

echo -e "\n=== Running Processes ==="
ps aux | grep -E "(roscore|rosmaster|gzserver|gzclient)" | grep -v grep

echo -e "\n=== Network Check ==="
ping -c 1 localhost >/dev/null 2>&1 && echo "localhost: OK" || echo "localhost: FAIL"

echo -e "\n=== Disk Space ==="
df -h | grep -E "(/$|/home)"

echo -e "\n=== Memory Usage ==="
free -h

echo -e "\n=== ROS Nodes ==="
if pgrep roscore >/dev/null; then
    timeout 5s rosnode list 2>/dev/null || echo "rosnode list timeout"
else
    echo "roscore not running"
fi

echo -e "\n=== Conda Environment ==="
conda info --envs 2>/dev/null || echo "conda not available"

echo -e "\n=== Recent Errors ==="
tail -5 ~/.ros/log/latest/roslaunch-*.log 2>/dev/null || echo "No recent ROS logs"
```

### Environment Reset Script
```bash
#!/bin/bash
# reset_environment.sh - Clean reset of ROS environment

echo "Stopping all ROS processes..."
killall -9 roscore rosmaster gzserver gzclient 2>/dev/null

echo "Cleaning log files..."
rm -rf ~/.ros/log/*

echo "Cleaning build artifacts..."
if [ -d ~/catkin_ws ]; then
    cd ~/catkin_ws
    rm -rf build/ devel/
fi

echo "Resetting environment variables..."
unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
fi

echo "Starting fresh roscore..."
roscore &
sleep 2

echo "Environment reset complete!"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
```

## Getting Help

### Log File Locations
```bash
# ROS logs
~/.ros/log/latest/
~/.ros/log/latest/roslaunch-*.log

# System logs
/var/log/syslog
journalctl -u roscore

# Application logs
~/catkin_ws/logs/
```

### Debugging Commands
```bash
# Verbose ROS output
roslaunch --screen package_name file.launch

# Debug node startup
rosrun --prefix 'gdb -ex run --args' package_name node_name

# Monitor system resources
htop
iotop
nethogs

# Network diagnostics
ss -tulpn | grep ros
tcpdump -i lo port 11311
```

### Community Resources
- **ROS Answers**: [answers.ros.org](https://answers.ros.org)
- **ROS Wiki Troubleshooting**: [wiki.ros.org/ROS/Troubleshooting](http://wiki.ros.org/ROS/Troubleshooting)
- **GitHub Issues**: Check package-specific repositories
- **ROS Discourse**: [discourse.ros.org](https://discourse.ros.org)

## Next Steps

When basic troubleshooting doesn't resolve your issue:

1. **Check component-specific guides**: 
   - [ROS-Specific Issues](ros-specific.md)
   - [Gazebo-Specific Issues](gazebo-specific.md)
   - [Conda-Specific Issues](conda-specific.md)

2. **Create a minimal reproduction case**
3. **Gather relevant logs and error messages**
4. **Ask for help on community forums with detailed information**

## Related Documentation

- **Installation**: [Sailing Robotics Installation](../installation/) for proper Yara_OVE setup
- **Basic Commands**: [Sailing Robot Commands](../usage/basic-commands.md) for fundamental sailing operations
- **Verification**: [Installation Verification](../installation/verification.md) for sailing robotics system validation
- **Sailing-Specific Issues**:
  - [ROS-Specific Issues](ros-specific.md) for sailing robot ROS problems
  - [Gazebo-Specific Issues](gazebo-specific.md) for marine simulation issues
  - [Conda-Specific Issues](conda-specific.md) for sailing AI environment problems