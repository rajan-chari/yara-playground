# ROS-Specific Troubleshooting

This guide focuses on issues specific to ROS Noetic installation, configuration, and operation.

## ROS Installation Issues

### Repository and Key Problems
**Problem**: Can't add ROS repository or GPG key errors
```bash
W: GPG error: http://packages.ros.org/ros/ubuntu focal InRelease
E: The repository 'http://packages.ros.org/ros/ubuntu focal InRelease' is not signed
```

**Solutions**:
```bash
# Remove old keys
sudo apt-key del $(apt-key list 2>/dev/null | grep -A1 "ROS" | grep pub | cut -d'/' -f2 | cut -d' ' -f1)

# Add fresh ROS key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Alternative method using newer apt-key format
wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc

# Update repository information
sudo apt update

# Verify repository
apt-cache policy | grep ros
```

### Package Installation Failures
**Problem**: ROS packages fail to install
```bash
E: Package 'ros-noetic-desktop-full' has no installation candidate
```

**Solutions**:
```bash
# Verify Ubuntu version compatibility
lsb_release -a
# ROS Noetic requires Ubuntu 20.04 (Focal)

# Check repository configuration
cat /etc/apt/sources.list.d/ros-latest.list
# Should contain: deb http://packages.ros.org/ros/ubuntu focal main

# Fix repository for correct Ubuntu version
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Clear package cache and update
sudo apt clean
sudo apt update

# Try installation again
sudo apt install ros-noetic-desktop-full
```

### rosdep Initialization Problems
**Problem**: rosdep fails to initialize or update
```bash
ERROR: cannot download default sources list from:
https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
```

**Solutions**:
```bash
# Clear existing rosdep data
sudo rm -rf /etc/ros/rosdep/
rm -rf ~/.ros/rosdep/

# Reinitialize rosdep
sudo rosdep init
rosdep update

# If network issues persist, use alternative method
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo wget -O /etc/ros/rosdep/sources.list.d/20-default.list https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list

# Update with verbose output for debugging
rosdep update --verbose

# Check rosdep status
rosdep --version
ls -la /etc/ros/rosdep/sources.list.d/
```

## Environment Configuration Issues

### Source Configuration Problems
**Problem**: ROS environment not properly sourced
```bash
bash: roscore: command not found
ROS_DISTRO environment variable not set
```

**Solutions**:
```bash
# Check current environment
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
echo $CMAKE_PREFIX_PATH

# Manual sourcing for testing
source /opt/ros/noetic/setup.bash
echo $ROS_DISTRO  # Should output: noetic

# Add to shell configuration
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# For zsh users
echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc

# Reload shell configuration
source ~/.bashrc

# Verify all variables are set
env | grep ROS
```

### Workspace Configuration Issues
**Problem**: Catkin workspace not properly configured
```bash
Package not found in workspace
catkin_make fails with CMake errors
```

**Solutions**:
```bash
# Verify workspace structure
ls -la ~/catkin_ws/
# Should have: src/ build/ devel/

# Recreate workspace if corrupted
rm -rf ~/catkin_ws/build/ ~/catkin_ws/devel/
cd ~/catkin_ws/
catkin_make

# Source workspace after building
source ~/catkin_ws/devel/setup.bash

# Add workspace sourcing to bashrc (after ROS sourcing)
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Verify workspace is in ROS package path
echo $ROS_PACKAGE_PATH
# Should include: /home/user/catkin_ws/src
```

### Python Path Issues
**Problem**: Python can't find ROS packages
```bash
ImportError: No module named rospy
ImportError: No module named geometry_msgs.msg
```

**Solutions**:
```bash
# Check Python version compatibility
python3 --version
# ROS Noetic uses Python 3

# Verify ROS Python packages are installed
dpkg -l | grep python3-rospy
sudo apt install python3-rospy python3-roslib

# Check Python path includes ROS
python3 -c "import sys; print('\n'.join(sys.path))"
# Should include: /opt/ros/noetic/lib/python3/dist-packages

# Add ROS Python path manually if missing
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Test ROS Python imports
python3 -c "import rospy; print('rospy import successful')"
python3 -c "import geometry_msgs; print('geometry_msgs import successful')"
```

## Runtime Issues

### Master Connection Problems
**Problem**: Nodes can't connect to ROS master
```bash
Unable to register with master node [http://localhost:11311]: master may not be running yet
```

**Solutions**:
```bash
# Check if roscore is running
ps aux | grep roscore
pgrep roscore

# Check ROS master URI
echo $ROS_MASTER_URI
# Should be: http://localhost:11311

# Start roscore if not running
roscore &

# Test master connection
rostopic list
rosnode list

# For remote master connection
export ROS_MASTER_URI=http://REMOTE_IP:11311
export ROS_IP=$(hostname -I | cut -d' ' -f1)

# Test connection to remote master
ping REMOTE_IP
telnet REMOTE_IP 11311
```

### Node Communication Issues
**Problem**: ROS nodes can't communicate
```bash
WARN: no messages received and simulated time is active
ERROR: service [/service_name] unavailable
```

**Solutions**:
```bash
# Check node status
rosnode list
rosnode ping /node_name

# Check topic connectivity
rostopic list
rostopic info /topic_name
rostopic hz /topic_name

# Debug communication with graph
rosrun rqt_graph rqt_graph

# Check for time synchronization issues
rosparam get use_sim_time
# If true, ensure clock publisher is running
rostopic echo /clock

# Reset time if needed
rosparam set use_sim_time false

# Check network configuration
echo $ROS_HOSTNAME
echo $ROS_IP
# Both should be set for multi-machine setups
```

### Package and Node Issues
**Problem**: ROS packages or nodes not found
```bash
[rospack] Error: package 'my_package' not found
[rosrun] Couldn't find executable named my_node
```

**Solutions**:
```bash
# Update package cache
rospack profile

# Check package path
rospack find package_name
echo $ROS_PACKAGE_PATH

# List available packages
rospack list | grep package_name

# For workspace packages, rebuild
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# Check executable permissions
ls -la ~/catkin_ws/src/package/scripts/
chmod +x ~/catkin_ws/src/package/scripts/node_name

# Verify package.xml and CMakeLists.txt
rospack depends package_name
```

## Message and Service Issues

### Message Compilation Problems
**Problem**: Custom messages not building
```bash
Could not find the required component 'my_msgs'
No such file or directory: my_msgs/SomeMessage.h
```

**Solutions**:
```bash
# Check message package structure
ls ~/catkin_ws/src/my_msgs/
# Should have: msg/ CMakeLists.txt package.xml

# Verify CMakeLists.txt message configuration
grep -A 10 "find_package" ~/catkin_ws/src/my_msgs/CMakeLists.txt
grep -A 5 "add_message_files" ~/catkin_ws/src/my_msgs/CMakeLists.txt
grep "generate_messages" ~/catkin_ws/src/my_msgs/CMakeLists.txt

# Required CMakeLists.txt sections:
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  YourMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Check package.xml dependencies
grep -A 5 "<depend>" ~/catkin_ws/src/my_msgs/package.xml
# Should include message_runtime

# Rebuild messages
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps my_msgs
source devel/setup.bash

# Verify message generation
rosmsg show my_msgs/YourMessage
```

### TF (Transform) Issues
**Problem**: Transform frame errors
```bash
Lookup would require extrapolation into the past
Frame [frame_id] does not exist
```

**Solutions**:
```bash
# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Monitor TF in real-time
rosrun tf tf_monitor

# Check specific transform
rosrun tf tf_echo source_frame target_frame

# Debug TF timing
rosparam set use_sim_time true  # If using simulation
rostopic echo /tf

# Visualize TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Check for missing transforms
rostopic echo /tf_static
```

### Parameter Server Issues
**Problem**: Parameters not loading or accessible
```bash
Parameter [/param_name] is not set
yaml.scanner.ScannerError: while scanning
```

**Solutions**:
```bash
# List all parameters
rosparam list

# Check parameter value
rosparam get /param_name

# Load parameters from file
rosparam load params.yaml /namespace

# Check YAML syntax
python3 -c "import yaml; yaml.safe_load(open('params.yaml'))"

# Set parameters manually
rosparam set /param_name value

# Dump current parameters for debugging
rosparam dump /tmp/current_params.yaml

# Delete problematic parameters
rosparam delete /param_name
```

## Build System Issues

### catkin_make Problems
**Problem**: Workspace build failures
```bash
CMake Error at CMakeLists.txt:1 (cmake_minimum_required)
Could not find a package configuration file provided by "catkin"
```

**Solutions**:
```bash
# Clean build directory
cd ~/catkin_ws/
rm -rf build/ devel/

# Source ROS before building
source /opt/ros/noetic/setup.bash

# Build with verbose output
catkin_make --verbose

# Build specific package
catkin_make --only-pkg-with-deps package_name

# Check CMake version
cmake --version
# Should be 3.10+

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# For Python node permissions
find ~/catkin_ws/src -name "*.py" -exec chmod +x {} \;
```

### Dependency Resolution
**Problem**: Package dependencies not resolved
```bash
Package 'dependency_name' not found
rosdep: command not found
```

**Solutions**:
```bash
# Update rosdep database
rosdep update

# Install dependencies for all packages
rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y

# Check specific package dependencies
rosdep check package_name

# Install specific dependency
sudo apt install ros-noetic-dependency-name

# For custom dependencies, check package.xml
cat ~/catkin_ws/src/package_name/package.xml
```

## Advanced Debugging

### ROS Logging and Debugging
```bash
# Set logging level
export ROSCONSOLE_CONFIG_FILE=/path/to/rosconsole.config

# Create debug config file
cat > rosconsole.config << 'EOF'
log4j.logger.ros=DEBUG
log4j.logger.ros.package_name=DEBUG
EOF

# Use roslaunch with screen output
roslaunch --screen package_name launch_file.launch

# Debug specific node
rosrun --prefix 'gdb -ex run --args' package_name node_name

# Monitor node resource usage
top -p $(pgrep -f node_name)
```

### Network Debugging
```bash
# Monitor ROS network traffic
sudo tcpdump -i lo port 11311
wireshark &  # Filter: tcp.port == 11311

# Check ROS master xmlrpc interface
curl http://localhost:11311/
rosnode info /rosout

# Test service calls
rosservice call /service_name "{}"
```

### Memory and Performance Issues
```bash
# Check for memory leaks
valgrind --tool=memcheck rosrun package_name node_name

# Profile CPU usage
perf record rosrun package_name node_name
perf report

# Monitor bag file size during recording
watch -n 1 'du -sh *.bag'

# Limit message rate for debugging
rostopic pub -r 1 /topic_name std_msgs/String "data: 'test'"
```

## Recovery Procedures

### Complete ROS Reset
```bash
#!/bin/bash
# reset_ros.sh - Complete ROS environment reset

# Stop all ROS processes
killall -9 roscore rosmaster roslaunch

# Clean logs
rm -rf ~/.ros/log/*

# Reset environment
unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH

# Re-source ROS
source /opt/ros/noetic/setup.bash
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
fi

# Restart roscore
roscore &
sleep 2

echo "ROS environment reset complete"
```

### Workspace Rebuild
```bash
#!/bin/bash
# rebuild_workspace.sh - Clean workspace rebuild

cd ~/catkin_ws/

# Clean build artifacts
rm -rf build/ devel/

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
catkin_make

# Source new build
source devel/setup.bash

echo "Workspace rebuild complete"
```

## Useful Diagnostic Commands

```bash
# ROS environment check
env | grep ROS | sort

# Package verification
rospack profile && rospack plugins --attrib=plugin

# Node health check
for node in $(rosnode list); do
    echo "Checking $node:"
    timeout 5s rosnode ping $node
done

# Topic health check
for topic in $(rostopic list); do
    echo "Topic $topic: $(timeout 2s rostopic hz $topic 2>/dev/null | grep average || echo 'No data')"
done

# Service health check
for service in $(rosservice list); do
    echo "Service $service: $(rosservice type $service)"
done
```

## Related Documentation

- **Common Issues**: [Common Issues](common-issues.md) for general troubleshooting
- **Installation**: [ROS Installation](../installation/ros-noetic.md) for setup guidance
- **Basic Commands**: [Basic Commands](../usage/basic-commands.md) for fundamental operations
- **Verification**: [Installation Verification](../installation/verification.md) for testing