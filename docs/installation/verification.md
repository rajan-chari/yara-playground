# Installation Verification Guide

This guide provides comprehensive verification procedures for your Yara_OVE installation, ensuring ROS Noetic, Gazebo, and Miniconda are ready for experimentation.

## Quick Verification

### All Components Status Check
```bash
# Check all installations at once
echo "=== ROS Noetic ==="
rosversion -d 2>/dev/null && echo "✅ ROS Noetic installed" || echo "❌ ROS not found"

echo "=== Gazebo Classic ==="
gazebo --version 2>/dev/null | head -1 && echo "✅ Gazebo installed" || echo "❌ Gazebo not found"

echo "=== Miniconda ==="
conda --version 2>/dev/null && echo "✅ Miniconda installed" || echo "❌ Conda not found"
```

## Detailed ROS Verification

### Environment Check
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Check environment variables
echo "ROS_VERSION: $ROS_VERSION"          # Expected: 1
echo "ROS_DISTRO: $ROS_DISTRO"            # Expected: noetic
echo "ROS_ROOT: $ROS_ROOT"                # Expected: /opt/ros/noetic/share/ros
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH" # Expected: /opt/ros/noetic/share
```

### Core Functionality Test
```bash
# Test roscore
roscore &
ROSCORE_PID=$!
sleep 3

# Test topic communication
rostopic list
echo "std_msgs/String 'data: Hello ROS'" | rostopic pub /test_topic std_msgs/String

# Test service calls
rosservice list
rosservice call /rosout/get_loggers

# Cleanup
kill $ROSCORE_PID
```

### Package System Test
```bash
# Test package discovery
rospack list | wc -l  # Should show 200+ packages
rospack find geometry_msgs

# Test message introspection
rosmsg show geometry_msgs/Twist
rosmsg list | head -5

# Test service introspection
rossrv show rosgraph_msgs/Clock
```

### Build System Test
```bash
# Test catkin workspace
cd ~/catkin_ws/
catkin_make

# Verify workspace structure
ls devel/
source devel/setup.bash
echo $ROS_PACKAGE_PATH  # Should include ~/catkin_ws/src
```

## Detailed Gazebo Verification

### Standalone Test
```bash
# Test Gazebo standalone (headless)
timeout 10s gazebo --verbose -s || echo "Gazebo test completed"
```

### ROS Integration Test
```bash
# Test ROS-Gazebo integration
roslaunch gazebo_ros empty_world.launch gui:=false &
GAZEBO_PID=$!
sleep 5

# Check Gazebo topics
rostopic list | grep gazebo

# Check Gazebo services
rosservice list | grep gazebo

# Test model spawning capability
echo '<sdf version="1.4"><model name="test"><static>true</static><link name="link"><visual name="visual"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>' > /tmp/test_model.sdf
rosrun gazebo_ros spawn_model -model test_box -sdf -file /tmp/test_model.sdf -x 0 -y 0 -z 1

# Cleanup
pkill -f roslaunch
kill $GAZEBO_PID 2>/dev/null
rm /tmp/test_model.sdf
```

### Plugin System Test
```bash
# Check available plugins
ls /opt/ros/noetic/lib/libgazebo_ros_*.so | head -5

# Check Gazebo plugin paths
gazebo --help | grep plugin
```

## Detailed Miniconda Verification

### Base Installation Test
```bash
# Check conda installation
conda info

# Check base environment
conda env list
which conda
which python  # When base environment is active
```

### Environment Management Test
```bash
# Create test environment
conda create -n verification_test python=3.9 numpy -y

# Test activation
conda activate verification_test
python --version  # Should show Python 3.9.x
python -c "import numpy; print(f'NumPy {numpy.__version__} imported successfully')"

# Test package installation
conda install matplotlib -y
python -c "import matplotlib; print(f'Matplotlib {matplotlib.__version__} imported successfully')"

# Test pip integration
pip install requests
python -c "import requests; print(f'Requests {requests.__version__} imported successfully')"

# Test deactivation and cleanup
conda deactivate
conda env remove -n verification_test -y
```

### ROS-Conda Compatibility Test
```bash
# Test that ROS works with conda installed
source /opt/ros/noetic/setup.bash
rosversion -d

# Test conda environment with ROS
conda create -n ros_compat_test python=3.8 -y
conda activate ros_compat_test
source /opt/ros/noetic/setup.bash
rosversion -d  # Should still work
conda deactivate
conda env remove -n ros_compat_test -y
```

## Integration Testing

### Full Workflow Test
```bash
# Test complete ROS + Gazebo + Python workflow
# 1. Start ROS
roscore &
ROSCORE_PID=$!
sleep 2

# 2. Start Gazebo
roslaunch gazebo_ros empty_world.launch gui:=false &
GAZEBO_PID=$!
sleep 5

# 3. Activate conda environment for analysis
conda create -n integration_test python=3.9 numpy matplotlib -y
conda activate integration_test

# 4. Test data flow
rostopic echo /clock -n 5 &
sleep 6

# 5. Cleanup
conda deactivate
conda env remove -n integration_test -y
pkill -f roslaunch
kill $ROSCORE_PID $GAZEBO_PID 2>/dev/null
```

### Performance Verification
```bash
# Check system resources
echo "=== System Resources ==="
free -h
df -h /
ps aux | grep -E "(ros|gazebo|conda)" | wc -l

# Check installation sizes
echo "=== Installation Sizes ==="
du -sh /opt/ros/noetic/
du -sh ~/miniconda3/
du -sh /usr/share/gazebo-11/ 2>/dev/null || echo "Gazebo size: N/A"
```

## Expected Results Summary

### ✅ Successful Verification Should Show:

**ROS Noetic:**
- Version: noetic (stable LTS release)
- Packages: 200+ available including navigation and control
- Core functionality: roscore, topics, services working
- Build system: catkin workspace functional

**Gazebo Classic:**
- Version: 11.15.1 (supports advanced physics)
- ROS integration: Topics and services available
- Model management: Spawn/delete capabilities
- Plugin system: ROS plugins loaded

**Miniconda:**
- Version: 25.7.0 (supports scientific frameworks)
- Environment management: Create/activate/remove working
- Package management: conda and pip integration
- ROS compatibility: No conflicts detected

### ❌ Common Issues

If verification fails, check:
- Environment sourcing (`source ~/.bashrc`)
- Path conflicts between system and conda Python
- Missing dependencies or incomplete installation
- Network connectivity for package downloads

## Next Steps

After successful verification, you can proceed to:
- **Basic Commands**: Learn essential commands in [Basic Commands](../usage/basic-commands.md)
- **Advanced Workflows**: Explore [Advanced Workflows](../usage/advanced-workflows.md)
- **Troubleshooting**: Reference [Common Issues](../troubleshooting/common-issues.md) if needed
- **Learning Path**: Follow the [Learning Path](../resources/learning-path.md)

---

**🚀 Verification Complete - Continue to Next Steps**

*If all verifications pass, your installation is ready for robotics research and development.*

*Original Yara_OVE project: [https://github.com/medialab-fboat/Yara_OVE](https://github.com/medialab-fboat/Yara_OVE)*