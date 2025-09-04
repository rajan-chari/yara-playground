# Gazebo-Specific Troubleshooting

This guide addresses issues specific to Gazebo Classic simulation and its integration with ROS.

## Installation and Setup Issues

### Gazebo Installation Problems
**Problem**: Gazebo installation fails or conflicts with ROS
```bash
E: Package 'gazebo11' has no installation candidate
dpkg: dependency problems prevent configuration of gazebo11
```

**Solutions**:
```bash
# Check Ubuntu version compatibility
lsb_release -a
# Gazebo 11 is compatible with Ubuntu 20.04

# Add Gazebo repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Add Gazebo key
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update package lists
sudo apt update

# Install Gazebo with ROS integration
sudo apt install gazebo11 libgazebo11-dev
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Verify installation
gazebo --version
which gzserver gzclient
```

### Gazebo-ROS Integration Issues
**Problem**: Gazebo doesn't integrate properly with ROS
```bash
[ERROR] Failed to load plugin libgazebo_ros_api_plugin.so
Resource not found: gazebo_ros
```

**Solutions**:
```bash
# Install missing ROS-Gazebo packages
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-gazebo-plugins ros-noetic-gazebo-msgs

# Check plugin path
echo $GAZEBO_PLUGIN_PATH
# Should include: /opt/ros/noetic/lib

# Add ROS plugins to Gazebo path
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH

# Add to bashrc
echo "export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc

# Source ROS before starting Gazebo
source /opt/ros/noetic/setup.bash
roslaunch gazebo_ros empty_world.launch
```

## Startup and Launch Issues

### Gazebo Won't Start
**Problem**: Gazebo fails to launch or crashes on startup
```bash
gzserver: /build/ogre-1.9-kiU5_5/ogre-1.9-1.9.0+dfsg1/OgreMain/src/OgreRenderSystem.cpp:546: virtual void Ogre::RenderSystem::setDepthBufferFor(Ogre::RenderTarget*): Assertion `bAttached' failed.
```

**Solutions**:
```bash
# Check graphics drivers
lspci | grep -i vga
glxinfo | grep "OpenGL version"

# For NVIDIA graphics issues
sudo apt install nvidia-driver-470  # or latest stable
sudo nvidia-settings

# For Intel graphics issues
sudo apt install mesa-utils
export LIBGL_ALWAYS_SOFTWARE=1  # Software rendering fallback

# For virtual machines/headless systems
export DISPLAY=:0
Xvfb :1 -screen 0 1024x768x24 &
export DISPLAY=:1

# Check Gazebo environment variables
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_PLUGIN_PATH

# Reset Gazebo configuration
rm -rf ~/.gazebo/
gazebo --version
```

### Graphics and Rendering Issues
**Problem**: Gazebo displays incorrectly or has rendering problems
```bash
QOpenGLShader::compile(Fragment): 0:1(10): error: GLSL 1.30 is not supported
[Err] [RenderEngine.cc:749] Can't open display: :0.0
```

**Solutions**:
```bash
# Check OpenGL support
glxinfo | grep -E "(version|vendor|renderer)"

# For headless systems, use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_IP=127.0.0.1

# Launch without GUI for headless operation
roslaunch gazebo_ros empty_world.launch gui:=false

# For remote display
export DISPLAY=:0.0
ssh -X user@remote_host
# Then run Gazebo on remote system

# Fix Qt OpenGL issues
export QT_X11_NO_MITSHM=1
export QT_OPENGL=desktop

# Alternative rendering backends
export OGRE_RTT_MODE=Copy
export GAZEBO_MASTER_URI=http://localhost:11345
```

### Memory and Performance Issues
**Problem**: Gazebo consumes excessive memory or runs slowly
```bash
System becomes unresponsive when running Gazebo
Gazebo simulation runs at very low real-time factor
```

**Solutions**:
```bash
# Monitor Gazebo memory usage
top -p $(pgrep gzserver)
htop

# Reduce physics update rate
rosrun gazebo_ros gazebo --pause -e ode
# In Gazebo GUI: World → Physics → Real Time Update Rate → 250 Hz

# Reduce rendering quality
# Edit ~/.gazebo/gui.ini
[rendering]
shadows=false
sky=false
grid=false

# Launch with reduced settings
roslaunch gazebo_ros empty_world.launch \
  physics:=ode \
  real_time_update_rate:=250 \
  max_step_size:=0.004 \
  gui:=false

# Limit CPU usage
nice -n 10 roslaunch gazebo_ros empty_world.launch
cpulimit -l 50 -e gzserver

# Clean model cache
rm -rf ~/.gazebo/models/*
gazebo --verbose
```

## Model and World Issues

### Model Loading Problems
**Problem**: Models fail to load or display incorrectly
```bash
[Err] [SystemPaths.cc:429] File or path does not exist[/path/to/model]
Error [parser.cc:581] Unable to find uri[model://model_name]
```

**Solutions**:
```bash
# Check model path
echo $GAZEBO_MODEL_PATH
ls ~/.gazebo/models/

# Download default models
cd ~/.gazebo/
wget -r -np -nH --cut-dirs=2 \
  http://models.gazebosim.org/

# Or use Gazebo model database
gazebo --verbose  # Will download models automatically

# Add custom model path
export GAZEBO_MODEL_PATH=/path/to/custom/models:$GAZEBO_MODEL_PATH

# Verify model structure
ls model_directory/
# Should contain: model.config model.sdf meshes/ materials/

# Check model.config format
<?xml version="1.0"?>
<model>
  <name>model_name</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Model description</description>
</model>

# Test model loading
gazebo --verbose model_name.sdf
```

### URDF/SDF Conversion Issues
**Problem**: Robot models don't display correctly when spawned
```bash
[ERROR] No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/joint_name
Warning [parser.cc:778] Converting a deprecated SDF source[/tmp/...].
```

**Solutions**:
```bash
# Check URDF syntax
check_urdf robot.urdf
rosrun urdf_parser check_urdf robot.urdf

# Convert URDF to SDF for inspection
gz sdf -p robot.urdf > robot.sdf

# Verify Gazebo-specific tags in URDF
grep -n "gazebo" robot.urdf

# Add necessary Gazebo plugins
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot</robotNamespace>
  </plugin>
</gazebo>

# Add joint properties for Gazebo
<gazebo reference="joint_name">
  <provideFeedback>true</provideFeedback>
</gazebo>

# Test spawning with verbose output
rosrun gazebo_ros spawn_model -file robot.urdf -urdf -model my_robot -verbose
```

### Physics Simulation Issues
**Problem**: Unrealistic physics behavior or simulation instability
```bash
Model falls through ground
Joints behave erratically
Simulation becomes unstable
```

**Solutions**:
```bash
# Check physics engine settings
# In Gazebo GUI: World → Physics

# Adjust time step
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>

# Improve collision detection
<collision>
  <geometry>
    <mesh>
      <uri>file://model.stl</uri>
    </mesh>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>

# Add contact parameters
<surface>
  <contact>
    <ode>
      <soft_cfm>0.01</soft_cfm>
      <soft_erp>0.2</soft_erp>
      <kp>1000</kp>
      <kd>100</kd>
    </ode>
  </contact>
</surface>

# Check model mass and inertia
# Ensure realistic values in URDF/SDF
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

## Plugin and Sensor Issues

### ROS Plugin Problems
**Problem**: Gazebo plugins don't work with ROS
```bash
[ERROR] Plugin libgazebo_ros_camera.so does not exist
[WARN] Controller Spawner couldn't find the expected controller_manager
```

**Solutions**:
```bash
# Check plugin installation
dpkg -l | grep gazebo-ros
ls /opt/ros/noetic/lib/ | grep gazebo

# Install missing plugins
sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-gazebo-plugins

# Verify plugin loading
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH
roslaunch gazebo_ros empty_world.launch verbose:=true

# Check plugin configuration in URDF
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>

# Test plugin functionality
rostopic list | grep camera
rostopic hz /camera/image_raw
```

### Sensor Data Issues
**Problem**: Sensors don't publish data or publish incorrect data
```bash
No data on sensor topics
Sensor data appears corrupted or unrealistic
```

**Solutions**:
```bash
# Check sensor topics
rostopic list | grep -E "(scan|image|imu|camera)"
rostopic info /topic_name

# Monitor sensor data
rostopic echo /scan
rostopic hz /camera/image_raw

# Check sensor configuration
<sensor type="ray" name="laser">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.1</resolution>
    </range>
  </ray>
</sensor>

# Verify TF frames for sensors
rosrun tf view_frames
evince frames.pdf

# Check sensor frame configuration
<frameName>laser_frame</frameName>
```

## Performance Optimization

### Simulation Speed Issues
**Problem**: Simulation runs slower than real-time
```bash
Real-time factor significantly less than 1.0
Simulation lags behind real-time
```

**Solutions**:
```bash
# Monitor performance
gz stats  # Shows real-time factor

# Reduce physics complexity
# Lower update rates
<real_time_update_rate>100</real_time_update_rate>
<max_step_size>0.01</max_step_size>

# Simplify collision geometry
# Use primitive shapes instead of meshes
<collision>
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
</collision>

# Disable unnecessary features
roslaunch gazebo_ros empty_world.launch \
  gui:=false \
  recording:=false \
  debug:=false

# Use ODE physics engine optimizations
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.0</sor>
    </solver>
  </ode>
</physics>
```

### Resource Management
**Problem**: Gazebo uses too many system resources
```bash
High CPU/GPU usage
Memory leaks during long simulations
```

**Solutions**:
```bash
# Limit Gazebo resources
# Set process priority
nice -n 10 gzserver

# Limit memory usage
ulimit -v 2000000  # Limit virtual memory to ~2GB

# Monitor resource usage
watch -n 1 'ps aux | grep gazebo'
watch -n 1 'nvidia-smi'  # For GPU monitoring

# Clean up periodically
pkill gzserver gzclient
rm -rf /tmp/gazebo_*

# Use headless mode when possible
export GAZEBO_IP=127.0.0.1
roslaunch gazebo_ros empty_world.launch gui:=false
```

## Networking and Multi-Machine Issues

### Multi-Machine Gazebo Setup
**Problem**: Gazebo doesn't work across multiple machines
```bash
Can't connect to Gazebo server on remote machine
Gazebo clients can't find server
```

**Solutions**:
```bash
# Set Gazebo networking
export GAZEBO_MASTER_URI=http://server_ip:11345
export GAZEBO_MASTER_IP=server_ip

# On server machine
gazebo --verbose --server-only

# On client machine
export GAZEBO_MASTER_URI=http://server_ip:11345
gzclient

# Check firewall settings
sudo ufw allow 11345  # Gazebo server port

# Test connectivity
telnet server_ip 11345
ping server_ip

# For ROS integration across machines
export ROS_MASTER_URI=http://ros_master_ip:11311
export ROS_IP=$(hostname -I | cut -d' ' -f1)
```

## Advanced Debugging

### Verbose Debugging
```bash
# Start Gazebo with maximum verbosity
gazebo --verbose --server --record_encoding=zlib

# Check Gazebo logs
ls ~/.gazebo/log/
tail -f ~/.gazebo/log/server_*/gzserver.log

# ROS-Gazebo debugging
roslaunch gazebo_ros empty_world.launch verbose:=true debug:=true

# Plugin debugging
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH
gdb --args gzserver worlds/empty.world
```

### Common Configuration Files

#### Gazebo configuration (~/.gazebo/gui.ini)
```ini
[geometry]
x=0
y=0

[rendering]
ambient_light=0.4 0.4 0.4 1.0
background_color=0.7 0.7 0.7 1.0
shadows=true
grid=true
sky=true

[physics]
gravity=0 0 -9.8
magnetic_field=6e-06 2.3e-05 -4.2e-05
```

#### Performance world file template
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="optimized_world">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Recovery Procedures

### Complete Gazebo Reset
```bash
#!/bin/bash
# reset_gazebo.sh - Complete Gazebo reset

# Stop all Gazebo processes
killall -9 gzserver gzclient gazebo

# Clean Gazebo files
rm -rf ~/.gazebo/log/*
rm -rf /tmp/gazebo_*

# Reset environment variables
unset GAZEBO_MASTER_URI GAZEBO_MASTER_IP
export GAZEBO_MASTER_URI=http://localhost:11345

# Clear model cache (optional)
# rm -rf ~/.gazebo/models/*

# Restart with clean environment
source /opt/ros/noetic/setup.bash
roslaunch gazebo_ros empty_world.launch

echo "Gazebo environment reset complete"
```

### Model Cache Refresh
```bash
#!/bin/bash
# refresh_models.sh - Refresh Gazebo models

# Backup existing models
mv ~/.gazebo/models ~/.gazebo/models.backup

# Download fresh models
mkdir -p ~/.gazebo/models
cd ~/.gazebo/models

# Download specific models
wget -r -np -nH --cut-dirs=2 -R "index.html*" \
  http://models.gazebosim.org/ground_plane/
wget -r -np -nH --cut-dirs=2 -R "index.html*" \
  http://models.gazebosim.org/sun/

echo "Model cache refreshed"
```

## Useful Diagnostic Commands

```bash
# Gazebo system check
gazebo --version
gz help

# Check Gazebo processes
ps aux | grep -E "(gazebo|gzserver|gzclient)"

# Monitor Gazebo performance
gz stats
gz topic -l

# Test basic functionality
timeout 10s gazebo --verbose

# Check ROS-Gazebo integration
rosservice list | grep gazebo
rostopic list | grep gazebo

# Network diagnostics
ss -tulpn | grep 11345
```

## Related Documentation

- **Common Issues**: [Common Issues](common-issues.md) for general troubleshooting
- **Installation**: [Gazebo Installation](../installation/gazebo.md) for setup guidance
- **Usage**: [Gazebo Simulation](../usage/gazebo-simulation.md) for basic operations
- **Verification**: [Installation Verification](../installation/verification.md) for testing