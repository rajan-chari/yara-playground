# Gazebo Simulation Guide

This guide covers practical usage of Gazebo Classic for robotic simulation, supporting the Yara_OVE experimental playground.

## Getting Started with Simulation

### Basic Environment Launch
```bash
# Launch Gazebo with empty world
roslaunch gazebo_ros empty_world.launch

# Launch with specific world
roslaunch gazebo_ros empty_world.launch world_name:=world_name.world

# Launch headless mode (no GUI)
roslaunch gazebo_ros empty_world.launch gui:=false

# Launch paused for robot setup
roslaunch gazebo_ros empty_world.launch paused:=true
```

### Common Launch Parameters
```bash
# Full parameter example
roslaunch gazebo_ros empty_world.launch \
  world_name:=willowgarage.world \
  paused:=false \
  use_sim_time:=true \
  gui:=true \
  recording:=false \
  debug:=false \
  verbose:=true
```

## Working with Robot Models

### Spawning Models

#### From URDF File
```bash
# Spawn robot from URDF file
rosrun gazebo_ros spawn_model \
  -model robot_name \
  -file robot.urdf \
  -urdf \
  -x 0 -y 0 -z 0.1
```

#### From SDF File
```bash
# Spawn model from SDF file
rosrun gazebo_ros spawn_model \
  -model model_name \
  -file model.sdf \
  -sdf \
  -x 0 -y 0 -z 0.1
```

#### From ROS Parameter
```bash
# Load URDF to parameter server
rosparam load robot.urdf robot_description

# Spawn robot from parameter
rosrun gazebo_ros spawn_model \
  -model robot_name \
  -param robot_description \
  -urdf \
  -x 0 -y 0 -z 0.1
```

#### With Initial Position and Orientation
```bash
# Spawn model with specific pose
rosrun gazebo_ros spawn_model \
  -model robot_name \
  -file robot.urdf \
  -urdf \
  -x 2.0 -y 1.0 -z 0.1 \
  -R 0.0 -P 0.0 -Y 1.57
```

### Managing Models
```bash
# List all models in simulation
rostopic echo /gazebo/model_states -n 1

# Remove model from simulation
rosservice call /gazebo/delete_model "model_name: 'robot_name'"

# Get model state
rosservice call /gazebo/get_model_state "model_name: 'robot_name'"

# Set model position and orientation
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'robot_name'
  pose:
    position: {x: 1.0, y: 2.0, z: 0.1}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## Simulation Control

### Physics Control
```bash
# Pause simulation
rosservice call /gazebo/pause_physics

# Unpause simulation
rosservice call /gazebo/unpause_physics

# Reset simulation (models return to initial state)
rosservice call /gazebo/reset_simulation

# Reset world (removes all spawned models)
rosservice call /gazebo/reset_world
```

### Time Management
```bash
# When use_sim_time:=true, Gazebo publishes simulation time
rostopic echo /clock

# Check if simulation time is being used
rosparam get /use_sim_time
```

### Physics Properties
```bash
# Get current physics properties
rosservice call /gazebo/get_physics_properties

# Set physics properties (example: reduce gravity)
rosservice call /gazebo/set_physics_properties "
time_step: 0.001
max_update_rate: 1000.0
gravity: {x: 0.0, y: 0.0, z: -5.0}
ode_config:
  auto_disable_bodies: false
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"
```

## Working with Worlds

### Available Worlds
```bash
# List available worlds
ls /usr/share/gazebo-11/worlds/

# Common worlds:
# - empty.world (default)
# - willowgarage.world
# - pioneer2dx.world
# - robocup_3Dsim.world
```

### Creating Custom Launch Files
```xml
<!-- my_simulation.launch -->
<launch>
  <!-- Launch Gazebo with custom world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find my_package)/worlds/my_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
  </include>

  <!-- Spawn robot -->
  <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model"
        args="-file $(find my_package)/urdf/robot.urdf
              -urdf -model my_robot
              -x 0 -y 0 -z 0.1"
        output="screen"/>
</launch>
```

## Sensors and Plugins

### Common Sensor Topics
```bash
# Camera sensors
rostopic list | grep camera
rostopic echo /camera/image_raw

# Lidar sensors
rostopic list | grep scan
rostopic echo /scan

# IMU sensors
rostopic list | grep imu
rostopic echo /imu/data

# GPS sensors
rostopic echo /fix
```

### Plugin Configuration Examples

#### Camera Plugin (in URDF/SDF)
```xml
<sensor type="camera" name="camera1">
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <camera_name>my_camera</camera_name>
    <frame_name>camera_link</frame_name>
    <image_topic_name>image_raw</image_topic_name>
    <camera_info_topic_name>camera_info</camera_info_topic_name>
  </plugin>
</sensor>
```

#### Lidar Plugin (in URDF/SDF)
```xml
<sensor type="ray" name="hokuyo">
  <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
    <topicName>/scan</topicName>
    <frameName>hokuyo_link</frameName>
  </plugin>
</sensor>
```

## Robot Control

### Differential Drive Example
```bash
# If robot has differential drive plugin, control with:
rostopic pub /cmd_vel geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

### Joint Control
```bash
# List available joint controllers
rostopic list | grep joint

# Control individual joints
rostopic pub /joint_position_controller/command std_msgs/Float64 "data: 1.57"

# Get joint states
rostopic echo /joint_states
```

## Monitoring and Debugging

### Gazebo Topics and Services
```bash
# List all Gazebo-related topics
rostopic list | grep gazebo

# Key topics:
rostopic echo /gazebo/link_states    # All link positions/velocities
rostopic echo /gazebo/model_states   # All model positions/velocities
rostopic echo /gazebo/parameter_updates  # Parameter changes

# List all Gazebo services
rosservice list | grep gazebo
```

### Performance Monitoring
```bash
# Check real-time factor
rostopic echo /gazebo/performance_metrics

# Monitor resource usage
htop
# Look for gzserver and gzclient processes
```

### Gazebo GUI Tools
- **World Properties**: Edit physics parameters
- **Model Editor**: Create/modify models
- **Joint Control**: Manual joint manipulation
- **Topic Visualization**: View sensor data
- **Statistics**: Performance metrics

## Example Workflows

### Basic Robot Simulation
```bash
# 1. Start Gazebo
roslaunch gazebo_ros empty_world.launch &

# 2. Spawn a simple robot
echo '<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>' > /tmp/simple_robot.urdf

rosrun gazebo_ros spawn_model -model simple_robot -file /tmp/simple_robot.urdf -urdf

# 3. Interact with the robot
# Use Gazebo GUI to apply forces or move the robot
```

### Sensor Data Collection
```bash
# 1. Launch simulation with sensor-equipped robot
roslaunch my_package robot_simulation.launch

# 2. Record sensor data
rosbag record /camera/image_raw /scan /imu/data -O sensor_data.bag

# 3. Analyze data later
rosbag play sensor_data.bag
```

### Multi-Robot Simulation
```bash
# Spawn multiple robots with different namespaces
rosrun gazebo_ros spawn_model -model robot1 -file robot.urdf -urdf -x 0 -y 0 -z 0.1
rosrun gazebo_ros spawn_model -model robot2 -file robot.urdf -urdf -x 2 -y 0 -z 0.1
rosrun gazebo_ros spawn_model -model robot3 -file robot.urdf -urdf -x 4 -y 0 -z 0.1
```

## Best Practices

### Performance Optimization
1. **Use headless mode** for automated testing: `gui:=false`
2. **Reduce physics accuracy** if real-time performance is needed
3. **Limit sensor update rates** to necessary frequencies
4. **Use simplified collision geometries** when possible

### Model Design
1. **Keep models lightweight** - avoid unnecessary detail
2. **Use appropriate inertial properties** for realistic physics
3. **Add collision geometries** for all interactive elements
4. **Test models incrementally** - start simple, add complexity

### Debugging Tips
1. **Check console output** for error messages
2. **Use rostopic echo** to verify data flow
3. **Monitor system resources** during simulation
4. **Save and version control** your world and launch files

## Next Steps

- **Advanced Integration**: See [Advanced Workflows](advanced-workflows.md) for complex scenarios
- **AI Development**: Try [Python Environments](python-environments.md) for machine learning
- **Troubleshooting**: Check [Gazebo-Specific Issues](../troubleshooting/gazebo-specific.md)
- **Learning Resources**: Follow [Learning Path](../resources/learning-path.md) for skill development

---

**🚀 Master Gazebo Simulation!**

*Use these techniques to fully leverage the Yara_OVE experimental playground's simulation capabilities.*

*Building upon the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE) research.*