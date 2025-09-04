# YARA-OVE Simulation Scenarios

This guide covers the various ocean simulation scenarios available in the YARA-OVE experimental playground, from basic wave physics to complete autonomous sailing demonstrations.

## 🌊 Overview

YARA-OVE provides three main simulation scenarios, each building upon the previous with increased complexity and functionality:

1. **Basic Ocean Waves** - Pure wave physics simulation
2. **Ocean with Navigation Buoys** - Wave physics + obstacle navigation
3. **Ocean with Sailing Boats** - Complete sailing robotics environment

## Scenario 1: Basic Ocean Waves

**Purpose**: Understanding wave physics, testing wave interactions, baseline ocean environment

### Launch Command
```bash
cd ~/yara_ws && source devel/setup.bash
roslaunch wave_gazebo ocean_world.launch
```

### What You'll See
- **Ocean Surface**: 1000m x 1000m wave field with Gerstner wave physics
- **Dynamic Waves**: Pierson-Moskowitz spectrum modeling with configurable parameters
- **Sky Environment**: Skybox with clouds and lighting
- **Physics**: No obstacles or boats - focus on wave behavior

### Key Features
- **Wave Physics**: Gerstner wave implementation with GPU acceleration
- **Wave Parameters**: Period=5s, Scale=1.5, Gain=0.1, Direction=[1.0, 0.0]
- **Performance**: 30 Hz update rate with visualization
- **Camera Position**: Positioned at [73.36, -0.35, 10.76] for wave viewing

### Use Cases
- **Wave Physics Analysis**: Study wave propagation and interaction
- **Visual Development**: Test wave rendering and GPU shader performance
- **Baseline Testing**: Foundation before adding complexity
- **Educational**: Understanding ocean wave mechanics and modeling

### Wave Monitoring
```bash
# Monitor wave field state
rostopic echo /gazebo/model_states | grep ocean_waves

# Check wave physics performance  
rostopic echo /gazebo/performance_metrics

# View wave field parameters
rosparam get /ocean_waves
```

## Scenario 2: Ocean with Navigation Buoys

**Purpose**: Navigation challenges, obstacle avoidance, path planning in dynamic environments

### Launch Command  
```bash
# Note: This scenario uses ocean_buoys.world.xacro
# Launch file needs to be created or accessed through yara-ove structure
cd ~/yara_ws && source devel/setup.bash
# roslaunch wave_gazebo ocean_buoys.launch  # (when available)
```

### What You'll See
- **Ocean Wave Base**: Same wave physics as Scenario 1
- **Navigation Buoys**: Multiple buoy types for obstacle navigation
- **Buoy Variety**: Different sizes and colors for navigation pattern recognition
- **Maritime Environment**: Offshore navigation scenario

### Available Buoy Types
Based on the ocean_buoys.world.xacro configuration:

#### Standard Navigation Buoys
- **polyform_a3**: Small navigation markers
- **polyform_a5**: Medium navigation markers  
- **polyform_a7**: Large navigation markers
- **surmark46104**: Specialized maritime markers
- **surmark950400**: Navigation aid markers
- **surmark950410**: Harbor entrance markers

### Buoy Positioning
```xml
<!-- Example buoy placement from ocean_buoys.world.xacro -->
<model name="robotx_2018_qualifying_avoid_obstacles_buoys">
  <include>
    <uri>model://robotx_2018_qualifying_avoid_obstacles_buoys</uri>
  </include>
</model>
```

### Use Cases
- **Autonomous Navigation**: Test path planning algorithms around obstacles
- **Computer Vision**: Buoy detection and classification training
- **Collision Avoidance**: Develop and test avoidance strategies
- **Competition Preparation**: Practice for sailing robotics competitions

### Navigation Monitoring
```bash
# List all buoy models in simulation
rostopic echo /gazebo/model_states | grep buoy

# Monitor buoy positions for navigation algorithms
rosservice call /gazebo/get_world_properties
```

## Scenario 3: Ocean with Sailing Boats

**Purpose**: Complete sailing robotics simulation, boat dynamics, autonomous sailing research

### Launch Command
```bash
# Note: This scenario uses ocean_wamv.world.xacro  
cd ~/yara_ws && source devel/setup.bash
# roslaunch wave_gazebo ocean_wamv.launch  # (when available)
```

### What You'll See
- **Maritime Scene**: Ocean waves + buoys + sailing boats
- **Multiple Boats**: Various boat models with different capabilities
- **Environment**: Sailing robotics testing environment
- **Wave Gauge**: Wave measurement tools for analysis

### Available Boat Models

#### EBoat (Digital Twin)
- **Length**: 2.5 meters
- **Type**: Research vessel digital twin based on UFF/UFRN F-Boat
- **Features**: Single sail, electric propeller, full sensor suite
- **Use Case**: Realistic autonomous sailing research

#### Fortune612 (RC Boat)
- **Length**: 0.99 meters  
- **Type**: RC sailing boat model
- **Features**: Main sail + jib sail, single cable control
- **Use Case**: Smaller scale testing and education

### Boat Specifications

#### EBoat Technical Details
```xml
<!-- From boto/model.sdf -->
<model name="eboat4">
  <link name="hull_link">
    <collision name="hull_collision">
      <geometry>
        <box><size>2.7 1.0 0.63</size></box>
      </geometry>
    </collision>
    <inertial>
      <mass>83.6871</mass>
      <inertia>
        <ixx>7.79423</ixx><iyy>42.8173</iyy><izz>47.1467</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

### Sailing Controls

#### Basic Boat Control
```bash
# List available boat control topics
rostopic list | grep boat

# Control boat movement (example)
rostopic pub /boat/cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Monitor boat state
rostopic echo /gazebo/model_states | grep boat
```

#### Sail Control
```bash
# Control sail position (when sail controllers available)
rostopic pub /sail/cmd std_msgs/Float64 "data: 0.5"

# Monitor wind conditions
rostopic echo /wind/direction
rostopic echo /wind/speed
```

### Use Cases
- **Autonomous Sailing**: Full autonomous navigation with sail control
- **Sailing Physics**: Study boat dynamics in wave conditions
- **Multi-Boat Coordination**: Fleet management and coordination
- **Research Validation**: Test algorithms in realistic conditions

### Advanced Monitoring
```bash
# Complete system monitoring
rostopic echo /gazebo/model_states

# Boat-specific monitoring
rostopic echo /boat/pose
rostopic echo /boat/velocity  
rostopic echo /sail/position

# Environmental monitoring
rostopic echo /wave_gauge/wave_height
rostopic echo /wind/conditions
```

## Scenario Progression Pathway

### 🔰 Beginner Path
1. **Start with Basic Ocean**: Understand wave physics fundamentals
2. **Add Buoys**: Learn navigation concepts
3. **Introduce Boats**: Begin sailing robotics

### 🧠 Researcher Path  
1. **Analyze Wave Physics**: Deep dive into Gerstner wave mathematics
2. **Develop Navigation Algorithms**: Test on buoy scenarios
3. **Implement Autonomous Sailing**: Full boat control research

### 🛠️ Developer Path
1. **Extend Wave Models**: Modify wave parameters and physics
2. **Create Custom Scenarios**: Design new buoy arrangements  
3. **Integrate New Boats**: Add custom boat models and controls

## Configuration and Customization

### Wave Parameter Customization
```xml
<!-- Modify wave_gazebo/world_models/ocean_waves/model.xacro -->
<xacro:macro name="ocean_waves" params="gain:=0.2 period:=7
                     direction_x:=0.8 direction_y:=0.6  
                     angle:=0.6 scale:=2.0">
```

### Adding Custom Buoys
```xml
<!-- Add to ocean_buoys.world.xacro -->
<model name="custom_buoy">
  <pose>10 5 0 0 0 0</pose>
  <include>
    <uri>model://your_custom_buoy</uri>
  </include>
</model>
```

### Boat Model Integration
```xml
<!-- Add new boat to ocean_wamv.world.xacro -->
<model name="new_sailing_boat">
  <pose>0 0 0 0 0 0</pose>
  <include>
    <uri>model://your_boat_model</uri>
  </include>
</model>
```

## Performance and System Requirements

### Scenario Complexity Comparison

| Scenario | Models | Physics Load | GPU Usage | Recommended RAM |
|----------|--------|-------------|-----------|-----------------|
| Basic Ocean | 1 | Low | Medium | 4GB |
| Ocean + Buoys | 5-15 | Medium | Medium | 6GB |
| Ocean + Boats | 10-25 | High | High | 8GB |

### Optimization Tips
1. **Reduce Wave Resolution**: Modify `cell_count` in wave configuration
2. **Limit Buoy Count**: Remove unnecessary buoys for performance
3. **Simplify Boat Models**: Use lower-detail boat models when possible
4. **GPU Settings**: Ensure proper GPU acceleration for wave shaders

## Troubleshooting

### Common Issues
```bash
# If ocean world fails to load
rospack find wave_gazebo

# Check wave plugin loading
grep -r "libWavefieldModelPlugin" /opt/ros/noetic/

# Verify boat model loading
rospack find yara_description
```

### Performance Issues
```bash
# Monitor system performance
htop | grep gazebo

# Check real-time factor
rostopic echo /gazebo/performance_metrics

# GPU utilization (if nvidia-smi available)
nvidia-smi
```

## Next Steps

- **Basic Usage**: See [Basic Commands](basic-commands.md) for fundamental operations
- **Advanced Experiments**: Check [Advanced Workflows](advanced-workflows.md) for research applications
- **Wave Physics Deep Dive**: Read [Additional Resources](../resources/additional-resources.md) for technical details
- **Learning Path**: Follow [Learning Path](../resources/learning-path.md) for structured progression

---

**🌊 Autonomous sailing scenarios**

*Choose your scenario based on your experience level and research goals. Each scenario builds upon the previous to create an autonomous sailing robotics experience.*

*Built upon the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE) research framework.*