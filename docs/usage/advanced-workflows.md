# Advanced Workflows

This guide covers integration patterns combining ROS, Gazebo simulation, and Python environments for complex robotics projects in the Yara_OVE experimental playground.

## 🌊 YARA-OVE Sailing Robotics Workflows

These workflows use the YARA-OVE wave physics system for autonomous sailing robotics research and development.

### Autonomous Sailing Research Pipeline
```bash
# 1. Setup YARA-OVE sailing environment
cd ~/yara_ws && source devel/setup.bash
conda activate sailing_research

# 2. Launch ocean simulation with wave physics
roslaunch wave_gazebo ocean_world.launch &
sleep 10

# 3. Add sailing boat for experiments
# rosrun gazebo_ros spawn_model -model eboat -file $(rospack find yara_description)/models/boto/model.sdf -sdf

# 4. Start data collection for sailing analysis
rosbag record /gazebo/model_states /cmd_vel /imu/data /wind -O sailing_experiment.bag &

# 5. Run autonomous sailing controller
python autonomous_sailing_controller.py

# 6. Analyze sailing performance with wave interactions
python sailing_performance_analyzer.py sailing_experiment.bag
```

### Roadmap for Sailing Experiments

#### Phase 1: Ocean Physics
- **Wave Physics Analysis**: Gerstner wave implementation with PMS modeling
- **Ocean World Launch**: Running [`roslaunch wave_gazebo ocean_world.launch`](../../yara-ove/wave_gazebo/launch/ocean_world.launch)
- **GPU-Accelerated Rendering**: Wave visualization with vertex shaders
- **Scenario Documentation**: [YARA-OVE Scenarios Guide](yara-ove-scenarios.md)

#### Phase 2: Sailing Boat Integration
```bash
# EBoat (Research Vessel) Integration
# 1. Spawn EBoat digital twin
rosrun gazebo_ros spawn_model \
  -model eboat_research \
  -file $(rospack find yara_description)/models/boto/model.sdf \
  -sdf -x 0 -y 0 -z 1.0

# 2. Setup sail control interfaces
rostopic pub /eboat/sail_cmd std_msgs/Float64 "data: 0.5"
rostopic pub /eboat/rudder_cmd std_msgs/Float64 "data: 0.0"

# 3. Monitor boat dynamics in waves
rostopic echo /gazebo/model_states | grep eboat
```

```bash
# Fortune612 (RC Boat) Integration
# 1. Spawn Fortune612 model
rosrun gazebo_ros spawn_model \
  -model fortune612 \
  -file $(rospack find yara_description)/models/fortune612/model.sdf \
  -sdf -x 5 -y 0 -z 1.0

# 2. Control main sail and jib
rostopic pub /fortune612/main_sail_cmd std_msgs/Float64 "data: 0.3"
rostopic pub /fortune612/jib_sail_cmd std_msgs/Float64 "data: 0.2"
```

#### Phase 3: Sailing Physics Research
```python
#!/usr/bin/env python3
# sailing_physics_analyzer.py - Advanced sailing dynamics analysis
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SailingPhysicsAnalyzer:
    def __init__(self):
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        self.wind_pub = rospy.Publisher('/wind/cmd', Twist, queue_size=1)
        
        # Sailing physics parameters
        self.boat_name = "eboat_research"
        self.current_pose = None
        self.wave_height_history = []
        
    def model_callback(self, msg):
        """Analyze boat motion in waves"""
        if self.boat_name in msg.name:
            idx = msg.name.index(self.boat_name)
            self.current_pose = msg.pose[idx]
            
            # Wave interaction analysis
            wave_height = self.current_pose.position.z
            self.wave_height_history.append(wave_height)
            
            if len(self.wave_height_history) > 100:
                self.analyze_wave_response()
                
    def analyze_wave_response(self):
        """Analyze boat response to wave motion"""
        heights = np.array(self.wave_height_history[-100:])
        
        # Calculate wave response metrics
        wave_amplitude = np.std(heights)
        wave_frequency = self.estimate_wave_frequency(heights)
        
        rospy.loginfo(f"Wave Response - Amplitude: {wave_amplitude:.3f}m, Frequency: {wave_frequency:.3f}Hz")
        
        # Clear history for next analysis
        self.wave_height_history = self.wave_height_history[-20:]
        
    def estimate_wave_frequency(self, heights):
        """Estimate dominant wave frequency from boat motion"""
        fft = np.fft.fft(heights)
        freqs = np.fft.fftfreq(len(heights), d=0.1)  # 10Hz sampling
        dominant_freq = freqs[np.argmax(np.abs(fft[1:len(fft)//2])) + 1]
        return abs(dominant_freq)

if __name__ == '__main__':
    rospy.init_node('sailing_physics_analyzer')
    analyzer = SailingPhysicsAnalyzer()
    rospy.spin()
```

#### Phase 4: Autonomous Sailing Control
```python
#!/usr/bin/env python3
# autonomous_sailing_controller.py - AI-driven sailing control
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class AutonomousSailingController:
    def __init__(self):
        # Publishers for boat control
        self.sail_pub = rospy.Publisher('/eboat/sail_cmd', Float64, queue_size=1)
        self.rudder_pub = rospy.Publisher('/eboat/rudder_cmd', Float64, queue_size=1)
        
        # Subscribers for state estimation
        rospy.Subscriber('/eboat/imu', Imu, self.imu_callback)
        rospy.Subscriber('/eboat/odom', Odometry, self.odom_callback)
        
        # Sailing controller parameters
        self.wind_direction = 0.0  # Relative wind direction
        self.wind_speed = 5.0      # Wind speed (m/s)
        self.target_heading = 0.0  # Target compass heading
        
        # Control loop timer
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
    def control_loop(self, event):
        """Main autonomous sailing control loop"""
        # Calculate optimal sail angle
        sail_angle = self.calculate_sail_angle(self.wind_direction)
        
        # Calculate rudder angle for heading control
        rudder_angle = self.calculate_rudder_angle(self.target_heading)
        
        # Publish control commands
        self.sail_pub.publish(Float64(data=sail_angle))
        self.rudder_pub.publish(Float64(data=rudder_angle))
        
    def calculate_sail_angle(self, wind_direction):
        """Calculate optimal sail angle based on wind direction"""
        # Simplified sail control logic
        apparent_wind = wind_direction  # Simplified - should include boat speed
        
        if abs(apparent_wind) < 0.5:  # Head-to-wind (no-go zone)
            return 0.0
        elif apparent_wind > 0:  # Wind from starboard
            return min(0.8, apparent_wind * 0.7)
        else:  # Wind from port
            return max(-0.8, apparent_wind * 0.7)
            
    def calculate_rudder_angle(self, target_heading):
        """Calculate rudder angle for heading control"""
        # Simplified heading control
        heading_error = target_heading - self.current_heading
        
        # Normalize angle to [-pi, pi]
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi
            
        # Proportional control
        return np.clip(heading_error * 2.0, -0.5, 0.5)
        
    def imu_callback(self, msg):
        """Process IMU data for orientation"""
        # Extract heading from quaternion
        # Simplified - should use proper quaternion to euler conversion
        pass
        
    def odom_callback(self, msg):
        """Process odometry for position and velocity"""
        self.current_heading = msg.pose.pose.orientation.z  # Simplified

if __name__ == '__main__':
    rospy.init_node('autonomous_sailing_controller')
    controller = AutonomousSailingController()
    rospy.spin()
```

### Multi-Boat Coordination Experiments
```python
#!/usr/bin/env python3
# fleet_coordination.py - Multi-boat sailing coordination
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np

class SailingFleetController:
    def __init__(self, boat_names):
        self.boat_names = boat_names
        self.boat_positions = {}
        self.formation_target = "line_formation"  # or "v_formation", "circle_formation"
        
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.fleet_callback)
        
        # Publishers for each boat
        self.boat_controllers = {}
        for boat in boat_names:
            self.boat_controllers[boat] = {
                'sail': rospy.Publisher(f'/{boat}/sail_cmd', Float64, queue_size=1),
                'rudder': rospy.Publisher(f'/{boat}/rudder_cmd', Float64, queue_size=1)
            }
    
    def fleet_callback(self, msg):
        """Coordinate multiple boats in formation"""
        # Extract positions of all fleet boats
        for boat in self.boat_names:
            if boat in msg.name:
                idx = msg.name.index(boat)
                self.boat_positions[boat] = msg.pose[idx].position
        
        if len(self.boat_positions) == len(self.boat_names):
            self.maintain_formation()
    
    def maintain_formation(self):
        """Implement formation control logic"""
        if self.formation_target == "line_formation":
            self.line_formation_control()
        elif self.formation_target == "v_formation":
            self.v_formation_control()
            
    def line_formation_control(self):
        """Maintain boats in line formation"""
        # Calculate formation center
        positions = list(self.boat_positions.values())
        center_x = np.mean([p.x for p in positions])
        center_y = np.mean([p.y for p in positions])
        
        # Control each boat to maintain formation
        for i, boat in enumerate(self.boat_names):
            target_x = center_x
            target_y = center_y + (i - len(self.boat_names)//2) * 5.0  # 5m spacing
            
            # Simple formation control
            pos = self.boat_positions[boat]
            error_x = target_x - pos.x
            error_y = target_y - pos.y
            
            # Convert to sail and rudder commands
            sail_cmd = 0.5  # Constant sail setting
            rudder_cmd = np.arctan2(error_y, error_x) * 0.5
            
            self.boat_controllers[boat]['sail'].publish(Float64(data=sail_cmd))
            self.boat_controllers[boat]['rudder'].publish(Float64(data=rudder_cmd))

# Usage
if __name__ == '__main__':
    rospy.init_node('sailing_fleet_controller')
    fleet = SailingFleetController(['eboat_1', 'eboat_2', 'fortune612_1'])
    rospy.spin()
```

### Research Applications and Future Directions

#### 1. Ocean Monitoring Missions
- **Environmental Data Collection**: Deploy autonomous sailing boats for ocean temperature, salinity measurements
- **Wave Spectrum Analysis**: Use boat motion to validate Gerstner wave models
- **Long-Duration Missions**: Test endurance and autonomous decision-making

#### 2. Sailing Robotics Competitions
- **RobotX Competition Training**: Practice obstacle avoidance with buoys
- **World Robotic Sailing Championship**: Test autonomous navigation strategies
- **Custom Challenges**: Design sailing tasks using YARA-OVE scenarios

#### 3. Advanced Research Topics
- **Machine Learning for Sailing**: Reinforcement learning with ESailor integration
- **Swarm Robotics**: Multi-boat coordination and communication
- **Weather Routing**: Optimal path planning considering wind and wave conditions
- **Energy Optimization**: Maximize sailing efficiency, minimize motor usage

### Integration with ESailor RL Framework
```python
# rl_sailing_environment.py - Reinforcement learning integration
import gym
from gym import spaces
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates

class YaraOVESailingEnv(gym.Env):
    """Custom Environment for YARA-OVE sailing RL experiments"""
    
    def __init__(self):
        super(YaraOVESailingEnv, self).__init__()
        
        # Action space: [sail_angle, rudder_angle]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -0.5]),
            high=np.array([1.0, 0.5]),
            dtype=np.float32
        )
        
        # Observation space: [boat_pos_x, boat_pos_y, boat_heading, wind_direction, wave_height]
        self.observation_space = spaces.Box(
            low=np.array([-100, -100, -np.pi, -np.pi, -2.0]),
            high=np.array([100, 100, np.pi, np.pi, 2.0]),
            dtype=np.float32
        )
        
        # ROS interface
        rospy.init_node('rl_sailing_env')
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        
    def step(self, action):
        """Execute action and return observation, reward, done, info"""
        # Apply action to boat
        self.apply_sailing_action(action)
        
        # Get new observation
        obs = self.get_observation()
        
        # Calculate reward
        reward = self.calculate_reward(obs, action)
        
        # Check if episode is done
        done = self.is_episode_done(obs)
        
        return obs, reward, done, {}
        
    def calculate_reward(self, obs, action):
        """Calculate reward based on sailing performance"""
        # Example reward function
        # Reward for moving towards target
        target_distance = np.sqrt(obs[0]**2 + obs[1]**2)
        distance_reward = -target_distance * 0.01
        
        # Penalty for extreme actions
        action_penalty = -0.1 * (np.abs(action[0]) + np.abs(action[1]))
        
        # Reward for efficient sailing (using wind)
        wind_efficiency = np.cos(obs[3] - obs[2])  # Sailing angle efficiency
        wind_reward = wind_efficiency * 0.1
        
        return distance_reward + action_penalty + wind_reward

# Usage with stable-baselines3
from stable_baselines3 import PPO

env = YaraOVESailingEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)
```

## Integrated Development Workflows

### Complete Robotics Pipeline
```bash
# 1. Setup integrated environment
conda create -n robotics_pipeline python=3.8 numpy opencv matplotlib
conda activate robotics_pipeline
pip install rospkg rospy_message_converter
source /opt/ros/noetic/setup.bash

# 2. Start simulation environment
roslaunch gazebo_ros empty_world.launch world_name:=custom_world.world &
sleep 5

# 3. Deploy robot with sensors
rosrun gazebo_ros spawn_model -model robot -file robot_sensors.urdf -urdf

# 4. Start data collection
rosbag record /camera/image_raw /scan /odom -O experiment.bag &

# 5. Run experiment
python mission_controller.py

# 6. Analyze results
python -c "
import rosbag_pandas as rbp
import matplotlib.pyplot as plt
df = rbp.bag_to_dataframe('experiment.bag')
# Analysis code here
"
```

### Multi-Environment Development
```bash
# Development environment
conda create -n dev python=3.9 pytest black flake8
conda activate dev

# Production environment
conda create -n prod python=3.8 --file requirements.txt
conda activate prod

# Testing environment
conda create -n test python=3.8 pytest-cov hypothesis
conda activate test

# Switch between environments for different phases
conda activate dev    # Algorithm development and debugging
conda activate test   # Running simulation tests
conda activate prod   # Mission deployment
```

## Advanced ROS Integration

### Custom Message Development
```bash
# 1. Create package with custom messages
cd ~/catkin_ws/src/
catkin_create_pkg my_custom_msgs std_msgs rospy roscpp

# 2. Define custom message
mkdir -p my_custom_msgs/msg
cat << 'EOF' > my_custom_msgs/msg/RobotState.msg
# Robot state message
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
sensor_msgs/JointState joints
float64 battery_level
string status
EOF

# 3. Update package.xml and CMakeLists.txt for message generation
# 4. Build messages
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# 5. Use in Python with conda environment
conda activate robotics_pipeline
source ~/catkin_ws/devel/setup.bash
python -c "from my_custom_msgs.msg import RobotState; print('Custom message imported')"
```

### Dynamic Reconfigure Integration
```python
#!/usr/bin/env python3
# advanced_controller.py - Dynamic parameter tuning
import rospy
from dynamic_reconfigure.server import Server
from my_package.cfg import ControllerConfig
import numpy as np

class AdvancedController:
    def __init__(self):
        self.srv = Server(ControllerConfig, self.reconfigure_callback)
        
        # Initialize with conda environment packages
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.01
        
    def reconfigure_callback(self, config, level):
        self.kp = config.kp
        self.ki = config.ki  
        self.kd = config.kd
        rospy.loginfo(f"Updated PID gains: P={self.kp}, I={self.ki}, D={self.kd}")
        return config
        
    def control_loop(self):
        # Advanced control algorithm using numpy/scipy
        error_integral = 0
        last_error = 0
        
        while not rospy.is_shutdown():
            # Get current state
            current_error = self.get_error()
            
            # PID control with numerical integration
            error_integral += current_error * 0.01  # dt = 0.01s
            error_derivative = (current_error - last_error) / 0.01
            
            control_output = (self.kp * current_error + 
                            self.ki * error_integral + 
                            self.kd * error_derivative)
            
            self.send_control_command(control_output)
            last_error = current_error
            
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('advanced_controller')
    controller = AdvancedController()
    controller.control_loop()
```

### Service-Based Architecture
```python
#!/usr/bin/env python3
# service_based_system.py - Microservices architecture with ROS
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist
import numpy as np
import pickle

class RoboticsService:
    def __init__(self, service_name):
        self.service_name = service_name
        rospy.Service(f'/{service_name}/enable', SetBool, self.enable_callback)
        rospy.Service(f'/{service_name}/reset', SetBool, self.reset_callback)
        
    def enable_callback(self, req):
        rospy.loginfo(f"{self.service_name}: Enable request - {req.data}")
        return SetBoolResponse(success=True, message=f"{self.service_name} enabled")
    
    def reset_callback(self, req):
        rospy.loginfo(f"{self.service_name}: Reset request")
        return SetBoolResponse(success=True, message=f"{self.service_name} reset")

class NavigationService(RoboticsService):
    def __init__(self):
        super().__init__("navigation")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Load ML model using conda environment packages
        self.model = self.load_navigation_model()
        
    def load_navigation_model(self):
        # Example: Load pre-trained model
        try:
            with open('navigation_model.pkl', 'rb') as f:
                return pickle.load(f)
        except FileNotFoundError:
            rospy.logwarn("Navigation model not found, using default behavior")
            return None
    
    def navigate_to_goal(self, goal_x, goal_y):
        if self.model:
            # Use ML model for navigation
            features = np.array([goal_x, goal_y]).reshape(1, -1)
            cmd_values = self.model.predict(features)[0]
        else:
            # Simple proportional controller
            cmd_values = [goal_x * 0.5, goal_y * 0.5]
        
        cmd = Twist()
        cmd.linear.x = cmd_values[0]
        cmd.angular.z = cmd_values[1]
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('navigation_service')
    nav_service = NavigationService()
    rospy.spin()
```

## Advanced Gazebo Integration

### Programmatic World Generation
```python
#!/usr/bin/env python3
# world_generator.py - Procedural world generation
import xml.etree.ElementTree as ET
import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel

class WorldGenerator:
    def __init__(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        
        self.spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
    def generate_random_obstacles(self, num_obstacles=10):
        """Generate random obstacles in simulation"""
        for i in range(num_obstacles):
            # Random position and size
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, 5)
            z = 0.5
            size = np.random.uniform(0.2, 1.0)
            
            # Generate SDF model
            sdf = self.create_box_sdf(f"obstacle_{i}", size, size, 1.0)
            
            # Spawn in Gazebo
            try:
                self.spawn_service(
                    model_name=f"obstacle_{i}",
                    model_xml=sdf,
                    robot_namespace="",
                    initial_pose=self.create_pose(x, y, z),
                    reference_frame="world"
                )
                rospy.loginfo(f"Spawned obstacle_{i} at ({x:.2f}, {y:.2f})")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn obstacle_{i}: {e}")
    
    def create_box_sdf(self, name, width, height, depth):
        """Create SDF string for a box"""
        return f"""<?xml version='1.0'?>
        <sdf version='1.4'>
          <model name='{name}'>
            <static>true</static>
            <link name='link'>
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>{width} {height} {depth}</size>
                  </box>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <box>
                    <size>{width} {height} {depth}</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Red</name>
                  </script>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
    
    def create_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Create geometry_msgs/Pose"""
        from geometry_msgs.msg import Pose, Point, Quaternion
        import tf.transformations
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(*quaternion)
        
        return pose
    
    def clear_world(self):
        """Remove all generated obstacles"""
        i = 0
        while True:
            try:
                self.delete_service(f"obstacle_{i}")
                rospy.loginfo(f"Deleted obstacle_{i}")
                i += 1
            except rospy.ServiceException:
                break

if __name__ == '__main__':
    rospy.init_node('world_generator')
    generator = WorldGenerator()
    
    # Generate random world
    generator.generate_random_obstacles(15)
    
    rospy.loginfo("World generation complete. Press Ctrl+C to clear and exit.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        generator.clear_world()
```

### Advanced Sensor Simulation
```python
#!/usr/bin/env python3
# sensor_simulator.py - Realistic sensor simulation with noise
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs.point_cloud2 as pc2

class AdvancedSensorSimulator:
    def __init__(self):
        # Publishers for synthetic sensor data
        self.lidar_pub = rospy.Publisher('/synthetic_scan', LaserScan, queue_size=1)
        self.cloud_pub = rospy.Publisher('/synthetic_cloud', PointCloud2, queue_size=1)
        
        # Parameters for realistic simulation
        self.lidar_range = 10.0
        self.lidar_noise_std = 0.02
        self.dropout_rate = 0.01
        
    def simulate_lidar_with_noise(self, true_ranges):
        """Add realistic noise to lidar data"""
        # Add Gaussian noise
        noisy_ranges = true_ranges + np.random.normal(0, self.lidar_noise_std, len(true_ranges))
        
        # Add dropouts (missed detections)
        dropout_mask = np.random.random(len(true_ranges)) < self.dropout_rate
        noisy_ranges[dropout_mask] = float('inf')
        
        # Add max range limitation
        noisy_ranges = np.clip(noisy_ranges, 0.1, self.lidar_range)
        
        return noisy_ranges
    
    def generate_synthetic_environment(self):
        """Generate synthetic sensor data for testing"""
        angles = np.linspace(-np.pi, np.pi, 360)
        
        # Create synthetic environment with walls and obstacles
        ranges = np.full(360, self.lidar_range)
        
        # Add walls at boundaries
        ranges[0:45] = 3.0    # Wall on one side
        ranges[315:360] = 3.0
        
        # Add circular obstacles
        for angle_deg in [90, 180, 270]:
            idx = int((angle_deg + 180) * 360 / 360)
            ranges[idx-5:idx+5] = 1.5
        
        # Add noise
        noisy_ranges = self.simulate_lidar_with_noise(ranges)
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser"
        scan.angle_min = -np.pi
        scan.angle_max = np.pi
        scan.angle_increment = 2 * np.pi / 360
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = self.lidar_range
        scan.ranges = noisy_ranges.tolist()
        
        return scan
    
    def run_simulation(self):
        """Main simulation loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Generate and publish synthetic sensor data
            synthetic_scan = self.generate_synthetic_environment()
            self.lidar_pub.publish(synthetic_scan)
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_simulator')
    simulator = AdvancedSensorSimulator()
    simulator.run_simulation()
```

## Machine Learning Integration

### Real-Time Learning Pipeline
```python
#!/usr/bin/env python3
# ml_pipeline.py - Online learning for robotics
import rospy
import numpy as np
import pickle
from sklearn.linear_model import SGDRegressor
from sklearn.preprocessing import StandardScaler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from collections import deque

class OnlineLearningPipeline:
    def __init__(self):
        # Initialize online learning model
        self.model = SGDRegressor(learning_rate='adaptive', eta0=0.01)
        self.scaler = StandardScaler()
        self.is_fitted = False
        
        # Data buffers
        self.feature_buffer = deque(maxlen=1000)
        self.target_buffer = deque(maxlen=1000)
        
        # ROS interfaces
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        
        self.current_scan = None
        self.current_odom = None
        self.last_cmd = None
        
        # Model updates
        self.update_timer = rospy.Timer(rospy.Duration(1.0), self.update_model)
        
    def scan_callback(self, msg):
        self.current_scan = np.array(msg.ranges)
        # Process scan data
        self.current_scan[np.isinf(self.current_scan)] = msg.range_max
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.collect_training_data()
        
    def extract_features(self):
        """Extract features from sensor data"""
        if self.current_scan is None or self.current_odom is None:
            return None
            
        # Lidar features
        scan_features = [
            np.min(self.current_scan),           # Closest obstacle
            np.mean(self.current_scan),          # Average distance
            np.std(self.current_scan),           # Distance variance
            np.sum(self.current_scan < 1.0),     # Number of close obstacles
        ]
        
        # Odometry features
        linear_vel = self.current_odom.twist.twist.linear.x
        angular_vel = self.current_odom.twist.twist.angular.z
        odom_features = [linear_vel, angular_vel]
        
        return np.array(scan_features + odom_features)
    
    def collect_training_data(self):
        """Collect training data for online learning"""
        if self.last_cmd is None:
            return
            
        features = self.extract_features()
        if features is None:
            return
            
        # Target is the command that was executed
        target = np.array([self.last_cmd.linear.x, self.last_cmd.angular.z])
        
        self.feature_buffer.append(features)
        self.target_buffer.append(target)
        
    def update_model(self, event):
        """Periodic model updates"""
        if len(self.feature_buffer) < 10:
            return
            
        # Convert buffers to arrays
        X = np.array(list(self.feature_buffer))
        y = np.array(list(self.target_buffer))
        
        if not self.is_fitted:
            # Initial fit
            X_scaled = self.scaler.fit_transform(X)
            self.model.fit(X_scaled, y)
            self.is_fitted = True
            rospy.loginfo("Model initially fitted")
        else:
            # Partial fit for online learning
            X_scaled = self.scaler.transform(X[-10:])  # Use recent data
            self.model.partial_fit(X_scaled, y[-10:])
            
        rospy.loginfo(f"Model updated with {len(X)} samples")
        
    def predict_command(self, features):
        """Predict command given current features"""
        if not self.is_fitted:
            return np.array([0.0, 0.0])
            
        features_scaled = self.scaler.transform(features.reshape(1, -1))
        prediction = self.model.predict(features_scaled)[0]
        return prediction
    
    def save_model(self, filename):
        """Save trained model"""
        model_data = {
            'model': self.model,
            'scaler': self.scaler,
            'is_fitted': self.is_fitted
        }
        with open(filename, 'wb') as f:
            pickle.dump(model_data, f)
        rospy.loginfo(f"Model saved to {filename}")
    
    def load_model(self, filename):
        """Load pre-trained model"""
        try:
            with open(filename, 'rb') as f:
                model_data = pickle.load(f)
            self.model = model_data['model']
            self.scaler = model_data['scaler']
            self.is_fitted = model_data['is_fitted']
            rospy.loginfo(f"Model loaded from {filename}")
        except FileNotFoundError:
            rospy.logwarn(f"Model file {filename} not found")

if __name__ == '__main__':
    rospy.init_node('ml_pipeline')
    pipeline = OnlineLearningPipeline()
    
    # Save model on shutdown
    def save_on_shutdown():
        pipeline.save_model('online_model.pkl')
    
    rospy.on_shutdown(save_on_shutdown)
    rospy.spin()
```

## Data Analysis Workflows

### Comprehensive Experiment Analysis
```python
#!/usr/bin/env python3
# experiment_analysis.py - Complete data analysis pipeline
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from rosbag_pandas import bag_to_dataframe
from scipy import stats
from sklearn.metrics import mean_squared_error, r2_score

class ExperimentAnalyzer:
    def __init__(self, bag_files):
        self.bag_files = bag_files if isinstance(bag_files, list) else [bag_files]
        self.data = {}
        
    def load_all_experiments(self):
        """Load data from all bag files"""
        for i, bag_file in enumerate(self.bag_files):
            rospy.loginfo(f"Loading {bag_file}...")
            df = bag_to_dataframe(bag_file)
            self.data[f'experiment_{i}'] = df
            
    def analyze_trajectory_performance(self):
        """Analyze trajectory tracking performance"""
        results = {}
        
        for exp_name, df in self.data.items():
            # Extract trajectory data
            odom_data = df[df.topic == '/odom']
            goal_data = df[df.topic == '/move_base_simple/goal']
            
            if len(odom_data) == 0 or len(goal_data) == 0:
                continue
                
            # Calculate trajectory metrics
            positions = odom_data[['pose.pose.position.x', 'pose.pose.position.y']].values
            
            # Path length
            path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            
            # Average speed
            times = odom_data.index.values
            avg_speed = path_length / (times[-1] - times[0])
            
            # Smoothness (jerk analysis)
            velocities = np.linalg.norm(
                odom_data[['twist.twist.linear.x', 'twist.twist.linear.y']].values, 
                axis=1
            )
            accelerations = np.diff(velocities)
            jerks = np.diff(accelerations)
            smoothness = np.std(jerks)
            
            results[exp_name] = {
                'path_length': path_length,
                'avg_speed': avg_speed,
                'smoothness': smoothness,
                'duration': times[-1] - times[0]
            }
            
        return pd.DataFrame(results).T
    
    def analyze_sensor_performance(self):
        """Analyze sensor data quality and performance"""
        sensor_analysis = {}
        
        for exp_name, df in self.data.items():
            # Lidar analysis
            scan_data = df[df.topic == '/scan']
            if len(scan_data) > 0:
                # Extract ranges
                ranges = np.array([
                    eval(row) if isinstance(row, str) else row 
                    for row in scan_data['ranges'].values
                ])
                
                # Quality metrics
                valid_rate = np.mean([
                    np.sum(np.isfinite(scan)) / len(scan) 
                    for scan in ranges
                ])
                
                avg_range = np.mean([
                    np.mean(scan[np.isfinite(scan)]) 
                    for scan in ranges if np.any(np.isfinite(scan))
                ])
                
                sensor_analysis[exp_name] = {
                    'lidar_valid_rate': valid_rate,
                    'avg_detection_range': avg_range
                }
                
        return pd.DataFrame(sensor_analysis).T
    
    def comparative_analysis(self):
        """Compare performance across experiments"""
        traj_results = self.analyze_trajectory_performance()
        sensor_results = self.analyze_sensor_performance()
        
        # Combine results
        combined = pd.concat([traj_results, sensor_results], axis=1)
        
        # Statistical analysis
        stats_summary = {
            'mean': combined.mean(),
            'std': combined.std(),
            'min': combined.min(),
            'max': combined.max()
        }
        
        return combined, pd.DataFrame(stats_summary)
    
    def generate_report(self, output_dir='analysis_results'):
        """Generate comprehensive analysis report"""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # Perform analyses
        combined_results, stats_summary = self.comparative_analysis()
        
        # Generate plots
        plt.figure(figsize=(15, 10))
        
        # Plot 1: Trajectory performance comparison
        plt.subplot(2, 3, 1)
        combined_results['path_length'].plot(kind='bar')
        plt.title('Path Length Comparison')
        plt.ylabel('Distance (m)')
        
        plt.subplot(2, 3, 2)
        combined_results['avg_speed'].plot(kind='bar')
        plt.title('Average Speed Comparison')
        plt.ylabel('Speed (m/s)')
        
        plt.subplot(2, 3, 3)
        combined_results['smoothness'].plot(kind='bar')
        plt.title('Trajectory Smoothness')
        plt.ylabel('Jerk STD')
        
        # Plot 2: Sensor performance
        plt.subplot(2, 3, 4)
        combined_results['lidar_valid_rate'].plot(kind='bar')
        plt.title('Lidar Data Quality')
        plt.ylabel('Valid Rate')
        
        # Plot 3: Correlation analysis
        plt.subplot(2, 3, 5)
        correlation_matrix = combined_results.corr()
        sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0)
        plt.title('Metric Correlations')
        
        # Plot 4: Statistical summary
        plt.subplot(2, 3, 6)
        stats_summary.T.plot(kind='bar')
        plt.title('Statistical Summary')
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        plt.savefig(f'{output_dir}/experiment_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Save detailed results
        combined_results.to_csv(f'{output_dir}/detailed_results.csv')
        stats_summary.to_csv(f'{output_dir}/statistical_summary.csv')
        
        print(f"Analysis complete. Results saved to {output_dir}/")
        return combined_results, stats_summary

# Usage example
if __name__ == '__main__':
    # Analyze multiple experiments
    bag_files = [
        'experiment_1.bag',
        'experiment_2.bag', 
        'experiment_3.bag'
    ]
    
    analyzer = ExperimentAnalyzer(bag_files)
    analyzer.load_all_experiments()
    results, summary = analyzer.generate_report()
    
    print("Experiment Analysis Summary:")
    print(summary)
```

## Deployment and Production

### Containerized Development
```dockerfile
# Dockerfile for ROS + Conda development environment
FROM osrf/ros:noetic-desktop-full

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

# Add conda to path
ENV PATH=/opt/conda/bin:$PATH

# Create robotics environment
RUN conda create -n robotics python=3.8 numpy opencv matplotlib && \
    /opt/conda/envs/robotics/bin/pip install rosbag_pandas

# Setup workspace
WORKDIR /catkin_ws
COPY . /catkin_ws/src/

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

### Automated Testing Pipeline
```bash
#!/bin/bash
# test_pipeline.sh - Automated testing for robotics projects

# Setup test environment
conda activate test_env
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Start test infrastructure
roscore &
ROSCORE_PID=$!
sleep 2

# Launch test simulation
roslaunch gazebo_ros empty_world.launch gui:=false &
GAZEBO_PID=$!
sleep 5

# Run unit tests
python -m pytest tests/unit/ -v

# Run integration tests
python -m pytest tests/integration/ -v

# Run system tests
timeout 60s python tests/system/test_full_system.py

# Cleanup
kill $ROSCORE_PID $GAZEBO_PID
wait
```

## Best Practices Summary

### Environment Management
1. **Use separate environments** for development, testing, and production
2. **Document environment dependencies** in environment.yml files
3. **Version control environment files** for reproducibility
4. **Test environment compatibility** with ROS integration

### Code Organization
1. **Modular design** - separate concerns into different packages
2. **Service-oriented architecture** for complex systems
3. **Comprehensive testing** at unit, integration, and system levels
4. **Documentation** for all interfaces and workflows

### Performance Optimization
1. **Profile code regularly** to identify bottlenecks
2. **Use vectorized operations** with NumPy for data processing
3. **Implement caching** for expensive computations
4. **Monitor system resources** during development and deployment

### Data Management
1. **Structured data collection** with proper metadata
2. **Version control data processing scripts** alongside code
3. **Automated analysis pipelines** for consistent results
4. **Backup and archive** important experimental data

## Next Steps

- **Basic Commands**: Review [Basic Commands](basic-commands.md) for fundamentals
- **Troubleshooting**: Check component-specific troubleshooting guides
- **Learning Path**: Follow [Learning Path](../resources/learning-path.md) for skill development
- **Community**: Engage with robotics communities for advanced techniques

---

**Advanced Robotics with Yara_OVE**

*These workflows use the Yara_OVE experimental playground for robotics experiments and research.*

*Building upon the work of the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE).*