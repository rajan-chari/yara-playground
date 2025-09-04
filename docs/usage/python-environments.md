# Python Environments with Miniconda

This guide covers practical usage of Miniconda for Python environment management in the Yara_OVE experimental playground.

## Getting Started with Environment Management

### Basic Environment Operations
```bash
# Create a new environment
conda create -n my_env python=3.9

# Create with specific packages
conda create -n my_env python=3.9 numpy matplotlib opencv

# Activate environment
conda activate my_env

# Check current environment
conda info --envs
echo $CONDA_DEFAULT_ENV

# Deactivate environment
conda deactivate

# Remove environment
conda env remove -n my_env
```

### Environment from File
```bash
# Export current environment
conda env export > environment.yml

# Create environment from file
conda env create -f environment.yml

# Update environment from file
conda env update -f environment.yml
```


## YARA-OVE Sailing Robotics Environments

### Autonomous Sailing AI Environment
```bash
# Create comprehensive sailing AI environment
conda create -n yara_sailing_ai python=3.9
conda activate yara_sailing_ai

# Install core scientific computing stack
conda install numpy pandas matplotlib seaborn scipy
conda install scikit-learn jupyter jupyterlab

# Install deep learning frameworks for sailing AI
conda install pytorch torchvision -c pytorch
pip install stable-baselines3[extra]  # Advanced RL algorithms
pip install gym                       # Environment framework

# Install sailing-specific packages
pip install rospkg rospy_message_converter
pip install sensor_msgs_py
pip install marine_navigation  # Hypothetical sailing package
pip install sailing_gym        # Sailing RL environments

# Install data analysis tools for sailing experiments
pip install rosbag rosbag_pandas bagpy
conda install -c conda-forge pyarrow

# Verify sailing AI environment
python -c "
import torch, stable_baselines3, gym, pandas as pd
import numpy as np, matplotlib.pyplot as plt
print('YARA-OVE Sailing AI environment ready!')
print(f'PyTorch: {torch.__version__}')
print(f'SB3: {stable_baselines3.__version__}')
"
```

### Ocean Wave Physics Environment
```bash
# Environment specialized for wave physics analysis
conda create -n yara_wave_physics python=3.9
conda activate yara_wave_physics

# Install mathematical computing packages
conda install numpy scipy matplotlib
conda install -c conda-forge numba  # JIT compilation for wave calculations

# Install 3D visualization for wave analysis
conda install -c conda-forge pyvista mayavi
conda install vtk

# Install signal processing for wave analysis
conda install -c conda-forge librosa soundfile
pip install spectrum  # Spectral analysis

# Install YARA-OVE specific tools
pip install rospkg
pip install gerstner_waves  # Hypothetical wave package
pip install ocean_toolkit   # Marine physics utilities

# Example: Analyze Gerstner wave parameters
cat << 'EOF' > wave_analysis.py
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def analyze_pierson_moskowitz_spectrum(wind_speed=10, fetch=1000):
    """Analyze PM spectrum for YARA-OVE wave simulation"""
    # Frequency range for analysis
    f = np.linspace(0.05, 1.0, 1000)
    
    # Pierson-Moskowitz spectrum
    g = 9.81  # Gravity
    alpha = 8.1e-3  # Phillips constant
    
    # Peak frequency
    fp = 0.14 * (g / wind_speed)
    
    # PM spectrum
    S_f = (alpha * g**2 / (2*np.pi)**4) * f**-5 * np.exp(-1.25 * (fp/f)**4)
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.loglog(f, S_f)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Spectral Density')
    plt.title('Pierson-Moskowitz Spectrum')
    plt.grid(True)
    
    # Wave height statistics
    m0 = np.trapz(S_f, f)  # Zeroth moment
    Hs = 4 * np.sqrt(m0)   # Significant wave height
    
    plt.subplot(2, 2, 2)
    plt.plot([Hs], [0], 'ro', markersize=10)
    plt.title(f'Significant Wave Height: {Hs:.2f} m')
    plt.ylabel('Wave Height (m)')
    
    print(f"Wind Speed: {wind_speed} m/s")
    print(f"Significant Wave Height: {Hs:.2f} m")
    print(f"Peak Period: {1/fp:.2f} s")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_pierson_moskowitz_spectrum()
EOF

python wave_analysis.py
```

### ESailor Reinforcement Learning Environment
```bash
# Specialized environment for ESailor RL framework integration
conda create -n yara_esailor_rl python=3.9
conda activate yara_esailor_rl

# Install reinforcement learning stack
pip install stable-baselines3[extra]
pip install ray[rllib]  # Distributed RL training
pip install wandb      # Experiment tracking

# Install sailing RL specific packages
pip install gym gymnasium
pip install sailing_gym esailor_framework  # Hypothetical packages
pip install marine_rl_utils

# Install PyTorch for policy networks
conda install pytorch torchvision -c pytorch

# Install ROS integration for real-time learning
pip install rospkg rospy_message_converter
source /opt/ros/noetic/setup.bash

# Example: ESailor training setup
cat << 'EOF' > esailor_training.py
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

class SailingEnvironment(gym.Env):
    """YARA-OVE Sailing Environment for RL"""
    
    def __init__(self):
        super(SailingEnvironment, self).__init__()
        
        # Define action space: [sail_angle, rudder_angle]
        self.action_space = gym.spaces.Box(
            low=np.array([-1.57, -0.7]), 
            high=np.array([1.57, 0.7]),
            dtype=np.float32
        )
        
        # Define observation space: [position, heading, wind, target]
        self.observation_space = gym.spaces.Box(
            low=np.array([-1000, -1000, -3.14, 0, -3.14, 0]),
            high=np.array([1000, 1000, 3.14, 20, 3.14, 1000]),
            dtype=np.float32
        )
        
        self.reset()
    
    def reset(self):
        self.state = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 100.0])
        return self.state
    
    def step(self, action):
        # Simplified sailing dynamics
        sail_angle, rudder_angle = action
        
        # Update state based on sailing physics
        reward = self.calculate_reward()
        done = self.check_done()
        
        return self.state, reward, done, {}
    
    def calculate_reward(self):
        # Sailing-specific reward function
        distance_to_target = self.state[5]
        speed_reward = np.linalg.norm(self.state[:2]) * 0.1
        return -distance_to_target * 0.001 + speed_reward

def train_sailing_agent():
    env = DummyVecEnv([lambda: SailingEnvironment()])
    
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=10000)
    
    model.save("yara_sailing_agent")
    print("ESailor agent trained successfully!")

if __name__ == "__main__":
    train_sailing_agent()
EOF
```

### Digital Twin Validation Environment
```bash
# Environment for E-Boat digital twin validation
conda create -n yara_digital_twin python=3.9
conda activate yara_digital_twin

# Install scientific computing and validation tools
conda install numpy pandas matplotlib scipy
conda install scikit-learn statsmodels

# Install data comparison and validation packages
pip install pingouin  # Statistical analysis
conda install -c conda-forge seaborn plotly

# Install ROS packages for real boat data
pip install rospkg rosbag rosbag_pandas
pip install sensor_msgs_py geometry_msgs

# Install marine engineering packages
pip install marine_engineering_toolbox  # Hypothetical
pip install boat_dynamics_analyzer

# Example: Digital twin validation script
cat << 'EOF' > digital_twin_validation.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, r2_score

class DigitalTwinValidator:
    def __init__(self):
        self.sim_data = []
        self.real_data = []
    
    def load_simulation_data(self, bag_file):
        """Load YARA-OVE simulation data"""
        # Simulated data loading
        self.sim_data = pd.read_csv('simulation_results.csv')
    
    def load_real_boat_data(self, bag_file):
        """Load real E-Boat data"""
        # Real data loading from ROS bag
        self.real_data = pd.read_csv('real_boat_data.csv')
    
    def validate_trajectory(self):
        """Validate trajectory prediction accuracy"""
        if len(self.sim_data) == 0 or len(self.real_data) == 0:
            return None
            
        # Align timestamps and compare trajectories
        mse = mean_squared_error(self.real_data['position'], 
                                self.sim_data['position'])
        r2 = r2_score(self.real_data['position'], 
                     self.sim_data['position'])
        
        print(f"Trajectory Validation Results:")
        print(f"MSE: {mse:.4f}")
        print(f"R² Score: {r2:.4f}")
        
        return {'mse': mse, 'r2': r2}
    
    def validate_sailing_performance(self):
        """Validate sailing performance metrics"""
        metrics = {}
        
        # Speed validation
        real_speed = np.mean(self.real_data['speed'])
        sim_speed = np.mean(self.sim_data['speed'])
        speed_error = abs(real_speed - sim_speed) / real_speed
        
        metrics['speed_error'] = speed_error
        
        print(f"Speed Validation: {speed_error:.2%} error")
        return metrics

# Usage example
validator = DigitalTwinValidator()
# validator.load_simulation_data('yara_simulation.bag')
# validator.load_real_boat_data('real_eboat.bag')
# results = validator.validate_trajectory()
EOF
```

### Performance Benchmarking Environment
```bash
# Environment for YARA-OVE performance analysis
conda create -n yara_performance python=3.9
conda activate yara_performance

# Install performance monitoring tools
conda install pandas numpy matplotlib
pip install psutil memory_profiler
pip install py-spy  # Python profiler

# Install ROS performance tools
pip install rospkg rostopic
source /opt/ros/noetic/setup.bash

# Example: Performance benchmarking script
cat << 'EOF' > performance_benchmark.py
import time
import psutil
import subprocess
from memory_profiler import profile

class YARAPerformanceBenchmark:
    def __init__(self):
        self.results = {}
    
    def benchmark_ocean_launch(self):
        """Benchmark ocean world launch time"""
        start_time = time.time()
        
        # Launch ocean world (simulated)
        print("Benchmarking ocean world launch...")
        time.sleep(2)  # Simulated launch time
        
        launch_time = time.time() - start_time
        self.results['ocean_launch_time'] = launch_time
        print(f"Ocean launch time: {launch_time:.2f}s")
    
    def benchmark_wave_rendering(self):
        """Benchmark wave rendering performance"""
        print("Benchmarking wave rendering...")
        
        # Simulate wave rendering benchmark
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_usage = psutil.virtual_memory().percent
        
        self.results['cpu_usage'] = cpu_percent
        self.results['memory_usage'] = memory_usage
        
        print(f"CPU Usage: {cpu_percent}%")
        print(f"Memory Usage: {memory_usage}%")
    
    def benchmark_300x_speedup(self):
        """Test 300x speedup capability"""
        print("Testing 300x speedup capability...")
        
        # Simulate speedup test
        base_time = 10.0  # 10 seconds real time
        speedup_time = base_time / 150  # Achieved speedup
        
        self.results['achieved_speedup'] = base_time / speedup_time
        print(f"Achieved speedup: {self.results['achieved_speedup']:.1f}x")

# Usage
benchmark = YARAPerformanceBenchmark()
benchmark.benchmark_ocean_launch()
benchmark.benchmark_wave_rendering()
benchmark.benchmark_300x_speedup()
EOF
```

## Specialized Environments

### Computer Vision Environment
```bash
# Create computer vision environment
conda create -n computer_vision python=3.9
conda activate computer_vision

# Install core vision packages
conda install numpy opencv matplotlib pillow
conda install -c conda-forge scikit-image

# Install deep learning frameworks
conda install pytorch torchvision -c pytorch
# or
conda install tensorflow

# Install additional tools
pip install rosbag_pandas  # For ROS data analysis
pip install open3d         # For 3D processing

# Verify setup
python -c "import cv2, numpy as np, torch; print('Computer vision environment ready')"
```

### Machine Learning Environment
```bash
# Create ML environment
conda create -n machine_learning python=3.9
conda activate machine_learning

# Install scientific computing stack
conda install numpy pandas matplotlib seaborn jupyter
conda install scikit-learn scipy

# Install specialized ML libraries
conda install -c conda-forge xgboost lightgbm
pip install catboost

# Install robotics tools
pip install rospkg
pip install sensor_msgs_py
pip install gym  # For environment simulation
pip install stable-baselines3  # For RL algorithms

# Start Jupyter for development
jupyter notebook
```

### ROS Development Environment
```bash
# Create ROS-compatible Python environment
conda create -n ros_development python=3.8  # ROS Noetic compatibility
conda activate ros_development

# Install ROS-compatible packages
conda install numpy matplotlib scipy
pip install rospkg catkin_pkg
pip install rospy_message_converter

# Verify ROS compatibility
source /opt/ros/noetic/setup.bash
python -c "import rospy; print('ROS environment ready')"
```

### Data Analysis Environment
```bash
# Create data analysis environment
conda create -n data_analysis python=3.9
conda activate data_analysis

# Install data science stack
conda install pandas numpy matplotlib seaborn plotly
conda install jupyter jupyterlab

# Install data processing tools
pip install rosbag rosbag_pandas
pip install bagpy  # Modern bag file analysis

# Install additional analysis tools
conda install -c conda-forge pyarrow fastparquet
conda install statsmodels
```

## Package Management Best Practices

### Using Conda vs Pip
```bash
# Prefer conda for scientific packages
conda install numpy pandas matplotlib opencv

# Use pip for pure Python packages or when conda version unavailable
pip install some_pure_python_package

# Check where packages come from
conda list  # Shows conda and pip packages
```

### Managing Dependencies
```bash
# Install packages with specific versions
conda install numpy=1.21.0 pandas>=1.3.0

# Install from specific channels
conda install -c conda-forge package_name
conda install -c pytorch pytorch

# Pin packages to prevent updates
conda install numpy=1.21.0 --no-update-deps
```

### Environment File Best Practices
```yaml
# environment.yml example
name: robotics_env
channels:
  - conda-forge
  - pytorch
  - defaults
dependencies:
  - python=3.9
  - numpy>=1.20
  - matplotlib
  - opencv
  - pytorch
  - pip
  - pip:
    - rosbag_pandas
    - some_pip_only_package
```

## ROS Integration Workflows

### ROS + Python Development
```bash
# Method 1: Activate conda env after sourcing ROS
source /opt/ros/noetic/setup.bash
conda activate my_env
# Now you have both ROS and conda packages

# Method 2: Create conda env with ROS-compatible Python
conda create -n my_project python=3.8
conda activate my_project
source /opt/ros/noetic/setup.bash
```

### Data Analysis Workflow
```bash
# Create analysis environment
conda create -n data_analysis python=3.9 pandas matplotlib jupyter
conda activate data_analysis
pip install rosbag_pandas bagpy

# Example data analysis script
cat << 'EOF' > analyze_data.py
import rosbag_pandas as rbp
import matplotlib.pyplot as plt

# Read ROS bag data
df = rbp.bag_to_dataframe('experiment.bag')

# Analyze trajectory
trajectory = df[df.topic == '/odom']
sensor_data = df[df.topic == '/sensor_data']

plt.figure(figsize=(15, 6))

# Plot trajectory
plt.subplot(1, 2, 1)
plt.plot(trajectory['pose.pose.position.x'], trajectory['pose.pose.position.y'])
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.grid(True)

# Plot sensor data
plt.subplot(1, 2, 2)
plt.plot(sensor_data.index, sensor_data['data'])
plt.xlabel('Time')
plt.ylabel('Sensor Value')
plt.title('Sensor Data')
plt.grid(True)

plt.tight_layout()
plt.show()
EOF

python analyze_data.py
```

### Sensor Data Processing
```bash
# Environment for sensor data processing
conda create -n sensors python=3.9
conda activate sensors

# Install sensor processing packages
conda install numpy opencv matplotlib
conda install -c conda-forge open3d  # Point clouds
pip install pyrealsense2             # RealSense cameras
pip install rospy_message_converter    # ROS message conversion

# Example: Process camera images from ROS bag
cat << 'EOF' > process_images.py
import cv2
import rosbag
from cv_bridge import CvBridge

bridge = CvBridge()
bag = rosbag.Bag('camera_data.bag')

for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Process image here
    cv2.imshow('Image', cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

bag.close()
cv2.destroyAllWindows()
EOF
```

## Advanced Environment Management

### Environment Cloning and Sharing
```bash
# Clone existing environment
conda create --name new_env --clone existing_env

# Export environment with exact versions
conda env export --no-builds > environment.yml

# Create environment with exact package versions
conda env create -f environment.yml --name exact_env
```

### Multiple Python Versions
```bash
# Create environments with different Python versions
conda create -n py38_env python=3.8
conda create -n py39_env python=3.9
conda create -n py310_env python=3.10

# Use for testing compatibility
conda activate py38_env
python my_script.py

conda activate py39_env
python my_script.py
```

### Performance Optimization
```bash
# Install mamba for faster package resolution
conda install mamba -n base -c conda-forge

# Use mamba instead of conda for faster operations
mamba create -n fast_env python=3.9 numpy pandas
mamba install pytorch torchvision -c pytorch
```

## Development Workflows

### Jupyter Development Workflow
```bash
# Create development environment
conda create -n jupyter_dev python=3.9
conda activate jupyter_dev

# Install Jupyter and extensions
conda install jupyterlab jupyter
conda install -c conda-forge jupyterlab-git
conda install ipywidgets

# Install kernel for environment
python -m ipykernel install --user --name jupyter_dev --display-name "Robotics Dev"

# Start JupyterLab
jupyter lab

# In Jupyter, select the "Robotics Dev" kernel
```

### Testing Workflow
```bash
# Create testing environment
conda create -n testing python=3.9
conda activate testing

# Install testing tools
conda install pytest pytest-cov
conda install -c conda-forge hypothesis

# Install your package in development mode
pip install -e .

# Run tests
pytest tests/ --cov=my_package
```

### Package Development Workflow
```bash
# Create package development environment
conda create -n pkg_dev python=3.9
conda activate pkg_dev

# Install development tools
conda install pip setuptools wheel twine
conda install -c conda-forge pre-commit black flake8

# Install your package in editable mode
pip install -e .

# Set up pre-commit hooks
pre-commit install
```

## Integration Examples

### ROS + OpenCV + Deep Learning
```bash
# Complete vision pipeline environment
conda create -n vision_pipeline python=3.8
conda activate vision_pipeline

# Install ROS-compatible packages
pip install rospkg rospy_message_converter

# Install vision packages
conda install opencv numpy matplotlib
conda install pytorch torchvision -c pytorch

# Install ROS bridge packages
pip install cv_bridge  # Note: might need system cv_bridge

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Example: Real-time object detection node
cat << 'EOF' > vision_node.py
#!/usr/bin/env python3
import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/detections', Image, queue_size=1)
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        annotated = results.render()[0]
        
        out_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        self.pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('vision_node')
    node = VisionNode()
    rospy.spin()
EOF

chmod +x vision_node.py
```

### ROS + Scientific Computing
```bash
# Scientific computing environment for robotics
conda create -n sci_robotics python=3.8
conda activate sci_robotics

# Install scientific stack
conda install numpy scipy matplotlib pandas
conda install scikit-learn

# Install ROS packages
pip install rospkg rospy_message_converter

# Install robotics-specific packages
pip install robotics-toolbox-python  # Peter Corke's toolbox
pip install modern_robotics           # Modern Robotics library

# Source ROS
source /opt/ros/noetic/setup.bash
```

## Troubleshooting

### Common Issues
```bash
# Environment activation not working
conda init bash
source ~/.bashrc

# Package conflicts
conda update --all
conda clean --all

# SSL certificate issues
conda config --set ssl_verify false  # Temporary fix
```

### Environment Debugging
```bash
# Check environment details
conda info
conda list
conda env list

# Find package locations
python -c "import package_name; print(package_name.__file__)"

# Check Python path
python -c "import sys; print('\n'.join(sys.path))"
```

## Best Practices Summary

1. **Use environment files** for reproducible setups
2. **Keep environments focused** - one environment per project type
3. **Document dependencies** clearly in environment.yml
4. **Test environments** before sharing with team
5. **Use mamba** for faster package operations
6. **Backup important environments** with export
7. **Keep base environment clean** - avoid installing packages there
8. **Use specific package versions** for production environments

## Next Steps

- **Advanced Integration**: See [Advanced Workflows](advanced-workflows.md) for complex pipelines
- **Basic Commands**: Review [Basic Commands](basic-commands.md) for fundamentals
- **Troubleshooting**: Check [Conda-Specific Issues](../troubleshooting/conda-specific.md)
- **Learning Path**: Follow [Learning Path](../resources/learning-path.md) for skill development

---

**🚀 Powerful Python Environment Management!**

*Master these environment management techniques to fully leverage the Yara_OVE experimental playground's potential for machine learning, data analysis, and algorithm development.*

*Supporting the original [Yara_OVE project](https://github.com/medialab-fboat/Yara_OVE) research.*