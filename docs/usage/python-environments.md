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