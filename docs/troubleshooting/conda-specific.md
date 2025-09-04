# Conda-Specific Troubleshooting for Sailing AI

This guide addresses issues specific to Miniconda/Anaconda installation, environment management, and integration with ROS for sailing AI development in the Yara_OVE experimental playground.

## Installation Issues

### Miniconda Installation Problems
**Problem**: Miniconda installation fails or corrupts
```bash
bash: conda: command not found
CondaHTTPError: HTTP 000 CONNECTION FAILED
```

**Solutions**:
```bash
# Check if conda is properly installed
ls -la ~/miniconda3/bin/conda

# Re-run installer if installation incomplete
cd ~/Downloads/
bash Miniconda3-latest-Linux-x86_64.sh -b -p ~/miniconda3

# Add conda to PATH manually
export PATH=~/miniconda3/bin:$PATH

# Initialize conda for shell
~/miniconda3/bin/conda init bash
source ~/.bashrc

# Test conda installation
conda --version
conda info

# Fix corrupted installation
rm -rf ~/miniconda3/
# Download fresh installer and reinstall
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
```

### PATH and Environment Issues
**Problem**: Conda commands not found or environment conflicts
```bash
conda: command not found
CommandNotFoundError: Your shell has not been properly configured
```

**Solutions**:
```bash
# Check current PATH
echo $PATH | grep conda

# Manually add conda to PATH
export PATH=~/miniconda3/bin:$PATH

# Reinitialize conda
~/miniconda3/bin/conda init bash
source ~/.bashrc

# Check conda configuration
conda info --envs
conda config --show

# Fix PATH conflicts
# Edit ~/.bashrc to ensure conda comes after ROS sourcing
source /opt/ros/noetic/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate base

# Verify proper initialization
which conda
conda --version
```

## Environment Management Issues

### Environment Creation Problems
**Problem**: Can't create or activate conda environments
```bash
CondaValueError: could not parse 'python=3.8' -> InvalidVersionSpec
PackagesNotFoundError: The following packages are not available from current channels
```

**Solutions**:
```bash
# Update conda first
conda update conda

# Create environment with explicit version
conda create -n test_env python=3.8 -y

# Check available Python versions
conda search python

# Use different channels if packages not found
conda create -n myenv python=3.8 -c conda-forge

# Clear package cache if corrupted
conda clean --all

# Create environment from file
conda env create -f environment.yml

# Debug environment creation
conda create -n test_env python=3.8 --verbose

# Alternative: use mamba for faster package resolution
conda install mamba -c conda-forge
mamba create -n myenv python=3.8
```

### Activation and Deactivation Issues
**Problem**: Environment activation fails or doesn't work properly
```bash
CommandNotFoundError: conda activate
CondaError: cannot activate environment
```

**Solutions**:
```bash
# Use conda activate instead of source activate
conda activate myenv

# If activation fails, use source method
source activate myenv

# Check if environment exists
conda env list
conda info --envs

# Reinitialize conda shell integration
conda init bash
source ~/.bashrc

# Manual activation for older versions
source ~/miniconda3/etc/profile.d/conda.sh
conda activate myenv

# Fix activation in scripts
#!/bin/bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate myenv
python script.py

# Check activation status
echo $CONDA_DEFAULT_ENV
which python
```

### Package Installation Issues
**Problem**: Packages fail to install or create conflicts
```bash
UnsatisfiableError: The following specifications were found to be incompatible
ResolvePackageNotFound: Package missing in current channels
```

**Solutions**:
```bash
# Update conda and packages
conda update --all

# Install from specific channel
conda install package_name -c conda-forge

# Use pip for packages not in conda
conda activate myenv
pip install package_name

# Resolve conflicts by creating fresh environment
conda create -n fresh_env python=3.8
conda activate fresh_env
conda install required_packages

# Check package conflicts
conda install package_name --dry-run

# Force reinstall if corrupted
conda install package_name --force-reinstall

# Alternative: use mamba for better dependency resolution
conda install mamba -c conda-forge
mamba install package_name

# Export and recreate environment
conda env export > environment.yml
conda env remove -n myenv
conda env create -f environment.yml
```

## Sailing ROS Integration Issues

### Conda-Sailing ROS Environment Conflicts
**Problem**: ROS and conda environments interfere with sailing robotics development
```bash
ImportError: No module named rospy (when sailing_ai conda environment activated)
Python path conflicts between ROS and sailing AI conda packages
```

**Solutions**:
```bash
# Proper sourcing order for sailing robotics in ~/.bashrc
# 1. Source ROS for sailing robotics first
source /opt/ros/noetic/setup.bash

# 2. Initialize conda for sailing AI
source ~/miniconda3/etc/profile.d/conda.sh

# 3. Activate sailing AI environment
conda activate sailing_ai

# Create sailing-compatible ROS environment
conda create -n sailing_ai python=3.8
conda activate sailing_ai
pip install rospkg catkin_pkg rospy_message_converter
pip install marine_navigation_lib  # Hypothetical sailing package

# Set PYTHONPATH for sailing ROS and conda integration
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Test sailing ROS import in conda environment
python -c "import rospy; print('Sailing ROS import successful')"
python -c "import numpy; print('NumPy for sailing math available')"

# Create wrapper script for sailing ROS+conda
#!/bin/bash
# sailing_ros_conda_setup.sh
source /opt/ros/noetic/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sailing_ai
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
echo "Sailing robotics environment ready"
```

### Python Version Conflicts
**Problem**: ROS requires specific Python version that conflicts with conda
```bash
ROS Noetic requires Python 3, conda environment has wrong version
ImportError: dynamic module does not define module export function
```

**Solutions**:
```bash
# Check Python versions
python3 --version  # System Python for ROS
conda activate myenv
python --version   # Conda environment Python

# Create environment with ROS-compatible Python
conda create -n ros_compat python=3.8
conda activate ros_compat

# Verify ROS Python compatibility
python -c "
import sys
print(f'Python version: {sys.version}')
print(f'Python path: {sys.path}')
try:
    import rospy
    print('ROS import: SUCCESS')
except ImportError as e:
    print(f'ROS import failed: {e}')
"

# Use system Python for ROS development
conda create -n ros_dev python=3.8 --no-deps
conda activate ros_env
# Install only additional packages via conda/pip
conda install numpy matplotlib opencv
pip install additional_packages
```

## Performance and Resource Issues

### Slow Environment Creation/Updates
**Problem**: Conda operations are extremely slow
```bash
Solving environment: | takes very long time
Installing packages: hangs or very slow
```

**Solutions**:
```bash
# Use mamba for faster package management
conda install mamba -c conda-forge
mamba create -n myenv python=3.8 numpy

# Configure conda for better performance
conda config --set channel_priority strict
conda config --add channels conda-forge

# Clean conda cache
conda clean --all

# Use libmamba solver
conda install conda-libmamba-solver
conda config --set solver libmamba

# Disable unnecessary features
conda config --set auto_activate_base false
conda config --set report_errors false

# Use local package cache
conda config --set use_only_tar_bz2 true

# Parallel downloads
conda config --set remote_max_retries 3
conda config --set remote_connect_timeout_secs 30
```

### Memory and Disk Usage
**Problem**: Conda uses excessive disk space or memory
```bash
~/miniconda3/ directory very large
System runs out of memory during installations
```

**Solutions**:
```bash
# Check conda disk usage
du -sh ~/miniconda3/
du -sh ~/miniconda3/pkgs/  # Package cache
du -sh ~/miniconda3/envs/  # Environments

# Clean package cache
conda clean --packages
conda clean --tarballs
conda clean --index-cache

# Remove unused environments
conda env list
conda env remove -n unused_env

# Use conda-pack for efficient environment storage
conda install conda-pack
conda pack -n myenv -o myenv.tar.gz

# Configure package cache location
conda config --add pkgs_dirs /path/to/large/disk/conda-pkgs

# Monitor memory during operations
htop  # While running conda commands

# Limit memory usage
ulimit -v 4000000  # Limit virtual memory
conda install package_name
```

## Network and Proxy Issues

### Network Connection Problems
**Problem**: Conda can't download packages due to network issues
```bash
CondaHTTPError: HTTP 000 CONNECTION FAILED
SSLError: certificate verify failed
```

**Solutions**:
```bash
# Check network connectivity
ping conda.anaconda.org
curl -I https://conda.anaconda.org

# Configure proxy if needed
conda config --set proxy_servers.http http://proxy:port
conda config --set proxy_servers.https https://proxy:port

# Disable SSL verification (not recommended for production)
conda config --set ssl_verify false

# Use alternative channels
conda config --add channels conda-forge
conda config --add channels bioconda

# Configure channel priorities
conda config --show channels
conda config --set channel_priority strict

# Use offline installation
# Download packages on connected machine
conda install --download-only package_name
# Transfer to offline machine and install

# Check conda configuration
conda config --show
```

### Certificate and SSL Issues
**Problem**: SSL/TLS certificate verification failures
```bash
SSLError: [SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed
URLError: <urlopen error [SSL: CERTIFICATE_VERIFY_FAILED]>
```

**Solutions**:
```bash
# Update certificates
sudo apt update && sudo apt install ca-certificates

# Update conda itself
conda update conda

# Configure conda to use system certificates
conda config --set ssl_verify true

# Set certificate bundle path
export SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
conda config --set ssl_verify /etc/ssl/certs/ca-certificates.crt

# Temporary workaround (not recommended)
conda config --set ssl_verify false

# Check certificate configuration
conda config --show ssl_verify
python -c "import ssl; print(ssl.get_default_verify_paths())"
```

## Environment Export/Import Issues

### Environment File Problems
**Problem**: Environment export/import fails or creates incorrect environments
```bash
CondaEnvException: Could not parse environment.yml
ResolvePackageNotFound: package versions not available
```

**Solutions**:
```bash
# Export environment correctly
conda env export > environment.yml

# Export with explicit packages only
conda env export --from-history > environment.yml

# Cross-platform export (without builds)
conda env export --no-builds > environment.yml

# Manual environment.yml creation
name: myenv
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.8
  - numpy
  - matplotlib
  - pip
  - pip:
    - some-pip-package

# Create from file with error handling
conda env create -f environment.yml --verbose

# Update existing environment from file
conda env update -f environment.yml

# Export for specific platform
conda list --export > requirements.txt
conda create -n newenv --file requirements.txt
```

## Advanced Debugging

### Verbose Debugging
```bash
# Enable verbose output
conda install package_name --verbose
conda create -n myenv python=3.8 --verbose --dry-run

# Check conda configuration
conda info
conda config --show-sources
conda config --describe

# Debug environment activation
conda activate myenv --verbose

# Check package dependencies
conda search package_name --info
mamba repoquery depends package_name

# Trace package conflicts
mamba install package_name --dry-run --verbose
```

### Log Analysis
```bash
# Find conda logs
ls ~/.conda/
cat ~/.conda/environments.txt

# Check package cache
ls ~/miniconda3/pkgs/

# Verify environment integrity
conda list -n myenv
conda list --show-channel-urls

# Check for corrupted packages
conda install package_name --force-reinstall

# Validate environment
python -c "
import sys
import pkg_resources
print('Python executable:', sys.executable)
print('Installed packages:')
for pkg in pkg_resources.working_set:
    print(f'  {pkg.key}=={pkg.version}')
"
```

## Recovery Procedures

### Complete Conda Reset
```bash
#!/bin/bash
# reset_conda.sh - Complete conda reset

# Backup environment list
conda env list > ~/conda_envs_backup.txt

# Remove all conda environments except base
for env in $(conda env list | awk '{print $1}' | grep -v '^#' | grep -v '^base$'); do
    conda env remove -n $env -y
done

# Clean all cache
conda clean --all -y

# Update conda
conda update conda -y

# Reinitialize
conda init bash
source ~/.bashrc

echo "Conda reset complete"
```

### Environment Recreation
```bash
#!/bin/bash
# recreate_env.sh - Recreate corrupted environment

ENV_NAME=$1
if [ -z "$ENV_NAME" ]; then
    echo "Usage: $0 <environment_name>"
    exit 1
fi

# Export current environment
conda env export -n $ENV_NAME > ${ENV_NAME}_backup.yml

# Remove environment
conda env remove -n $ENV_NAME -y

# Recreate from backup
conda env create -f ${ENV_NAME}_backup.yml

echo "Environment $ENV_NAME recreated"
```

### Package Cache Cleanup
```bash
#!/bin/bash
# cleanup_conda.sh - Clean conda package cache

echo "Before cleanup:"
du -sh ~/miniconda3/

# Clean package cache
conda clean --packages -y
conda clean --tarballs -y
conda clean --index-cache -y

# Remove temporary files
rm -rf ~/miniconda3/pkgs/.tmp*

echo "After cleanup:"
du -sh ~/miniconda3/
```

## Integration Best Practices

### Sailing ROS + Conda Workflow
```bash
# 1. Create sailing AI environment
conda create -n sailing_ai python=3.8 -y
conda activate sailing_ai

# 2. Install sailing-specific scientific packages
conda install numpy scipy matplotlib opencv -c conda-forge
conda install jupyter pandas scikit-learn pytorch -c conda-forge

# 3. Install sailing ROS-compatible packages
pip install rospkg catkin_pkg rospy_message_converter
pip install geopy pyproj  # For GPS and navigation calculations
pip install windrose  # For wind data analysis

# 4. Create sailing activation script
cat > ~/activate_sailing_ros_conda.sh << 'EOF'
#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sailing_ai
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
echo "Sailing ROS + Conda AI environment activated"
echo "Ready for autonomous sailing development!"
EOF

chmod +x ~/activate_sailing_ros_conda.sh

# 5. Use in sailing development
source ~/activate_sailing_ros_conda.sh
python your_sailing_ai_script.py
jupyter notebook  # For sailing data analysis
```

### Sailing AI Environment Templates
```yaml
# environment_sailing_ai.yml - Template for sailing AI development
name: sailing_ai
channels:
  - conda-forge
  - pytorch
  - defaults
dependencies:
  - python=3.8
  - numpy
  - scipy
  - matplotlib
  - opencv
  - jupyter
  - pandas
  - scikit-learn
  - pytorch
  - pytest
  - pip
  - pip:
    - rospkg
    - catkin_pkg
    - rospy_message_converter
    - rosbag_pandas
    - geopy
    - pyproj
    - windrose
    - marine_weather_api
```

```yaml
# environment_sailing_vision.yml - Template for marine computer vision
name: sailing_vision
channels:
  - conda-forge
  - pytorch
  - defaults
dependencies:
  - python=3.8
  - numpy
  - opencv
  - pytorch
  - torchvision
  - pillow
  - matplotlib
  - jupyter
  - pip
  - pip:
    - rospkg
    - cv_bridge
    - marine_object_detection
```

```yaml
# environment_sailing_rl.yml - Template for sailing reinforcement learning
name: sailing_rl
channels:
  - conda-forge
  - pytorch
  - defaults
dependencies:
  - python=3.8
  - numpy
  - matplotlib
  - pytorch
  - gymnasium
  - stable-baselines3
  - tensorboard
  - jupyter
  - pip
  - pip:
    - rospkg
    - sailing_gym_env
    - yara_ove_rl
```

## Useful Diagnostic Commands

```bash
# System check
conda --version
conda info
conda config --show

# Environment check
conda env list
conda list
conda list --show-channel-urls

# Package search and info
conda search package_name
conda search package_name --info

# Debugging commands
conda install package_name --dry-run
conda create -n test --dry-run python=3.8

# Performance monitoring
time conda install package_name
du -sh ~/miniconda3/
```

## Related Documentation

- **Common Issues**: [Common Issues](common-issues.md) for general sailing robotics troubleshooting
- **Installation**: [Miniconda Installation](../installation/miniconda.md) for sailing AI setup
- **Usage**: [Sailing AI Environments](../usage/python-environments.md) for sailing development operations
- **Verification**: [Installation Verification](../installation/verification.md) for sailing AI testing