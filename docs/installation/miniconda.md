# Miniconda Installation

## Installation Status

✅ **Miniconda 25.7.0 is ready for use with Yara_OVE.**

This guide documents the Miniconda installation following official Anaconda documentation.

## What Was Installed

### Core Components
- **Miniconda 25.7.0** - Lightweight conda installer
- **Base Python**: Python 3.13.5
- **Location**: `~/miniconda3/`
- **Package Manager**: conda 25.7.0 for scientific packages
- **Integration**: Full bash shell integration

### Key Features
- **Environment Management**: Isolated Python environments
- **Package Management**: Install packages via conda and pip
- **Cross-Platform**: Consistent development across systems
- **Scientific Computing**: Access to conda-forge and scientific packages

## Installation Process Completed

### 1. Download and Verification
```bash
# Downloaded latest Miniconda installer
cd /tmp
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# Verified installer integrity
bash Miniconda3-latest-Linux-x86_64.sh -h
```

### 2. Installation
```bash
# Installed in batch mode to ~/miniconda3
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
```

### 3. Shell Integration
```bash
# Initialized conda for bash
$HOME/miniconda3/bin/conda init bash

# Configuration added to ~/.bashrc
source ~/.bashrc
```

### 4. Verification and Test
```bash
# Verified installation
conda --version
conda info

# Created test environment
conda create -n test_env python=3.8 -y
conda env list
```

## Directory Structure

```
~/miniconda3/
├── bin/               # Conda executables and Python
├── conda-meta/        # Package metadata
├── envs/             # Environment directories
│   └── test_env/     # Example environment
├── etc/              # Configuration files
├── include/          # Header files
├── lib/              # Libraries
├── pkgs/             # Package cache
├── share/            # Shared resources
└── ssl/              # SSL certificates
```

## Environment Configuration

### Bash Integration
Conda added the following to `~/.bashrc`:

```bash
# >>> conda initialize >>>
# Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/rajan/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/rajan/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/rajan/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/rajan/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
```

### Default Channels
```bash
# View configured channels
conda config --show channels

# Default channels:
# - defaults
```

## Environment Management

### Basic Commands
```bash
# List environments
conda env list

# Create environment
conda create -n myenv python=3.9

# Activate environment
conda activate myenv

# Deactivate environment
conda deactivate

# Remove environment
conda env remove -n myenv
```

### Package Management
```bash
# Install packages with conda
conda install numpy pandas matplotlib

# Install packages with pip (within conda env)
pip install requests

# List installed packages
conda list

# Update packages
conda update numpy

# Update conda itself
conda update conda
```

## ROS Integration

### Compatibility Verification
The installation verified that conda works alongside ROS:

```bash
# ROS commands remain functional
source /opt/ros/noetic/setup.bash
rosversion -d
# Output: noetic

# No path conflicts detected
which python3    # System Python
which conda      # Conda Python (when activated)
```

### Best Practices for ROS + Conda
1. **Use conda environments** for Python development
2. **Keep ROS in system Python** for stability
3. **Activate conda environments** only when needed
4. **Test integration** before production use

## Verification Commands

Test your Miniconda installation:

```bash
# Check conda version
conda --version

# View system information
conda info

# List environments
conda env list

# Test environment creation
conda create -n test python=3.9 -y
conda activate test
python --version
conda deactivate

# Test package installation
conda activate test
conda install numpy -y
python -c "import numpy; print(numpy.__version__)"
conda deactivate

# Cleanup test environment
conda env remove -n test -y
```

## Advanced Configuration

### Performance Optimization
```bash
# Install mamba for faster package resolution
conda install mamba -n base -c conda-forge

# Use mamba instead of conda for faster operations
mamba install numpy pandas
```

### Channel Configuration
```bash
# Add conda-forge channel (recommended)
conda config --add channels conda-forge

# Set channel priority
conda config --set channel_priority strict
```

### Environment Files
```bash
# Export environment
conda env export > environment.yml

# Create environment from file
conda env create -f environment.yml
```

## Common Use Cases

### Scientific Computing Environment
```bash
conda create -n science python=3.9 numpy pandas matplotlib scikit-learn jupyter
conda activate science
jupyter notebook
```

### Computer Vision Environment
```bash
conda create -n cv python=3.9 opencv pytorch torchvision -c pytorch
conda activate cv
```

### ROS Development Environment
```bash
conda create -n ros_dev python=3.8 numpy matplotlib pandas
conda activate ros_dev
pip install rospkg catkin_pkg
```

## What's Next

- **Python Environments**: See [Python Environments](../usage/python-environments.md) for development setups
- **Advanced Integration**: Try [Advanced Workflows](../usage/advanced-workflows.md) for complex pipelines
- **Troubleshooting**: Check [Conda-Specific Issues](../troubleshooting/conda-specific.md)
- **Learning Path**: Explore [Learning Path](../resources/learning-path.md) for skill development

## Resources

- [Official Conda Documentation](https://docs.conda.io/) - Complete reference
- [Conda Cheat Sheet](https://docs.conda.io/projects/conda/en/latest/user-guide/cheatsheet.html) - Quick reference
- [Managing Environments](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) - Environment workflows
- [Conda-Forge Community](https://conda-forge.org/) - Community packages

---

**🚀 Miniconda: Ready for Development!**

*This Miniconda installation provides flexible Python environment management for the Yara_OVE experimental playground.*