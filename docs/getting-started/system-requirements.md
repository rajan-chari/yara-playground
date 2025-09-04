# System Requirements

## Operating System Compatibility

### Supported Systems
- **Ubuntu 20.04 LTS (Focal Fossa)** - ✅ **Recommended and Tested**
- Ubuntu 18.04 LTS (Bionic Beaver) - Limited support
- Ubuntu 22.04 LTS (Jammy Jellyfish) - Experimental

### Hardware Requirements

#### Minimum Requirements
- **CPU**: 2 cores, 2.0 GHz
- **RAM**: 4 GB
- **Storage**: 10 GB free space
- **Graphics**: Intel HD Graphics or equivalent

#### Recommended Requirements
- **CPU**: 4+ cores, 2.5+ GHz
- **RAM**: 8+ GB
- **Storage**: 20+ GB free space (SSD preferred)
- **Graphics**: Dedicated GPU for Gazebo simulation

## Current System Status

Your system meets all requirements:

- **OS**: Ubuntu 20.04.6 LTS (focal) ✅
- **Architecture**: x86_64 ✅
- **Available Space**: 954 GB ✅
- **Environment**: WSL2 compatible ✅

## Component Requirements

### ROS Noetic
- Ubuntu 20.04 LTS (primary target)
- Python 3.8+
- GCC 9.x compiler

### Gazebo Classic 11
- OpenGL 3.3+ support
- Mesa 18.0+ or proprietary GPU drivers
- 2 GB RAM for basic simulation

### Miniconda
- 400 MB disk space (base installation)
- Internet connection for package downloads
- Bash shell for optimal integration

## Network Requirements

- **Internet connection** for package downloads
- **Bandwidth**: 100 Mbps recommended for initial installation
- **Firewall**: Allow HTTP/HTTPS for package repositories

## Known Limitations

### WSL2
- ✅ **ROS**: Full support
- ✅ **Gazebo**: GUI support with X11 forwarding
- ✅ **Miniconda**: Full support

### Virtual Machines
- ⚠️ **3D acceleration** may be limited
- ⚠️ **Performance** impact on simulation
- ✅ **Development** workflows supported

## Verification Commands

Check your system compatibility:

```bash
# OS Version
lsb_release -a

# Architecture
uname -m

# Available space
df -h

# Memory
free -h

# CPU info
nproc