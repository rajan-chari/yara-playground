# System Requirements

## Operating System Compatibility

### Supported Systems
- **Ubuntu 20.04 LTS (Focal Fossa)** - ✅ **Recommended**
- Ubuntu 18.04 LTS (Bionic Beaver) - Limited support
- Ubuntu 22.04 LTS (Jammy Jellyfish) - Experimental compatibility

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
- **Graphics**: Dedicated GPU (enhanced rendering and processing)

## System Verification

To check if your system meets the requirements:

```bash
# Check OS version
lsb_release -a

# Check architecture
uname -m

# Check available disk space
df -h

# Check memory
free -h
```

## Component Requirements

### ROS Noetic
- Ubuntu 20.04 LTS (optimal compatibility)
- Python 3.8+ (required for modern packages)
- GCC 9.x compiler (for building packages)

### Gazebo Classic 11
- OpenGL 3.3+ support
- Mesa 18.0+ or proprietary GPU drivers
- 2 GB RAM for basic simulation (4+ GB for complex scenarios)

### Miniconda
- 400 MB disk space (base installation)
- Internet connection for package downloads
- Bash shell compatibility

## Network Requirements

- **Internet connection** for package downloads and updates
- **Bandwidth**: 100 Mbps recommended for large assets
- **Firewall**: Allow HTTP/HTTPS for package repositories

## Known Limitations

### WSL2 Development
- ✅ **ROS**: Full support
- ✅ **Gazebo**: GUI support with X11 forwarding
- ✅ **Miniconda**: Complete development support

### Virtual Machines
- ⚠️ **3D acceleration** may limit visualization quality
- ⚠️ **Performance** impact on complex simulations
- ✅ **Development** workflows fully supported

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