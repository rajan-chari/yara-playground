# System Requirements

## YARA-OVE System Architecture

### Core Components Integration
YARA-OVE integrates multiple specialized components to deliver a comprehensive autonomous sailing simulation platform:

**Simulation Core**:
- **ROS Noetic**: Distributed robotics middleware for real-time communication
- **Gazebo Classic 11**: Advanced physics simulation with marine-specific plugins
- **Wave Physics Engine**: GPU-accelerated Gerstner wave implementation with Pierson-Moskowitz spectrum

**Marine Simulation Stack**:
- **Ocean Environment**: 1000m×1000m ocean world with realistic wave physics
- **Sailing Models**: EBoat (2.5m research vessel) and Fortune612 (0.99m RC boat)
- **Environmental Systems**: Dynamic wind modeling, GPS simulation, marine sensors

**Performance Architecture**:
- **300x Simulation Speedup**: Advanced temporal scaling for rapid experimentation
- **GPU Acceleration**: Vertex shader-based wave rendering at 30Hz update rate
- **Distributed Processing**: Multi-node ROS architecture supporting complex scenarios

### Value Propositions

**Research Excellence**:
- Bridge simulation-to-reality gap with validated E-Boat digital twin
- Accelerate sailing robotics research through proven 300x speedup capability
- Access comprehensive wave physics modeling with Gerstner wave implementation

**Educational Impact**:
- Structured learning progression from basic ocean simulation to autonomous sailing
- Hands-on experience with research-grade sailing robotics systems
- Integration with modern AI/ML frameworks for sailing algorithm development

**Industry Applications**:
- Autonomous shipping route optimization and testing
- Marine robotics system development and validation
- Ocean monitoring mission planning and execution

# System Requirements

## Operating System Compatibility

### Supported Systems
- **Ubuntu 20.04 LTS (Focal Fossa)** - ✅ **Recommended**
- Ubuntu 18.04 LTS (Bionic Beaver) - Limited support
- Ubuntu 22.04 LTS (Jammy Jellyfish) - Experimental compatibility

### Hardware Requirements

#### Minimum Requirements (Basic Ocean Simulation)
- **CPU**: 4 cores, 2.5 GHz (for ROS Noetic + Gazebo Classic 11)
- **RAM**: 8 GB (wave physics simulation requires additional memory)
- **Storage**: 15 GB free space (includes wave_gazebo packages and models)
- **Graphics**: OpenGL 3.3+ compatible GPU (required for wave rendering)

#### Recommended Requirements (Advanced Sailing Scenarios)
- **CPU**: 8+ cores, 3.0+ GHz (optimal for 300x speedup capability)
- **RAM**: 16+ GB (multi-boat scenarios and complex wave environments)
- **Storage**: 50+ GB SSD (fast I/O for large simulation datasets)
- **Graphics**: Dedicated GPU with 4+ GB VRAM (GPU-accelerated wave rendering at 30Hz)

#### High-Performance Requirements (Research Applications)
- **CPU**: 12+ cores, 3.5+ GHz (maximum simulation throughput)
- **RAM**: 32+ GB (large-scale fleet simulations and ML training)
- **Storage**: 100+ GB NVMe SSD (high-speed data processing)
- **Graphics**: RTX 3060+ or equivalent (optimal wave physics rendering)

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

# GPU info (for wave rendering)
lspci | grep VGA
glxinfo | grep "OpenGL version"
```

## YARA-OVE Performance Verification

### Ocean Simulation Readiness Check
```bash
# Test basic ocean world launch
roslaunch wave_gazebo ocean_world.launch gui:=false &
sleep 10

# Verify wave physics topics
rostopic list | grep wave
rostopic echo /ocean/wave_state --once

# Check simulation performance
rostopic hz /clock

# Cleanup
pkill -f roslaunch
```

### GPU Acceleration Validation
```bash
# Check OpenGL support for wave rendering
glxinfo | grep "direct rendering"
glxinfo | grep "OpenGL version"

# Verify GPU memory for wave physics
nvidia-smi  # For NVIDIA GPUs
gpu-manager --list  # For other GPUs

# Test wave rendering performance
roslaunch wave_gazebo ocean_world.launch &
rostopic hz /gazebo/visual_quality
```

### System Performance Benchmarks

**Basic Ocean Simulation (Target Performance)**:
- Wave rendering: 30 Hz sustained
- ROS node communication: <10ms latency
- Memory usage: <4 GB for single boat scenario

**Advanced Sailing Scenarios (Target Performance)**:
- Multi-boat simulation: 5+ boats at 20 Hz
- 300x speedup mode: Maintain >100x actual speedup
- Memory usage: <16 GB for complex fleet operations

**Research-Grade Performance (Target Performance)**:
- Large-scale simulations: 10+ boats with full sensor suites
- Real-time ML training: Concurrent RL policy updates
- Data throughput: >1 GB/min simulation data recording

### Troubleshooting Performance Issues

**Low Frame Rate Solutions**:
```bash
# Reduce wave resolution for better performance
roslaunch wave_gazebo ocean_world.launch wave_resolution:=25

# Disable advanced visual effects
roslaunch wave_gazebo ocean_world.launch visual_quality:=low

# Enable headless mode for maximum performance
roslaunch wave_gazebo ocean_world.launch gui:=false headless:=true
```

**Memory Optimization**:
```bash
# Limit simultaneous boats in scenarios
rosparam set /max_boats 3

# Reduce wave field complexity
rosparam set /wave_field_size 500

# Enable memory-efficient mode
rosparam set /memory_optimization true
```

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