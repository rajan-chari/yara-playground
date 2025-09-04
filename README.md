# Yara_OVE Experimental Playground

An experimental environment for exploring autonomous sailing robot simulation using the **Yara_OVE** (Yara Ocean Virtual Environment) framework.

## What is This Project?

This repository serves as a hands-on exploration platform for **Yara_OVE**, an autonomous sailing robot simulation developed by academic researchers. Yara_OVE provides detailed marine robotics simulation with realistic sailing dynamics and autonomous navigation challenges.

**This is not a production system** – it's a learning laboratory where you can experiment with sailing robotics technology.

## Implementation Status

This experimental playground provides an autonomous sailing robotics environment with wave physics simulation.

### Available Features
- **Environment Setup**: ROS Noetic, Gazebo 11, and Conda integration
- **Ocean Simulation**: Launch with [`roslaunch wave_gazebo ocean_world.launch`](yara-ove/wave_gazebo/launch/ocean_world.launch)
- **Wave Physics**: Gerstner wave implementation with Pierson-Moskowitz spectrum modeling
- **GPU-Accelerated Rendering**: Vertex shaders for ocean visualization
- **Multiple Scenarios**: Ocean waves, buoys, and sailing boat demonstrations

### Ocean Simulation
**Basic Usage**: Wave physics with modeling
```bash
cd ~/yara_ws && source devel/setup.bash
roslaunch wave_gazebo ocean_world.launch
```

**Sailing Boat Models**: EBoat (2.5m research vessel) and Fortune612 (0.99m RC boat)
**Scenarios**: Ocean with navigation buoys, WAMV demonstrations, sailing physics
**Research Integration**: ESailor reinforcement learning environments

## Yara_OVE Integration

The Yara_OVE framework is now included as a git submodule at [`yara-ove/`](yara-ove/) for direct experimentation. To set up the integrated framework, run the setup script:

```bash
./scripts/setup-yara-ove.sh
```


## 🗺️ Navigation Guide

### 🌊 **New to YARA-OVE Ocean Simulation?**
**Getting started with ocean simulation:**
1. **[Quick Start Guide](docs/getting-started/quick-start.md#yara-ove-ocean-simulation)** - Launch ocean world
2. **[YARA-OVE Scenarios](docs/usage/yara-ove-scenarios.md)** - Three complexity levels
3. **[Wave Physics Deep Dive](docs/resources/additional-resources.md#wave-physics-technical-deep-dive)** - Understanding the science
4. **[Learning Path](docs/resources/learning-path.md#yara-ove-ocean-simulation-progression)** - Structured progression

### 🤖 **AI/ML & Sailing Robotics Focus?**
**Autonomous sailing AI pathway:**
1. **[System Architecture](docs/getting-started/system-requirements.md#yara-ove-system-architecture)** - Platform overview
2. **[Sailing AI Environments](docs/usage/python-environments.md#yara-ove-sailing-robotics-environments)** - ML development setup
3. **[Advanced Workflows](docs/usage/advanced-workflows.md#yara-ove-sailing-robotics-research)** - Research approaches
4. **[ESailor RL Integration](docs/usage/python-environments.md#esailor-reinforcement-learning-environment)** - Sailing agent training

### ⛵ **Practical Sailing Models & Commands?**
**Working with sailing models:**
1. **[YARA-OVE Sailing Models](docs/usage/basic-commands.md#yara-ove-sailing-boat-models)** - EBoat & Fortune612 control
2. **[Gazebo Ocean Simulation](docs/usage/gazebo-simulation.md#yara-ove-ocean-simulation)** - Marine physics
3. **[Performance Benchmarking](docs/usage/python-environments.md#performance-benchmarking-environment)** - System optimization
4. **[Digital Twin Validation](docs/usage/python-environments.md#digital-twin-validation-environment)** - Model comparison

### 🔬 **Research & Development Focus?**
**Sailing robotics research:**
1. **[Value Propositions](docs/getting-started/system-requirements.md#value-propositions)** - Research overview
2. **[Sailing Algorithms](docs/resources/learning-path.md#phase-3-autonomous-sailing-research-hours-26-50)** - Research applications
3. **[Wave Physics Technical Details](docs/resources/additional-resources.md#wave-physics-technical-deep-dive)** - Gerstner wave mathematics
4. **[Multi-Boat Scenarios](docs/usage/basic-commands.md#multi-boat-scenarios)** - Fleet coordination

**🌊 Next Step**: Launch ocean simulation → [YARA-OVE Scenarios Guide](docs/usage/yara-ove-scenarios.md)

## About Yara_OVE

**Yara_OVE** is an ocean virtual environment for autonomous sailing robots, originally developed by researchers at Medialab FBoat. The system provides:

### 🌊 **Sailing Physics**
- **6-DOF sailing dynamics**: Freedom of movement with boat physics
- **300x real-time speedup**: Accelerated learning and experimentation
- **Wind/wave simulation**: Dynamic environmental conditions
- **Digital twin of E-Boat**: Based on real-world sailing vessel data

### 🤖 **Navigation Capabilities**
- **Reinforcement learning integration**: Train AI agents for sailing decisions
- **Tacking maneuvers**: Sailing behaviors
- **Upwind navigation strategies**: Sailing techniques
- **Real-time decision making**: Responses to changing conditions

### 🏗️ **Built on Technology**
- **Gazebo simulation engine**: 3D physics
- **ROS integration**: Robot Operating System compatibility
- **Extensible architecture**: Customizable for experiments

## Why Experiment with Sailing Robotics?

Autonomous sailing presents unique challenges that make it fascinating for researchers and enthusiasts:

- **Complex environmental dynamics**: Wind patterns, wave interactions, and weather changes
- **Multi-objective optimization**: Speed, safety, energy efficiency, and route planning
- **Non-linear control systems**: Sailing requires complex decision-making algorithms
- **Real-world applications**: Ocean monitoring, autonomous cargo, environmental research

## Prerequisites

This experimental environment builds on a solid robotics foundation. You'll need:

- **Ubuntu 20.04 LTS** with ROS Noetic installed
- **Gazebo Classic 11** for 3D simulation
- **Python environment** (Miniconda recommended) for algorithm development
- **Basic robotics knowledge** helpful but not required

*Note: This documentation will help you verify and set up the required ROS/Gazebo/Python stack.*

## Learning Opportunities

Experimenting with Yara_OVE opens doors to understanding:

### 🧠 **Artificial Intelligence & Machine Learning**
- Reinforcement learning for autonomous decision-making
- Multi-agent systems and fleet coordination
- Computer vision for environmental perception
- Predictive modeling for weather and sea conditions

### ⛵ **Marine Engineering & Physics**
- Sailing aerodynamics and hydrodynamics
- Boat stability and performance optimization
- Weather routing and tactical sailing
- Energy-efficient propulsion systems

### 🤖 **Advanced Robotics**
- Sensor fusion and state estimation
- Path planning in dynamic environments
- Robust control systems design
- Human-robot interaction for maritime operations

## Getting Started with Yara_OVE

### Phase 1: Environment Setup
1. **Verify Base Installation**: Confirm ROS Noetic and Gazebo are working → [System Requirements](docs/getting-started/system-requirements.md)
2. **Test Basic Functionality**: Run initial system checks → [Installation Verification](docs/installation/verification.md)

### Phase 2: Yara_OVE Installation
1. **Clone Yara_OVE Repository**: Get the simulation framework
2. **Build Sailing Packages**: Compile the specialized sailing physics
3. **Configure Environment**: Set up for autonomous sailing experiments

### Phase 3: First Experiments
1. **Launch Basic Simulation**: Start with simple sailing scenarios
2. **Explore Sailing Physics**: Understand wind/wave interactions
3. **Experiment with Control**: Try manual and autonomous sailing modes

## Original Academic Work

This experimental playground is built to explore the research behind **Yara_OVE**:

**Original Repository**: [github.com/medialab-fboat/Yara_OVE](https://github.com/medialab-fboat/Yara_OVE)

### Academic Contributors & Acknowledgments
We gratefully acknowledge the researchers at **Medialab FBoat** who developed this sailing simulation framework. Their work represents years of research in autonomous marine robotics, sailing physics modeling, and reinforcement learning applications.

*This experimental repository is an independent exploration of their publicly available research and should not be considered an official extension or endorsement.*

## Documentation Structure

The existing documentation structure supports your Yara_OVE experimentation:

### 🚀 Getting Started
- [**Quick Start Guide**](docs/getting-started/quick-start.md) - Foundation setup and YARA-OVE ocean launch
- [**System Requirements**](docs/getting-started/system-requirements.md) - Prerequisites and system architecture overview

### 📥 Installation Guides
- [**ROS Noetic Installation**](docs/installation/ros-noetic.md) - Robotics framework
- [**Gazebo Installation**](docs/installation/gazebo.md) - 3D simulation environment
- [**Miniconda Installation**](docs/installation/miniconda.md) - Python for development
- [**Installation Verification**](docs/installation/verification.md) - Verify installation

### 💻 Usage Guides
- [**Basic Commands**](docs/usage/basic-commands.md) - ROS commands and YARA-OVE sailing models
- [**Gazebo Simulation**](docs/usage/gazebo-simulation.md) - Running simulations and YARA-OVE ocean physics
- [**Python Environments**](docs/usage/python-environments.md) - ML/AI setup and sailing robotics environments
- [**Advanced Workflows**](docs/usage/advanced-workflows.md) - Complex experiments and sailing research
- [**YARA-OVE Scenarios**](docs/usage/yara-ove-scenarios.md) - Sailing complexity levels

### 🔧 Troubleshooting
- [**Common Issues**](docs/troubleshooting/common-issues.md) - General troubleshooting
- [**ROS-Specific Issues**](docs/troubleshooting/ros-specific.md) - ROS problems and solutions
- [**Gazebo-Specific Issues**](docs/troubleshooting/gazebo-specific.md) - Simulation environment issues
- [**Conda-Specific Issues**](docs/troubleshooting/conda-specific.md) - Python environment problems

### 📚 Resources
- [**Learning Path**](docs/resources/learning-path.md) - Structured progression and YARA-OVE ocean simulation
- [**Additional Resources**](docs/resources/additional-resources.md) - External resources and wave physics details

## Required Foundation Components

To run Yara_OVE experiments, you need these core components:

### 📦 Core Components
| Component | Version | Purpose |
|-----------|---------|---------|
| **Ubuntu** | 20.04 LTS | Stable Linux foundation |
| **ROS Noetic** | 1.15.x | Robot communication framework |
| **Gazebo Classic** | 11.15.1 | 3D sailing simulation |
| **Miniconda** | 25.7.0 | AI/ML development environment |

*Use the [installation guides](docs/installation/) to set up these components and [verification](docs/installation/verification.md) to confirm they work correctly.*

## Next Steps: Your Sailing Robotics Journey

### 🔰 **New to Sailing Robotics?**
1. **Learn the Basics**: Start with [Learning Path](docs/resources/learning-path.md)
2. **Understand Sailing Physics**: Study wind/wave dynamics
3. **Explore Simple Controls**: Begin with basic sailing maneuvers

### ⚡ **Ready to Experiment?**
1. **Clone Yara_OVE**: Get the sailing simulation framework
2. **Run First Simulation**: Launch a basic sailing scenario
3. **Modify Parameters**: Experiment with wind, waves, and boat configurations

### 🧠 **AI/ML Focus?**
1. **Study Reinforcement Learning**: Understand autonomous decision-making
2. **Experiment with Training**: Create sailing AI agents
3. **Analyze Performance**: Evaluate autonomous sailing strategies

### 🌊 **Marine Engineering Interest?**
1. **Study Sailing Dynamics**: Dive deep into sailing physics
2. **Optimize Boat Design**: Experiment with hull and sail configurations
3. **Weather Strategy**: Explore routing and tactical decisions

## Community & Support

### Learning Resources
- **Original Yara_OVE Repository**: [github.com/medialab-fboat/Yara_OVE](https://github.com/medialab-fboat/Yara_OVE)
- **ROS Community**: [ROS Answers](https://answers.ros.org) | [ROS Discourse](https://discourse.ros.org)
- **Sailing Robotics**: Research papers and academic resources

### Getting Help
1. **Check Documentation**: Start with guides above
2. **Review Troubleshooting**: Common issues and solutions
3. **Explore Examples**: Learn from working configurations
4. **Join Community**: Connect with sailing robotics enthusiasts

## Research & Academic Context

This experimental playground connects you to active research areas:
- **Autonomous marine vehicles** and ocean exploration
- **Reinforcement learning** for complex control systems
- **Environmental modeling** for marine conditions
- **Multi-objective optimization** in dynamic environments
- **Human-robot interaction** in maritime contexts

## License & Ethical Use

This experimental repository is for educational and research purposes. Please:
- **Respect the original research**: Credit the Yara_OVE academic contributors
- **Use responsibly**: This is experimental software, not for production marine systems
- **Share knowledge**: Contribute back to the sailing robotics community
- **Follow academic standards**: Cite original work in any publications

---

**🌊 Launch YARA-OVE ocean simulation → [YARA-OVE Ocean Launch](docs/getting-started/quick-start.md#yara-ove-ocean-simulation)**

**⛵ Explore sailing scenarios → [YARA-OVE Scenarios Guide](docs/usage/yara-ove-scenarios.md)**

**🤖 Sailing AI research → [YARA-OVE Learning Progression](docs/resources/learning-path.md#yara-ove-ocean-simulation-progression)**

**🔬 Wave physics details → [Wave Physics Technical Details](docs/resources/additional-resources.md#wave-physics-technical-deep-dive)**

---

<details>
<summary>📋 Experimental Roadmap</summary>

### Phase 1: Foundation Setup
- Install ROS Noetic + Gazebo + Python environment
- Verify basic simulation capabilities
- Review documentation structure

### Phase 2: Yara_OVE Integration
- Clone and build Yara_OVE framework
- Configure sailing physics simulation
- Run basic autonomous sailing demos

### Phase 3: Advanced Experiments
- Implement reinforcement learning integration
- Create custom sailing scenarios
- Perform performance analysis and optimization

### Phase 4: Research Applications
- Develop novel sailing strategies
- Experiment with multi-boat coordination
- Plan real-world validation

</details>