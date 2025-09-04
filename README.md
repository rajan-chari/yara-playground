# Yara_OVE Experimental Playground

An experimental environment for exploring autonomous sailing robot simulation using the **Yara_OVE** (Yara Ocean Virtual Environment) framework.

## What is This Project?

This repository serves as a hands-on exploration platform for **Yara_OVE**, an autonomous sailing robot simulation developed by academic researchers. Yara_OVE provides detailed marine robotics simulation with realistic sailing dynamics and autonomous navigation challenges.

**This is not a production system** – it's a learning laboratory where you can experiment with sailing robotics technology.

## About Yara_OVE

**Yara_OVE** is an ocean virtual environment designed specifically for autonomous sailing robots, originally developed by researchers at Medialab FBoat. The system offers:

### 🌊 **Advanced Sailing Physics**
- **6-DOF sailing dynamics**: Complete freedom of movement with realistic boat physics
- **300x real-time speedup**: Accelerated learning and experimentation
- **Specialized wind/wave simulation**: Dynamic environmental conditions
- **Digital twin of E-Boat**: Based on real-world sailing vessel data

### 🤖 **Autonomous Navigation Capabilities**
- **Reinforcement learning integration**: Train AI agents for sailing decisions
- **Autonomous tacking maneuvers**: Complex sailing behaviors
- **Upwind navigation strategies**: Advanced sailing techniques
- **Real-time decision making**: Adaptive responses to changing conditions

### 🏗️ **Built on Proven Technology**
- **Gazebo simulation engine**: Industry-standard 3D physics
- **ROS integration**: Robot Operating System compatibility
- **Extensible architecture**: Customize for your experiments

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
- [**Quick Start Guide**](docs/getting-started/quick-start.md) - Foundation setup verification
- [**System Requirements**](docs/getting-started/system-requirements.md) - Prerequisites for sailing simulation

### 📥 Installation Guides
- [**ROS Noetic Installation**](docs/installation/ros-noetic.md) - Core robotics framework
- [**Gazebo Installation**](docs/installation/gazebo.md) - 3D sailing simulation environment
- [**Miniconda Installation**](docs/installation/miniconda.md) - Python for algorithm development
- [**Installation Verification**](docs/installation/verification.md) - Confirm your foundation is solid

### 💻 Usage Guides
- [**Basic Commands**](docs/usage/basic-commands.md) - Essential ROS commands for sailing
- [**Gazebo Simulation**](docs/usage/gazebo-simulation.md) - Running sailing simulations
- [**Python Environments**](docs/usage/python-environments.md) - ML/AI development setup
- [**Advanced Workflows**](docs/usage/advanced-workflows.md) - Complex sailing experiments

### 🔧 Troubleshooting
- [**Common Issues**](docs/troubleshooting/common-issues.md) - General troubleshooting
- [**ROS-Specific Issues**](docs/troubleshooting/ros-specific.md) - ROS problems and solutions
- [**Gazebo-Specific Issues**](docs/troubleshooting/gazebo-specific.md) - Simulation environment issues
- [**Conda-Specific Issues**](docs/troubleshooting/conda-specific.md) - Python environment problems

### 📚 Resources
- [**Learning Path**](docs/resources/learning-path.md) - Structured approach to sailing robotics
- [**Additional Resources**](docs/resources/additional-resources.md) - External sailing robotics resources

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

**🌊 Ready to explore autonomous sailing? Your experimental journey begins with understanding the foundation → [Quick Start Guide](docs/getting-started/quick-start.md)**

**⛵ Fascinated by sailing robotics? Dive into the original research → [Yara_OVE Repository](https://github.com/medialab-fboat/Yara_OVE)**

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