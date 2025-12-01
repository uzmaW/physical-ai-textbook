# Physical AI & Humanoid Robotics: Embodied Intelligence in Practice

[![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/)
[![Code License: MIT](https://img.shields.io/badge/Code%20License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-blue)](https://www.python.org/)

## 📚 About This Textbook

An open-access, comprehensive university textbook on Physical AI and Humanoid Robotics, designed for a 13-week capstone course. This resource integrates cutting-edge research with hands-on practice, culminating in a capstone project where students build an autonomous humanoid robot powered by Vision-Language-Action (VLA) models.

### Key Features

- **🎓 Academic Rigor**: Peer-reviewed citations, IEEE format references
- **💻 Executable Code**: All examples run in ROS 2 Humble and NVIDIA Isaac Sim
- **🤖 RAG-Powered AI Tutor**: Integrated chatbox for instant help
- **🌍 Multilingual**: English and Urdu translations
- **🎯 Practical Focus**: Weekly labs with real-world applications
- **🚀 Modern Stack**: VLA models, deep RL, sim-to-real transfer

---

## 📖 Textbook Structure

### **Week 1-2: Foundations of Physical AI**
- [1.1 Introduction to Physical AI](./docs/week-01.mdx)
- 1.2 Embodied Intelligence and Biomimicry
- 1.3 History and Evolution of Humanoid Robots
- [**Lab 1**: ROS 2 Environment Setup](./docs/week-01.mdx)

### **Week 3: Humanoid Robot Hardware**
- 3.1 Mechanical Design Principles
- 3.2 Actuators and Sensors
- 3.3 Power Systems and Energy Management
- **Lab 2**: URDF Modeling and RViz Visualization

### **Week 4: Kinematics & Dynamics**
- 4.1 Forward Kinematics
- 4.2 Inverse Kinematics (Analytical & Numerical)
- 4.3 Robot Dynamics and Equation of Motion
- **Lab 3**: Kinematics Solver Implementation

### **Week 5: Perception Systems**
- 5.1 Vision Systems (RGB, Depth, Stereo)
- 5.2 Depth Sensing Technologies
- 5.3 Tactile and Force Sensing
- **Lab 4**: Camera Integration and Object Detection

### **Week 6: Sensor Fusion & SLAM**
- 6.1 Multi-Sensor Fusion (Kalman Filters, Particle Filters)
- 6.2 SLAM Algorithms (ORB-SLAM2, Cartographer)
- 6.3 Localization and Mapping
- **Lab 5**: SLAM Implementation with Nav2

### **Week 7: Bipedal Locomotion**
- 7.1 Walking Theory and Gait Patterns
- 7.2 Balance and Stability (ZMP, Capture Point)
- 7.3 Gait Generation and Trajectory Optimization
- **Lab 6**: Bipedal Walking Controller

### **Week 8: Motion Planning**
- 8.1 Path Planning Algorithms (RRT, RRT*, A*)
- 8.2 Trajectory Optimization (TOPP, CHOMP)
- 8.3 Collision Avoidance and Safety
- **Lab 7**: Navigation Stack with Obstacle Avoidance

### **Week 9: Manipulation & Grasping**
- 9.1 Manipulation Theory
- 9.2 Grasp Planning and Force Closure
- 9.3 Impedance Control and Compliance
- **Lab 8**: MoveIt 2 Manipulation Pipeline

### **Week 10: Deep Reinforcement Learning**
- 10.1 RL Fundamentals for Robotics
- 10.2 Policy Gradient Methods (PPO, SAC, TRPO)
- 10.3 Sim-to-Real Transfer Techniques
- **Lab 9**: Training RL Policies in Isaac Sim

### **Week 11: Vision-Language-Action Models**
- 11.1 Introduction to VLA Models
- 11.2 Foundation Models (RT-2, PaLM-E, OpenVLA)
- 11.3 Natural Language Task Specification
- **Lab 10**: VLA Model Integration with OpenAI

### **Week 12: Applications & Ethics**
- 12.1 Industrial and Manufacturing Applications
- 12.2 Service Robotics and Healthcare
- 12.3 Ethical Considerations and Safety
- **Lab 11**: Real-World Application Demo

### **Week 13: Capstone Project**
- [**13.1 Project Overview**: The Autonomous Humanoid](./docs/week-13/capstone-overview.md)
- 13.2 System Integration
- 13.3 VLA Deployment and Testing
- 13.4 Validation and Performance Evaluation

---

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Node.js**: 18+ (for Docusaurus)
- **GPU**: NVIDIA (for Isaac Sim, optional)

### Installation

```bash
# Clone the repository
git clone https://github.com/piaic/humanoid-ai-textbook.git
cd humanoid-ai-textbook

# Install Node.js dependencies
npm install

# Start development server
npm start

# Build for production
npm run build

# The textbook will be available at http://localhost:3000
```

### ROS 2 Setup (for Code Labs)

```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Clone code examples
git clone https://github.com/piaic/humanoid-ai-code.git

# Build workspace
cd ~/humanoid_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 📚 Learning Path

### For Students
1. Read chapters sequentially (each week builds on previous concepts)
2. Complete all code labs before progressing
3. Attempt exercises (basic → intermediate → advanced)
4. Start planning capstone project from Week 1
5. Use the AI Tutor (right sidebar) for clarification

### For Instructors
- Each week designed for 6-8 hours of student engagement
- Labs can be completed in 2-hour sessions
- Exercise grading: Basic (40%), Intermediate (40%), Advanced (20%)
- Capstone can be individual or team-based (2-3 students)
- Rubrics and assessment tools provided

### For Self-Learners
- Follow the suggested timeline or go at your own pace
- Join community forums for support
- Share your capstone project with the community
- Contribute improvements back to the textbook

---

## 🛠️ Technology Stack

### Core Frameworks
- **ROS 2 Humble**: Robot middleware
- **Gazebo Classic 11** / **Isaac Sim 2023.1.0**: Simulation
- **PyTorch 2.0+**: Deep learning
- **OpenAI API**: VLA models

### Frontend (Textbook Platform)
- **Docusaurus v3**: Static site generator
- **Tailwind CSS**: Styling
- **Zustand**: State management
- **KaTeX**: Math rendering

### Backend (RAG & Personalization)
- **Neon Postgres**: User profiles, progress tracking
- **Qdrant**: Vector database for RAG
- **OpenAI Agents SDK**: AI tutor orchestration
- **FastAPI**: Backend API

### Robotics Libraries
- **Nav2**: Navigation stack
- **MoveIt 2**: Manipulation planning
- **ros2_control**: Hardware abstraction
- **cv_bridge**: ROS-OpenCV integration

---

## 🎯 Capstone Project: The Autonomous Humanoid

The culmination of the course is building a complete autonomous humanoid system that can:

**Task**: *"Walk to the kitchen table, pick up the red mug, and bring it to me."*

**Components**:
- Natural language understanding (VLA model)
- Visual perception (YOLO v8, depth sensing)
- Bipedal locomotion (whole-body controller)
- Manipulation (MoveIt 2, grasp planning)
- Human-robot interaction (handoff, safety)

**Evaluation**: Success rate (50 trials), execution time, code quality, innovation

See [Capstone Overview](./docs/week-13/capstone-overview.md) for full details.

---

## 📊 Integra ted Sources

This textbook synthesizes insights from:

1. **Jim Rauf (OLLI 2025)**: "Exploring Humanoid Robots 8"
   - Historical context, robot platforms, market analysis

2. **China Unicom (2025)**: "Applications and Development Prospects of Humanoid Robots"
   - Technical specifications, industry deployments, market forecasts

3. **Arthur D. Little (2025)**: "BLUE SHIFT Physical AI"
   - VLA models, AI integration, future trends

4. **ACM Survey**: "Humanoid Robots and Humanoid AI" (DOI: 10.1145/3770574)
   - Academic foundations, algorithms, research challenges

All sources properly cited in IEEE format throughout the textbook.

---

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](./CONTRIBUTING.md) for guidelines.

### Areas for Contribution
- Additional exercises and solutions
- Code examples for new robot platforms
- Urdu translation improvements
- Bug fixes and typo corrections
- New lab modules (e.g., soft robotics, swarm robotics)

### How to Contribute
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-lab`)
3. Make your changes
4. Run tests and linters
5. Submit a Pull Request with detailed description

---

## 📜 License

- **Textbook Content**: [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
  - You are free to share and adapt with attribution and share-alike

- **Code Examples**: [MIT License](https://opensource.org/licenses/MIT)
  - Maximum reusability for research and commercial use

---

## 🏆 Acknowledgments

### Authors
- **PIAIC Faculty**: Physical AI curriculum development
- **Industry Advisors**: Tesla, Boston Dynamics, NVIDIA
- **Research Contributors**: ACM Robotics community

### Special Thanks
- ROS 2 maintainers and community
- NVIDIA Isaac Sim team
- OpenAI for VLA model access
- Open-source robotics community

### Citations
This textbook would not be possible without:
- Brohan et al. (RT-2 paper)
- Driess et al. (PaLM-E paper)
- Kajita et al. (Humanoid robotics foundations)
- And hundreds of other researchers—see [Bibliography](./docs/bibliography.md)

---

## 📞 Support

- **AI Tutor**: Use the integrated chatbox (right sidebar)
- **GitHub Issues**: [Report bugs or suggest features](https://github.com/piaic/humanoid-ai-textbook/issues)
- **Discussions**: [Community forum](https://github.com/piaic/humanoid-ai-textbook/discussions)
- **Email**: humanoid-ai@piaic.org

---

## 🗺️ Roadmap

### v1.0 (Current)
- ✅ 13 weeks of content
- ✅ Executable code labs
- ✅ Capstone project
- ✅ English version complete

### v1.1 (Q2 2025)
- 🔄 Complete Urdu translations
- 🔄 Video lectures for each chapter
- 🔄 Interactive simulations (WebGL)

### v2.0 (Q3 2025)
- 📋 Additional robot platforms (Unitree G1, Figure 02)
- 📋 Advanced topics (swarm robotics, soft robots)
- 📋 Industry case studies
- 📋 Certification program

---

## 📈 Citation

If you use this textbook in your research or teaching, please cite:

```bibtex
@book{piaic2025physical,
  title={Physical AI \& Humanoid Robotics: Embodied Intelligence in Practice},
  author={PIAIC Faculty},
  year={2025},
  publisher={Presidential Initiative for Artificial Intelligence \& Computing},
  url={https://humanoid-ai-textbook.piaic.org},
  note={Licensed under CC BY-SA 4.0}
}
```

---

## ⭐ Star Us!

If you find this textbook useful, please star the repository to help others discover it!

---

**Copyright © 2025 PIAIC • Presidential Initiative for Artificial Intelligence & Computing**

