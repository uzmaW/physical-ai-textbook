---
id: glossary
title: Glossary of Terms
sidebar_label: Glossary
sidebar_position: 100
---

# Glossary of Terms

Comprehensive glossary of technical terms used throughout the textbook. Terms are alphabetized with definitions, Urdu translations, and cross-references.

---

## A

**Actuator** (ایکچویٹر)
: A mechanical device that converts energy (electrical, hydraulic, pneumatic) into motion. In humanoid robots, actuators drive joint movements.
: *See also*: Motor, Servo, Quasi-Direct Drive

**ASIMO** (اسیمو)
: Advanced Step in Innovative Mobility. Honda's humanoid robot (2000-2018) that demonstrated bipedal walking, stair climbing, and human interaction capabilities.

**Atlas** (ایٹلس)
: Boston Dynamics' hydraulic humanoid robot known for dynamic locomotion, backflips, and parkour capabilities. Transitioned to electric actuation in 2024.

---

## B

**Balance** (توازن)
: The ability of a robot to maintain stability and prevent falling. Achieved through control algorithms that manage the center of mass relative to the support polygon.
: *See also*: ZMP, Capture Point

**Biomimicry** (حیاتیاتی نقل)
: Design approach that mimics biological systems. Humanoid robots use biomimicry to replicate human locomotion, manipulation, and sensing.

**Bipedal Locomotion** (دو پیروں پر چلنا)
: Walking or running on two legs. Inherently unstable and requires sophisticated control algorithms.
: *See also*: Gait, ZMP

---

## C

**Capstone Project** (اختتامی منصوبہ)
: Final integrative project where students apply all learned concepts. In this course: building "The Autonomous Humanoid."

**Capture Point** (قبضہ نقطہ)
: A point on the ground where a robot can step to come to a complete stop. Used in push recovery and balance control [1].
: *Reference*: Pratt et al., 2006

**Center of Mass (CoM)** (مرکز کمیت)
: The average position of all mass in a system. Critical for balance and stability analysis.
: *Formula*: $\text{CoM} = \frac{1}{m}\sum_{i} m_i \mathbf{r}_i$

**CLIP** (کلِپ)
: Contrastive Language-Image Pre-training. OpenAI's vision-language model that learns to match images with text descriptions [2].
: *See also*: VLA Models, Foundation Models

**colcon** (کولکون)
: Build tool for ROS 2 workspaces. Successor to catkin (ROS 1).
: *Usage*: `colcon build --symlink-install`

---

## D

**Degrees of Freedom (DOF)** (آزادی کی ڈگریاں)
: Number of independent parameters defining a robot's configuration. Typical humanoid: 30-50 DOF.
: *Example*: Human arm has 7 DOF (shoulder 3, elbow 1, wrist 3)

**Depth Sensing** (گہرائی کا احساس)
: Measuring distance to objects using stereo cameras, structured light, or Time-of-Flight sensors.
: *See also*: RGB-D Camera, LiDAR

**Docusaurus** (ڈوکیوساورس)
: Static site generator used for this textbook. Built on React, supports MDX and internationalization.

**Dynamics** (حرکیات)
: Study of forces and torques that cause motion. Governed by Newton's laws and Euler-Lagrange equations.
: *Equation*: $\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)$
: *See also*: Kinematics, Inverse Dynamics

---

## E

**Embodied AI** (مجسم مصنوعی ذہانت)
: AI systems that learn through physical interaction with the world. Contrasts with purely digital AI.
: *See also*: Physical AI, Sensorimotor Learning

**End-Effector** (اختتامی متاثر کن)
: The device at the end of a robotic arm (gripper, hand, tool). The "hand" of the robot.

---

## F

**Figure AI** (فگر اے آئی)
: Company developing general-purpose humanoid robots. Figure 01 deployed in BMW manufacturing (2024).

**Force Control** (قوت کنٹرول)
: Regulating the forces a robot applies during contact. Essential for delicate manipulation and human-safe interaction.
: *See also*: Impedance Control, Admittance Control

**Forward Kinematics (FK)** (آگے کی حرکیات)
: Computing end-effector position/orientation from joint angles.
: *Equation*: $\mathbf{x} = f(\mathbf{q})$ where $\mathbf{q}$ = joint angles, $\mathbf{x}$ = pose

**Foundation Models** (بنیادی ماڈلز)
: Large neural networks pre-trained on vast datasets (e.g., GPT-4, CLIP). Can be adapted to many downstream tasks with minimal additional training.

---

## G

**Gait** (چال)
: Pattern of limb movements during walking. Humanoid gaits: walking, running, climbing stairs.
: *See also*: Gait Generation, Trajectory Optimization

**Gazebo** (گزیبو)
: Open-source 3D robotics simulator. Provides physics simulation, sensor models, and ROS integration.

**GelSight** (جیل سائٹ)
: High-resolution tactile sensor using camera and deformable gel. Detects contact geometry and forces [3].

**GPT-4** (جی پی ٹی-4)
: OpenAI's large language model (175B+ parameters). Used in VLA models for natural language understanding.

**Grasp Planning** (پکڑ کی منصوبہ بندی)
: Computing optimal finger positions and forces to securely hold an object.
: *Metric*: Grasp quality (epsilon, volume, force closure)

---

## H

**Haptic Feedback** (لمسی رائے)
: Tactile sensations provided to a user (e.g., in teleoperation). Force reflection, vibration.

**Humanoid Robot** (انسان نما روبوٹ)
: Robot with human-like form: head, torso, two arms, two legs. Designed to operate in human environments.
: *Examples*: ASIMO, Atlas, Optimus, Unitree H1

---

## I

**Impedance Control** (مزاحمت کنٹرول)
: Control strategy that regulates the dynamic relationship between force and motion. Allows compliant interaction.
: *Equation*: $F = M\ddot{x} + B\dot{x} + Kx$
: *See also*: Force Control, Admittance Control

**IMU (Inertial Measurement Unit)** (نامیاتی پیمائش اکائی)
: Sensor measuring acceleration (accelerometer) and angular velocity (gyroscope). Essential for balance and state estimation.
: *Typical Specs*: 6-axis (3-axis accel + 3-axis gyro) or 9-axis (+ magnetometer)

**Inverse Kinematics (IK)** (الٹی حرکیات)
: Computing joint angles needed to achieve a desired end-effector pose. Opposite of FK.
: *Equation*: $\mathbf{q} = f^{-1}(\mathbf{x})$
: *Methods*: Analytical, Jacobian-based, optimization

**Isaac Sim** (آئزک سم)
: NVIDIA's robotics simulator built on Omniverse. Photorealistic rendering, GPU-accelerated physics, ROS 2 integration.

---

## J

**Jacobian Matrix** (جیکوبین میٹرکس)
: Matrix relating joint velocities to end-effector velocities.
: *Equation*: $\dot{\mathbf{x}} = J(\mathbf{q})\dot{\mathbf{q}}$
: *Uses*: IK, singularity analysis, force mapping

---

## K

**Kalman Filter** (کالمان فلٹر)
: Optimal state estimator for linear systems with Gaussian noise. Used for sensor fusion.
: *Variants*: Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF)

**Kinematics** (حرکیات)
: Study of motion without considering forces. Positions, velocities, accelerations.
: *See also*: Forward Kinematics, Inverse Kinematics, Dynamics

---

## L

**LiDAR** (لائڈار)
: Light Detection and Ranging. Laser-based distance sensor providing 3D point clouds.
: *Range*: 0.1m - 100m+, depending on model
: *See also*: Depth Sensing, SLAM

**Locomotion** (حرکت)
: Self-propelled movement. For humanoids: walking, running, climbing.
: *See also*: Bipedal Locomotion, Gait

---

## M

**Manipulation** (ہیرا پھیری)
: Using arms/hands to interact with objects. Includes grasping, placing, pushing, tool use.
: *See also*: Grasp Planning, Force Control

**MoveIt** (موو اِٹ)
: Motion planning framework for ROS. Provides IK solvers, collision checking, trajectory optimization.
: *Version*: MoveIt 2 for ROS 2 Humble

**MPC (Model Predictive Control)** (ماڈل پیشن گوئی کنٹرول)
: Control strategy that optimizes future behavior over a prediction horizon.
: *Horizon*: 0.3-1.0 seconds typical for humanoids
: *Advantages*: Handles constraints, anticipates disturbances

---

## N

**Nav2** (نیو دو)
: Navigation stack for ROS 2. Path planning, obstacle avoidance, recovery behaviors.
: *Algorithms*: Dijkstra, A*, TEB (Timed Elastic Band)

**Neural Network** (عصبی نیٹ ورک)
: Machine learning model inspired by biological neurons. Foundation of deep learning.
: *Types*: CNN (vision), RNN (sequences), Transformer (attention-based)

---

## O

**OpenAI** (اوپن اے آئی)
: AI research company. Developed GPT-4, CLIP, and contributed to robotics through VLA models.

**OpenVLA** (اوپن وی ایل اے)
: Open-source Vision-Language-Action model (7B parameters). Accessible alternative to proprietary VLA models.

**Optimus** (آپٹیمس)
: Tesla's humanoid robot. Gen 2 (2024) features 11-DOF hands, $20k target cost, 0.6 m/s walking speed.

---

## P

**PaLM-E** (پام-ای)
: Pathways Language Model - Embodied. Google's 562B parameter multimodal model integrating vision, language, and sensor data [4].

**Perception** (ادراک)
: Sensing and interpreting the environment. Vision, depth, tactile, proprioception.
: *Pipeline*: Sensor data → Processing → Feature extraction → Scene understanding

**Physical AI** (جسمانی مصنوعی ذہانت)
: AI systems that perceive, reason about, and interact with the physical world through robotic embodiment [5].
: *Components*: Perception, Cognition, Action

**PPO (Proximal Policy Optimization)** (قربت پالیسی اصلاح)
: Deep reinforcement learning algorithm. Stable, sample-efficient. Widely used in robotics [6].

**Proprioception** (خود احساس)
: Sensing internal state (joint angles, velocities, torques). Analogous to human proprioception.
: *Sensors*: Encoders, IMU, force/torque sensors

---

## Q

**Qdrant** (کیوڈرینٹ)
: Vector database for AI applications. Used in this textbook for RAG (Retrieval-Augmented Generation).
: *Features*: Semantic search, filtering, high-performance indexing

**QP (Quadratic Programming)** (چوکور پروگرامنگ)
: Optimization technique minimizing quadratic objective subject to linear constraints. Used in whole-body control.

**Quasi-Direct Drive (QDD)** (نیم براہ راست ڈرائیو)
: Low gear ratio actuators (6:1 to 9:1). High backdrivability, torque sensing via current. Used in Unitree H1.

---

## R

**RAG (Retrieval-Augmented Generation)** (بازیافت بڑھی ہوئی نسل)
: AI technique combining retrieval from a knowledge base with generative models. Powers this textbook's AI tutor.

**RGB-D Camera** (آر جی بی-ڈی کیمرہ)
: Camera providing both color (RGB) and depth (D) information.
: *Examples*: Intel RealSense, Azure Kinect, ZED

**RL (Reinforcement Learning)** (تقویتی سیکھنا)
: Machine learning paradigm where agents learn by trial-and-error, receiving rewards for desired behaviors.
: *Key Algorithms*: PPO, SAC, TRPO
: *See also*: Deep RL, Sim-to-Real

**ROS 2** (روز دو)
: Robot Operating System 2. Middleware for building robot applications. Successor to ROS 1.
: *Features*: DDS communication, real-time support, better security
: *Current Version*: Humble Hawksbill (2022-2027)

**RT-2 (Robotics Transformer 2)** (روبوٹکس ٹرانسفارمر دو)
: Google DeepMind's VLA model (55B parameters). Transfers web knowledge to robotic control [7].

---

## S

**SAC (Soft Actor-Critic)** (نرم اداکار نقاد)
: Off-policy deep RL algorithm for continuous action spaces. Maximum entropy formulation encourages exploration.

**Sensor Fusion** (سینسر فیوژن)
: Combining data from multiple sensors to improve state estimation.
: *Techniques*: Kalman filtering, particle filters, complementary filters

**Sim-to-Real** (سم-سے-حقیقی)
: Transferring policies trained in simulation to real robots.
: *Techniques*: Domain randomization, system identification, fine-tuning

**SLAM (Simultaneous Localization and Mapping)** (بیک وقت مقام اور نقشہ سازی)
: Building a map of an unknown environment while simultaneously localizing within it.
: *Algorithms*: ORB-SLAM2, Cartographer, LSD-SLAM

---

## T

**Tactile Sensor** (لمسی سینسر)
: Sensor detecting contact, pressure, texture. Analogous to human skin.
: *Technologies*: Capacitive, resistive, optical (GelSight)

**Teleoperation** (دور سے آپریشن)
: Remote control of a robot by a human operator. Used for data collection and difficult tasks.
: *Interfaces*: VR controllers, motion capture, exoskeletons

**TF (Transform)** (تبدیل)
: ROS 2 library managing coordinate frame transformations. Essential for multi-sensor systems.
: *Usage*: `tf2_ros` package

**Trajectory** (رفتار کا راستہ)
: Path through configuration space over time. Specifies positions, velocities, accelerations.
: *Generation*: Interpolation (polynomial, spline), optimization

---

## U

**Unitree H1** (یونی ٹری ایچ وَن)
: Chinese humanoid robot. World's fastest (3.3 m/s), lightweight (47kg), open-source development platform.
: *Cost*: ~$90,000

**URDF (Unified Robot Description Format)** (متحدہ روبوٹ تفصیل فارمیٹ)
: XML format for describing robot kinematics and dynamics. Standard in ROS.
: *Elements*: links, joints, sensors, materials

---

## V

**VLA (Vision-Language-Action)** (نظر-زبان-عمل)
: Foundation models integrating vision, natural language, and robot control. Enable language-driven robot behavior.
: *Examples*: RT-2, PaLM-E, OpenVLA

---

## W

**Whole-Body Control** (پورے جسم کا کنٹرول)
: Coordinating all DOF of a robot simultaneously. Uses optimization (QP) to achieve multiple objectives (balance, reach target, avoid limits).
: *Hierarchy*: Balance > CoM tracking > Task execution > Joint limits

---

## X

**xacro** (زیکرو)
: XML macro language for URDF. Enables parameterization and modularization of robot descriptions.
: *Usage*: `xacro robot.urdf.xacro > robot.urdf`

---

## Y

**YOLO (You Only Look Once)** (یولو)
: Real-time object detection algorithm. YOLO v8 (2023) achieves 30-60 FPS on GPU.
: *Use Case*: Detecting objects (mugs, tables, humans) in humanoid perception

---

## Z

**Zero-Moment Point (ZMP)** (صفر لمحہ نقطہ)
: Point on the ground where net moment of contact forces is zero. Classic stability criterion for bipedal robots [8].
: *Condition*: ZMP inside support polygon → stable

**Zustand** (زوسٹینڈ)
: Lightweight state management library for React. Used in this textbook's frontend for user profiles and preferences.

---

## Acronyms

| Acronym | Full Form | Urdu |
|---------|-----------|------|
| AI | Artificial Intelligence | مصنوعی ذہانت |
| CoM | Center of Mass | مرکز کمیت |
| DOF | Degrees of Freedom | آزادی کی ڈگریاں |
| FK | Forward Kinematics | آگے کی حرکیات |
| GPU | Graphics Processing Unit | گرافکس پروسیسنگ یونٹ |
| IK | Inverse Kinematics | الٹی حرکیات |
| IMU | Inertial Measurement Unit | نامیاتی پیمائش اکائی |
| LiDAR | Light Detection and Ranging | لائٹ ڈیٹیکشن اینڈ رینجنگ |
| MPC | Model Predictive Control | ماڈل پیشن گوئی کنٹرول |
| PIAIC | Presidential Initiative for AI & Computing | صدارتی اقدام برائے اے آئی |
| QP | Quadratic Programming | چوکور پروگرامنگ |
| RAG | Retrieval-Augmented Generation | بازیافت بڑھی ہوئی نسل |
| RGB-D | Red-Green-Blue + Depth | رنگین-گہرائی |
| RL | Reinforcement Learning | تقویتی سیکھنا |
| ROS | Robot Operating System | روبوٹ آپریٹنگ سسٹم |
| SLAM | Simultaneous Localization and Mapping | بیک وقت مقام و نقشہ سازی |
| URDF | Unified Robot Description Format | متحدہ روبوٹ تفصیل فارمیٹ |
| VLA | Vision-Language-Action | نظر-زبان-عمل |
| ZMP | Zero-Moment Point | صفر لمحہ نقطہ |

---

## Cross-References

For deeper understanding, see related chapters:
- **Physical AI Concepts** → Chapter 1.1
- **Humanoid Hardware** → Chapter 3
- **Kinematics & Dynamics** → Chapter 4
- **Perception Systems** → Chapters 5-6
- **Locomotion & Control** → Chapters 7-8
- **Manipulation** → Chapter 9
- **Deep RL** → Chapter 10
- **VLA Models** → Chapter 11
- **Applications** → Chapter 12
- **Capstone Integration** → Chapter 13

---

## References

[1] J. Pratt et al., "Capture Point: A Step toward Humanoid Push Recovery," 2006.

[2] A. Radford et al., "Learning Transferable Visual Models From Natural Language Supervision," 2021.

[3] W. Yuan et al., "GelSight: High-Resolution Robot Tactile Sensors," 2017.

[4] D. Driess et al., "PaLM-E: An Embodied Multimodal Language Model," 2023.

[5] Arthur D. Little, "BLUE SHIFT Physical AI," 2025.

[6] J. Schulman et al., "Proximal Policy Optimization Algorithms," 2017.

[7] A. Brohan et al., "RT-2: Vision-Language-Action Models," 2023.

[8] M. Vukobratović and B. Borovac, "Zero-Moment Point—Thirty Five Years," 2004.

