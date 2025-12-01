---
id: capstone-overview
title: "13.1 Capstone Project: The Autonomous Humanoid"
sidebar_label: "13.1 Project Overview"
sidebar_position: 1
---

# Capstone Project: The Autonomous Humanoid

## Project Overview

Welcome to the culmination of your 13-week journey into Physical AI and Humanoid Robotics! In this capstone project, you will integrate everything you've learned to build **"The Autonomous Humanoid"**—a fully functional robotic system capable of understanding natural language instructions, perceiving its environment, navigating bipedally, and manipulating objects.

### Project Scope

**Goal**: Build a simulated humanoid robot that can execute the command:

> **"Walk to the kitchen table, pick up the red mug, and bring it to me."**

This deceptively simple task requires:
1. **Natural Language Understanding**: Parse and interpret the command
2. **Vision**: Detect the table, localize the mug, identify the human
3. **Navigation**: Plan and execute bipedal walking with obstacle avoidance
4. **Manipulation**: Grasp the mug and maintain stable grip while walking
5. **Human-Robot Interaction**: Approach the human and perform safe handoff

---

## Learning Outcomes

By completing this capstone, you will:

1. **Integrate** ROS 2, Gazebo Classic/Isaac Sim, and OpenAI VLA models into a cohesive system
2. **Implement** a complete Physical AI stack: perception → cognition → action
3. **Deploy** a Vision-Language-Action model for task-level control
4. **Debug** complex multi-component robotic systems
5. **Evaluate** system performance using quantitative metrics
6. **Document** your work for reproducibility and future development

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     USER INTERFACE                            │
│              Natural Language Task Input                      │
└────────────────────────┬─────────────────────────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────────┐
│              VLA MODEL (OpenAI-based)                         │
│    • Language Understanding (GPT-4)                           │
│    • Vision Processing (CLIP)                                 │
│    • Action Generation (Policy Network)                       │
└────────────────────────┬─────────────────────────────────────┘
                         │
        ┌────────────────┴──────────────────┐
        │                                   │
┌───────▼─────────┐              ┌──────────▼────────┐
│   PERCEPTION    │              │    PLANNING       │
│  • RGB-D Camera │              │ • Task Sequencer  │
│  • Object Det.  │              │ • Motion Planner  │
│  • SLAM         │              │ • Grasp Planner   │
│  • Human Det.   │              │ • MPC Controller  │
└───────┬─────────┘              └──────────┬────────┘
        │                                   │
        └────────────────┬──────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────────┐
│                  CONTROL LAYER (ROS 2)                        │
│  • Whole-Body Controller                                      │
│  • Bipedal Locomotion                                         │
│  • Manipulation Controller                                    │
│  • Balance & Stability                                        │
└────────────────────────┬─────────────────────────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────────┐
│            SIMULATION (Gazebo / Isaac Sim)                    │
│  • Humanoid Robot (Unitree H1 or custom)                      │
│  • Kitchen Environment                                        │
│  • Physics Engine                                             │
│  • Sensor Simulation                                          │
└──────────────────────────────────────────────────────────────┘
```

**Figure 13.1**: System architecture for "The Autonomous Humanoid." Bidirectional arrows indicate feedback loops essential for reactive control.

---

## Technology Stack

### Core Frameworks
- **ROS 2 Humble**: Middleware for component communication
- **Gazebo Classic 11** or **NVIDIA Isaac Sim 2023.1.0**: Physics simulation
- **OpenAI API**: VLA model backend (GPT-4 + CLIP + custom policy)
- **PyTorch 2.0+**: Neural network inference

### Perception
- **cv_bridge**: ROS-OpenCV integration
- **YOLO v8**: Real-time object detection
- **Depth Image to Laser Scan**: 2D navigation from 3D data
- **Nav2**: Navigation stack for path planning

### Control
- **MoveIt 2**: Manipulation planning
- **ros2_control**: Hardware abstraction layer
- **Whole-Body Controller**: Custom QP-based controller (provided)

### VLA Integration
- **OpenAI Agents SDK**: Task orchestration
- **LangChain**: Prompt engineering and chain-of-thought reasoning
- **Qdrant**: Vector database for few-shot examples

---

## Project Phases

### Phase 1: Environment Setup (Days 1-2)
- Set up simulation environment
- Import humanoid robot model (Unitree H1 URDF)
- Create kitchen scene with table, mug, and obstacles
- Verify sensor data streams (camera, IMU, joint states)

### Phase 2: Perception Pipeline (Days 3-5)
- Implement RGB-D camera processing
- Integrate YOLO v8 for object detection ("mug," "table," "human")
- Add semantic segmentation for scene understanding
- Implement SLAM for localization

### Phase 3: VLA Model Integration (Days 6-9)
- Set up OpenAI API access
- Design prompt templates for task decomposition
- Implement VLA node that:
  - Receives natural language input
  - Analyzes current scene (visual + proprioceptive)
  - Generates action sequence
- Add few-shot examples to Qdrant for improved performance

### Phase 4: Motion Planning & Control (Days 10-14)
- Implement bipedal walking controller (Week 7 content)
- Integrate MoveIt 2 for arm motion planning
- Add whole-body controller for coordinated locomotion + manipulation
- Tune PID/MPC parameters for stability

### Phase 5: Task Execution (Days 15-18)
- Implement task state machine:
  1. Navigate to table
  2. Detect and localize mug
  3. Plan grasp approach
  4. Execute grasp
  5. Walk to human
  6. Perform handoff
- Add error handling and recovery behaviors

### Phase 6: Testing & Refinement (Days 19-21)
- Run full task 50 times, log success rate
- Identify failure modes (lost balance, missed grasp, etc.)
- Refine controllers and VLA prompts
- Optimize for speed and robustness

---

## Detailed Component Specifications

### 1. VLA Model Node

**File**: `vla_task_planner.py`

```python
#!/usr/bin/env python3
"""
VLA-based task planner for autonomous humanoid.
Integrates OpenAI GPT-4 for language understanding and task sequencing.

Citations:
    [1] Brohan et al., "RT-2: Vision-Language-Action Models," arXiv:2307.15818, 2023
    [2] Driess et al., "PaLM-E: An Embodied Multimodal Language Model," arXiv:2303.03378, 2023
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import openai
import json
import base64
from typing import List, Dict, Optional

class VLATaskPlanner(Node):
    """
    High-level task planner using Vision-Language-Action model.

    Subscribed Topics:
        /task_command (std_msgs/String): Natural language task input
        /camera/rgb/image_raw (sensor_msgs/Image): RGB camera feed
        /camera/depth/image_raw (sensor_msgs/Image): Depth image
        /robot/state (String): Current robot state JSON

    Published Topics:
        /task/action_sequence (String): JSON array of actions
        /task/status (String): Task execution status
    """

    def __init__(self):
        super().__init__('vla_task_planner')

        # OpenAI API setup
        self.declare_parameter('openai_api_key', '')
        api_key = self.get_parameter('openai_api_key').value
        openai.api_key = api_key if api_key else os.getenv('OPENAI_API_KEY')

        # Publishers
        self.action_pub = self.create_publisher(
            String, '/task/action_sequence', 10
        )
        self.status_pub = self.create_publisher(
            String, '/task/status', 10
        )

        # Subscribers
        self.task_sub = self.create_subscription(
            String, '/task_command', self.task_callback, 10
        )
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, '/robot/state', self.state_callback, 10
        )

        # State variables
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.robot_state = {}

        self.get_logger().info('VLA Task Planner initialized')

    def rgb_callback(self, msg: Image):
        """Store latest RGB image."""
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def state_callback(self, msg: String):
        """Store robot state."""
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse robot state')

    def task_callback(self, msg: String):
        """
        Process incoming task command using VLA model.

        Args:
            msg: Natural language task description
        """
        task = msg.data
        self.get_logger().info(f'Received task: {task}')

        # Update status
        self.publish_status('processing_task')

        # Generate action sequence using VLA
        try:
            action_sequence = self.generate_action_sequence(task)
            self.publish_actions(action_sequence)
            self.publish_status('task_planned')
        except Exception as e:
            self.get_logger().error(f'Task planning failed: {e}')
            self.publish_status('task_failed')

    def generate_action_sequence(self, task: str) -> List[Dict]:
        """
        Use VLA model to decompose task into action primitives.

        Args:
            task: Natural language task description

        Returns:
            List of action dictionaries with type, parameters, and expected duration
        """
        # Encode current visual observation
        image_b64 = self.encode_image(self.latest_rgb) if self.latest_rgb is not None else None

        # Construct VLA prompt with visual grounding
        system_prompt = """You are a humanoid robot control system. Given a task description
        and current visual observation, decompose the task into a sequence of primitive actions.

        Available actions:
        1. navigate_to(target: str, x: float, y: float) - Walk to a location
        2. detect_object(object_name: str) - Find and localize an object
        3. grasp_object(object_name: str, approach: str) - Pick up an object
        4. release_object() - Release currently grasped object
        5. look_at(target: str) - Orient head/cameras toward target
        6. wait(duration: float) - Pause for specified seconds

        Respond in JSON format:
        {
          "reasoning": "step-by-step thought process",
          "actions": [
            {"type": "action_name", "params": {...}, "duration_estimate": 5.0},
            ...
          ]
        }
        """

        user_prompt = f"""Task: {task}

        Current robot state:
        - Position: ({self.robot_state.get('x', 0.0)}, {self.robot_state.get('y', 0.0)})
        - Battery: {self.robot_state.get('battery', 100)}%
        - Grasped object: {self.robot_state.get('grasped_object', 'none')}

        Visual observation: [see attached image]

        Please plan the action sequence to accomplish this task."""

        # Call OpenAI API with vision
        messages = [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_prompt},
                ]
            }
        ]

        if image_b64:
            messages[1]["content"].append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{image_b64}"
                }
            })

        response = openai.ChatCompletion.create(
            model="gpt-4-vision-preview",
            messages=messages,
            max_tokens=1000,
            temperature=0.3  # Lower temperature for more deterministic planning
        )

        # Parse response
        result = json.loads(response.choices[0].message.content)
        self.get_logger().info(f"VLA reasoning: {result['reasoning']}")

        return result['actions']

    def encode_image(self, cv_image) -> str:
        """Encode OpenCV image to base64 for API transmission."""
        import cv2
        _, buffer = cv2.imencode('.jpg', cv_image)
        return base64.b64encode(buffer).decode('utf-8')

    def publish_actions(self, actions: List[Dict]):
        """Publish action sequence to execution layer."""
        msg = String()
        msg.data = json.dumps(actions)
        self.action_pub.publish(msg)
        self.get_logger().info(f'Published {len(actions)} actions')

    def publish_status(self, status: str):
        """Publish task status update."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLATaskPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Action Executor Node

**File**: `action_executor.py`

```python
#!/usr/bin/env python3
"""
Executes primitive actions planned by VLA model.
Interfaces with navigation, manipulation, and whole-body control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
import json
from typing import Dict, List


class ActionExecutor(Node):
    """
    Executes low-level actions from high-level VLA plan.

    Subscribed Topics:
        /task/action_sequence (String): JSON array of actions

    Published Topics:
        /task/execution_status (String): Real-time execution updates
    """

    def __init__(self):
        super().__init__('action_executor')

        # Action clients for subsystems
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

        # Subscribers
        self.action_sub = self.create_subscription(
            String, '/task/action_sequence', self.execute_sequence, 10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/task/execution_status', 10
        )

        self.get_logger().info('Action Executor ready')

    def execute_sequence(self, msg: String):
        """
        Execute action sequence sequentially.

        Args:
            msg: JSON array of actions
        """
        try:
            actions = json.loads(msg.data)
            self.get_logger().info(f'Executing {len(actions)} actions')

            for idx, action in enumerate(actions):
                self.publish_status(f'executing_action_{idx+1}/{len(actions)}')
                success = self.execute_action(action)

                if not success:
                    self.get_logger().error(f'Action {idx+1} failed: {action["type"]}')
                    self.publish_status('execution_failed')
                    return

            self.publish_status('execution_complete')

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self.publish_status('execution_error')

    def execute_action(self, action: Dict) -> bool:
        """
        Execute a single primitive action.

        Args:
            action: Action dictionary with type and params

        Returns:
            True if successful, False otherwise
        """
        action_type = action['type']
        params = action.get('params', {})

        if action_type == 'navigate_to':
            return self.navigate_to(params)
        elif action_type == 'detect_object':
            return self.detect_object(params)
        elif action_type == 'grasp_object':
            return self.grasp_object(params)
        elif action_type == 'release_object':
            return self.release_object()
        elif action_type == 'look_at':
            return self.look_at(params)
        elif action_type == 'wait':
            return self.wait(params)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def navigate_to(self, params: Dict) -> bool:
        """Navigate to target location using Nav2."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = params.get('x', 0.0)
        goal_pose.pose.position.y = params.get('y', 0.0)
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to ({params["x"]}, {params["y"]})')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().status == 4  # SUCCEEDED

    def grasp_object(self, params: Dict) -> bool:
        """Execute grasping motion using MoveIt 2."""
        object_name = params.get('object_name', 'unknown')
        self.get_logger().info(f'Grasping {object_name}')

        # This would interface with MoveIt 2 for grasp planning
        # Simplified for demonstration:
        # 1. Get object pose from perception
        # 2. Plan approach trajectory
        # 3. Execute arm motion
        # 4. Close gripper

        # Placeholder - actual implementation in full code
        return True

    def publish_status(self, status: str):
        """Publish execution status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Evaluation Metrics

Your capstone will be evaluated on:

### 1. Task Success Rate (40%)
- **Metric**: Percentage of successful task completions out of 50 trials
- **Criteria**:
  - **Excellent (90-100%)**: ≥45/50 successful
  - **Good (75-89%)**: 38-44/50 successful
  - **Satisfactory (60-74%)**: 30-37/50 successful
  - **Needs Improvement (<60%)**: <30/50 successful

### 2. Execution Time (15%)
- **Metric**: Average time to complete task
- **Criteria**:
  - **Excellent**: <60 seconds
  - **Good**: 60-90 seconds
  - **Satisfactory**: 90-120 seconds

### 3. Code Quality (20%)
- Documentation, modularity, adherence to ROS 2 best practices
- Proper error handling and logging
- Unit tests for critical components

### 4. System Integration (15%)
- Successful integration of all components
- Proper use of ROS 2 interfaces
- VLA model effectively guides behavior

### 5. Innovation (10%)
- Novel approaches to challenging problems
- Additional features beyond requirements
- Creative use of VLA capabilities

---

## Submission Requirements

### Code Repository
- GitHub/GitLab repository with:
  - All source code
  - Launch files
  - Configuration files (YAML, URDF)
  - README with setup instructions
  - `requirements.txt` for Python dependencies

### Documentation (10-15 pages PDF)
1. **Introduction**: Project goals and approach
2. **System Architecture**: Detailed component diagram
3. **Implementation**: Key design decisions, algorithms used
4. **Evaluation**: Test results, failure analysis, metrics
5. **Conclusion**: Lessons learned, future work
6. **References**: Properly cited sources (IEEE format)

### Video Demonstration (3-5 minutes)
- Show successful task execution
- Narrate system behavior at each step
- Include at least one failure mode and recovery
- Upload to YouTube/Vimeo, include link in README

### Presentation (10 minutes)
- Live demo of system (if possible)
- Walk through architecture and key components
- Discuss challenges and how you overcame them
- Q&A with instructor/peers

---

## Timeline

| Week | Milestones |
|------|------------|
| 1 | Environment setup, robot model imported |
| 2 | Perception pipeline functional |
| 3 | VLA model integrated, basic task decomposition working |
| 4 | Motion planning and control tuned |
| 5 | Full task execution, testing and refinement |
| 6 | Documentation, video, and final submission |

---

## Provided Resources

### Starter Code
- `humanoid_ws/`: ROS 2 workspace template
- `models/unitree_h1/`: URDF and meshes for Unitree H1 robot
- `worlds/kitchen.world`: Gazebo world with table, mug, obstacles
- `config/controllers.yaml`: Pre-configured controllers
- `launch/bringup.launch.py`: Launch file to start all nodes

### Example VLA Prompts
- Few-shot examples for common tasks (stored in Qdrant)
- Prompt templates for task decomposition
- Vision encoding utilities

### Testing Suite
- Automated test scenarios
- Metrics collection scripts
- Visualization tools (RViz configurations)

---

## Tips for Success

1. **Start Simple**: Get each component working independently before integration
2. **Use Simulation**: Iterate quickly in Gazebo before attempting real hardware
3. **Debug Systematically**: Use `ros2 topic echo`, `rqt_graph`, and `rviz2`
4. **Version Control**: Commit often, use branches for experiments
5. **Ask for Help**: Use office hours, forums, and the AI tutor
6. **Document as You Go**: Don't wait until the end to write documentation
7. **Test Edge Cases**: What happens if the mug is knocked over? Battery dies mid-task?

---

## Common Pitfalls

- **Overfitting VLA Prompts**: Test with variations of task descriptions
- **Ignoring Failure Modes**: Real robots fail; plan for graceful degradation
- **Poor Calibration**: Ensure camera-robot transforms are accurate
- **Timing Issues**: ROS 2 is asynchronous; handle message latency properly
- **Neglecting Safety**: Even in simulation, validate collision avoidance

---

## Next Steps

Proceed to the detailed implementation guides:

- [13.2 System Integration](./system-integration): Connecting all components
- [13.3 VLA Deployment](./vla-deployment): Advanced VLA techniques
- [13.4 Testing & Validation](./testing-validation): Comprehensive testing strategies

---

**Copyright Notice**: © 2025 PIAIC. Licensed under CC BY-SA 4.0.

## References

[1] A. Brohan et al., "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control," arXiv:2307.15818, 2023.

[2] D. Driess et al., "PaLM-E: An Embodied Multimodal Language Model," arXiv:2303.03378, 2023.

[3] OpenAI, "GPT-4 Technical Report," arXiv:2303.08774, 2023.

[4] ROS 2 Control Documentation, https://control.ros.org/, 2024.

[5] Nav2 Documentation, https://navigation.ros.org/, 2024.

[6] MoveIt 2 Documentation, https://moveit.picknik.ai/, 2024.

