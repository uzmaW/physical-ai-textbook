# Gazebo Scene Generator

Generate complete Gazebo simulation environments with URDF, SDF, and launch files.

## Capabilities

- Create SDF world files with physics, lighting, objects
- Generate URDF robot models with Gazebo plugins
- Create Python launch files for easy startup
- Add sensor plugins (camera, LiDAR, IMU)
- Configure contact dynamics and materials

## Usage

User describes environment:
- Type: kitchen, warehouse, outdoor, office
- Objects: tables, chairs, obstacles, graspable items
- Robot: humanoid, quadruped, mobile manipulator
- Sensors needed: RGB camera, depth, LiDAR

## Output Files

1. **World SDF** (`world_name.sdf`):
   - Physics engine configuration
   - Ground plane
   - Lighting (sun, ambient)
   - Static models (furniture)
   - Dynamic models (graspable objects)

2. **Robot URDF** (if custom robot):
   - Links and joints
   - Gazebo plugins (ros2_control, sensors)
   - Material properties (friction, inertia)

3. **Launch File** (`gazebo_bringup.launch.py`):
   - Start Gazebo with world
   - Spawn robot at specified pose
   - Start robot_state_publisher
   - Launch RViz with config

## Example

**Input**: "Create kitchen environment with table, red mug, and Unitree H1 humanoid"

**Outputs**:
- `kitchen_world.sdf` - Complete kitchen with physics
- `spawn_h1.launch.py` - Launch file
- README with usage instructions

## Quality Standards

- ✅ Valid SDF 1.6+ syntax
- ✅ Realistic physics parameters
- ✅ Proper collision geometries
- ✅ ROS 2 Humble compatible plugins
- ✅ Comments explaining parameters

