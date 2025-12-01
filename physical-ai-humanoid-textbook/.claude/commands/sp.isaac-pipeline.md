# Isaac ROS Perception Pipeline Generator

Generate NVIDIA Isaac ROS 2 perception pipeline configurations.

## Capabilities

- Create YAML configs for Isaac ROS nodes
- Generate launch files for perception pipelines
- Configure DNN inference (YOLO, SegNet, PoseNet)
- Set up camera calibration parameters
- Create multi-sensor fusion pipelines

## Usage

User specifies:
- Perception task: object detection, segmentation, SLAM
- Sensor: camera (RGB-D, stereo), LiDAR
- Model: YOLO v5/v8, SegNet, cuvSLAM

## Output

1. YAML configuration file
2. Launch file (Python)
3. README with Isaac ROS installation instructions

## Example

**Input**: "Object detection with RGB camera using YOLO v8"

**Outputs**:
- `yolo_detection.yaml` - DNN inference config
- `perception.launch.py` - Launch file
- Configured for GPU acceleration

## Quality

- ✅ Isaac ROS 2.0+ compatible
- ✅ GPU optimization settings
- ✅ Proper topic remapping
- ✅ Performance tuning (batch size, TensorRT)
