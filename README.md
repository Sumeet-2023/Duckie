# Autonomous Object-Following Duckiebot

This project implements an autonomous object-following robot with obstacle avoidance for the Duckiebot platform running on Jetson Nano 4GB.

## Features

- **Object Detection**: Color-based object detection using OpenCV
- **Object Following**: PID-controlled following behavior
- **Obstacle Avoidance**: Camera-based obstacle detection and avoidance
- **State Machine**: Intelligent switching between searching, following, and avoiding states
- **ROS Integration**: Full ROS integration with Duckietown message types
- **Real-time Processing**: Optimized for Jetson Nano performance

## System Requirements

- Duckiebot with Jetson Nano 4GB
- ROS Noetic (or compatible)
- Duckietown dt-core and dt-ros-commons
- OpenCV 4.x
- Python 3.8+

## Installation

1. **Clone this repository to your Duckiebot:**
   ```bash
   cd ~/catkin_ws/src
   git clone <your-repo-url> duckiebot_autonomous
   ```

2. **Install dependencies:**
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Make Python scripts executable:**
   ```bash
   chmod +x ~/catkin_ws/src/duckiebot_autonomous/src/*.py
   ```

## Usage

### Basic Operation

1. **Set your vehicle name:**
   ```bash
   export VEHICLE_NAME=your_duckiebot_name
   ```

2. **Launch the autonomous system:**
   ```bash
   roslaunch duckiebot_autonomous autonomous_following.launch veh:=$VEHICLE_NAME
   ```

### Advanced Configuration

You can customize the behavior by modifying parameters in the launch file or config file:

```bash
roslaunch duckiebot_autonomous autonomous_following.launch \
  veh:=$VEHICLE_NAME \
  target_object:=red_ball \
  max_linear_speed:=0.3 \
  target_distance:=1.5
```

### Available Target Objects

- `red_ball`: Red colored ball or object
- `blue_ball`: Blue colored ball or object  
- `green_ball`: Green colored ball or object
- `yellow_duckie`: Yellow Duckietown duck

## Node Architecture

### 1. Color Object Detection Node (`color_object_detection_node.py`)
- Detects colored objects using HSV color filtering
- Publishes object position and distance estimates
- Provides debug visualization

### 2. Obstacle Detection Node (`obstacle_detection_node.py`)
- Detects obstacles using edge detection
- Estimates obstacle distance
- Publishes obstacle warnings

### 3. Autonomous Controller Node (`autonomous_controller_node.py`)
- Main control logic with state machine
- PID control for smooth following
- Coordinates object following and obstacle avoidance

### 4. Object Following Node (`object_following_node.py`)
- Alternative integrated implementation
- Single node approach (can be used instead of modular approach)

## ROS Topics

### Published Topics
- `/{vehicle}/car_cmd_switch_node/cmd` - Control commands
- `/{vehicle}/object_detection/detections` - Object detection results
- `/{vehicle}/obstacle_detection/obstacle_detected` - Obstacle warnings
- `/{vehicle}/autonomous_controller/state` - Current robot state

### Subscribed Topics
- `/{vehicle}/camera_node/image/compressed` - Camera feed
- Various detection and control topics for coordination

## Configuration

Edit `config/autonomous_config.yaml` to adjust:
- Color detection ranges
- Control parameters
- Safety thresholds
- Camera settings

## Calibration

### Object Detection Calibration
1. Launch the system with debug visualization
2. Adjust HSV color ranges in the config file
3. Test with your specific objects under your lighting conditions

### Distance Estimation Calibration
1. Place objects at known distances
2. Adjust distance estimation parameters
3. Test following behavior at different distances

### Control Tuning
1. Start with conservative gains
2. Gradually increase until smooth following is achieved
3. Test obstacle avoidance scenarios

## Safety Features

- **Speed Limiting**: Maximum speeds are enforced
- **Obstacle Detection**: Automatic stopping when obstacles detected
- **Timeout Handling**: Safe behavior when object is lost
- **Conservative Control**: Reduced speed during sharp turns

## Troubleshooting

### Diagnostic Tools

I've included comprehensive error handling and diagnostic tools:

**Quick Health Check:**
```bash
python3 diagnostic_tool.py --health-check
```

**Full Diagnostic Report:**
```bash
python3 diagnostic_tool.py --full-report
```

**Emergency Recovery:**
```bash
./error_recovery.sh
```

### Common Issues

1. **Object not detected:**
   - Check lighting conditions
   - Adjust HSV color ranges
   - Verify camera is working
   - Run: `python3 diagnostic_tool.py --health-check`

2. **Erratic following behavior:**
   - Reduce control gains
   - Check for image processing delays
   - Verify object detection consistency
   - Check CPU usage: `./error_recovery.sh` → option 4

3. **Robot doesn't move:**
   - Check ROS topic connections
   - Verify vehicle name parameter
   - Ensure motors are enabled
   - Emergency stop: `./error_recovery.sh` → option 1

4. **Nodes crashing:**
   - Check error logs: view generated diagnostic report
   - Clean restart: `./error_recovery.sh` → option 3
   - Check system resources

### Error Reporting

When asking for help, always include:
1. Full diagnostic report: `python3 diagnostic_tool.py --full-report`
2. Recent error messages from ROS logs
3. System specifications and ROS version
4. Steps to reproduce the issue

### Debug Tools

- View debug images: `rqt_image_view`
- Monitor topics: `rostopic echo /topic_name`
- Check node status: `rosnode list`
- View error logs: `rostopic echo /rosout | grep ERROR`

## Performance Optimization for Jetson Nano

- Image processing is optimized for 640x480 resolution
- Debug image publishing is optional
- Adjustable processing rates for different performance requirements

## Future Enhancements

- Deep learning-based object detection
- LIDAR integration for better obstacle detection
- Multi-object tracking
- Path planning algorithms
- SLAM integration

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test thoroughly on hardware
4. Submit a pull request

## License

MIT License - see LICENSE file for details

## Acknowledgments

- Duckietown community
- ROS community
- OpenCV contributors