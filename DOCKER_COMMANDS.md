# Duckiebot Docker Quick Commands
# ================================

# First, make the Docker setup script executable and run it:
chmod +x docker_setup_duckie.sh
./docker_setup_duckie.sh

# After setup, to launch the autonomous system:
cd /code/catkin_ws
source devel/setup.bash
export VEHICLE_NAME=pinkduckie
roslaunch duckiebot_autonomous autonomous_following.launch veh:=$VEHICLE_NAME

# To run diagnostics:
cd /code/catkin_ws/src/duckiebot_autonomous
python3 diagnostic_tool.py --health-check

# To check ROS topics:
rostopic list | grep pinkduckie

# To view camera feed:
rostopic echo /pinkduckie/camera_node/image/compressed --max-count=1

# To monitor object detection:
rostopic echo /pinkduckie/object_detection/detections

# To stop all nodes:
rosnode kill -a

# Emergency recovery:
cd /code/catkin_ws/src/duckiebot_autonomous
./error_recovery.sh