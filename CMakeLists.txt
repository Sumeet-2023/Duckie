cmake_minimum_required(VERSION 3.0.2)
project(duckiebot_autonomous)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  duckietown_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    cv_bridge 
    image_transport
    duckietown_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  src/object_following_node.py
  src/obstacle_detection_node.py
  src/color_object_detection_node.py
  src/autonomous_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

#############
## Testing ##
#############

## Add gtest based cpp tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # No C++ tests for now, only Python nodes
endif()