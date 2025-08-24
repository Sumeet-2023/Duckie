#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import math

class AutonomousControllerNode:
    def __init__(self):
        rospy.init_node('autonomous_controller_node', anonymous=True)
        
        # Get parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'pinkduckie')
        
        # Control parameters
        self.kp_linear = rospy.get_param('~kp_linear', 0.5)
        self.ki_linear = rospy.get_param('~ki_linear', 0.1)
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.0)
        
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.4)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
        
        self.target_distance = rospy.get_param('~target_distance', 1.0)
        self.obstacle_avoidance_distance = rospy.get_param('~obstacle_avoidance_distance', 0.5)
        
        # State management
        self.state = "SEARCHING"  # SEARCHING, FOLLOWING, AVOIDING
        self.last_detection = None
        self.last_detection_time = rospy.Time.now()
        self.detection_timeout = 2.0  # seconds
        
        # Control variables
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        
        # Image parameters (typical camera resolution)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width // 2
        
        # Obstacle detection
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            Twist,
            queue_size=1
        )
        
        # Subscribers
        self.detection_sub = rospy.Subscriber(
            f'/{self.vehicle_name}/object_detection/detection_data',
            Float32MultiArray,
            self.detection_callback
        )
        
        self.obstacle_sub = rospy.Subscriber(
            f'/{self.vehicle_name}/obstacle_detection/obstacle_distance',
            Float32MultiArray,
            self.obstacle_callback
        )
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Autonomous Controller Node initialized")

    def detection_callback(self, msg):
        """Handle detection data from color detection node"""
        # Enhanced validation - expect exactly 6 elements: [x, y, area, confidence, width, height]
        if len(msg.data) != 6:
            rospy.logwarn(f"Invalid object detection data length: {len(msg.data)}, expected 6")
            return

        try:
            center_x = msg.data[0]
            center_y = msg.data[1] 
            area = msg.data[2]
            confidence = msg.data[3]
            width = msg.data[4]
            height = msg.data[5]
            
            # Apply thresholds for valid detection
            if area > 100 and confidence > 0.3:
                self.last_detection = {
                    'center_x': center_x,
                    'center_y': center_y,
                    'area': area,
                    'confidence': confidence,
                    'width': width,
                    'height': height,
                    'timestamp': rospy.Time.now()
                }
                
                self.last_detection_time = rospy.Time.now()
                
                # Calculate control based on detection
                self.calculate_control()
                rospy.logdebug(f"Object detected at ({center_x:.1f}, {center_y:.1f}), area: {area:.1f}, confidence: {confidence:.2f}")
            else:
                rospy.logdebug(f"Detection filtered out - area: {area:.1f}, confidence: {confidence:.2f}")
                
        except Exception as e:
            rospy.logerr(f"Error processing detection: {e}")

    def obstacle_callback(self, msg):
        """Handle obstacle detection data"""
        try:
            if len(msg.data) > 0:
                self.min_obstacle_distance = min(msg.data)
                self.obstacle_detected = self.min_obstacle_distance < self.obstacle_avoidance_distance
                
                if self.obstacle_detected and self.state == "FOLLOWING":
                    self.state = "AVOIDING"
                    rospy.loginfo(f"State: AVOIDING - Obstacle at {self.min_obstacle_distance:.2f}m")
                    
        except Exception as e:
            rospy.logerr(f"Error processing obstacle data: {e}")

    def calculate_control(self):
        """Calculate control commands based on current detection"""
        if not self.last_detection:
            return
            
        # Switch to following state if we have a good detection
        if self.state == "SEARCHING":
            self.state = "FOLLOWING"
            rospy.loginfo("State: FOLLOWING - Target acquired")

    def control_loop(self, event):
        """Main control loop"""
        current_time = rospy.Time.now()
        
        # Check for detection timeout
        if (current_time - self.last_detection_time).to_sec() > self.detection_timeout:
            if self.state != "SEARCHING":
                self.state = "SEARCHING"
                rospy.loginfo("State: SEARCHING - Lost target")
                self.last_detection = None
        
        # Execute behavior based on current state
        if self.state == "FOLLOWING" and self.last_detection:
            self.following_behavior()
        elif self.state == "AVOIDING":
            self.avoidance_behavior()
        else:
            self.search_behavior()

    def following_behavior(self):
        """Control behavior when following an object"""
        if not self.last_detection:
            return
            
        center_x = self.last_detection['center_x']
        area = self.last_detection['area']
        
        # Calculate angular error (steering)
        x_error = center_x - self.image_center_x
        angular_error = -x_error / (self.image_width / 2)  # Normalize to [-1, 1]
        
        self.angular_error_integral += angular_error
        angular_velocity = (self.kp_angular * angular_error + 
                          self.ki_angular * self.angular_error_integral)
        angular_velocity = max(-self.max_angular_speed, 
                             min(self.max_angular_speed, angular_velocity))
        
        # Calculate linear velocity based on object size (area)
        # Larger area = closer object = slower speed
        target_area = 5000  # Desired object area for target distance
        area_error = (target_area - area) / target_area
        
        self.linear_error_integral += area_error
        linear_velocity = (self.kp_linear * area_error + 
                         self.ki_linear * self.linear_error_integral)
        linear_velocity = max(-self.max_linear_speed, 
                            min(self.max_linear_speed, linear_velocity))
        
        # Safety check - don't get too close
        if area > 8000:  # Very close
            linear_velocity = min(0, linear_velocity)
        
        self.publish_velocity(linear_velocity, angular_velocity)
        
        rospy.logdebug(f"Following: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}")

    def search_behavior(self):
        """Control behavior when searching for object"""
        # Slowly rotate to search for the object
        linear_velocity = 0.0
        angular_velocity = 0.3  # Slow rotation
        
        self.publish_velocity(linear_velocity, angular_velocity)
        rospy.loginfo("State: SEARCHING - Looking for target object")

    def avoidance_behavior(self):
        """Control behavior when avoiding obstacles"""
        # Simple obstacle avoidance - back up and turn
        linear_velocity = -0.2
        angular_velocity = 1.0
        
        self.publish_velocity(linear_velocity, angular_velocity)
        rospy.loginfo("State: AVOIDING - Obstacle detected")
        
        # Return to searching after brief avoidance
        if self.min_obstacle_distance > self.obstacle_avoidance_distance * 1.2:
            self.state = "SEARCHING"
            self.obstacle_detected = False

    def publish_velocity(self, linear_vel, angular_vel):
        """Publish velocity commands to robot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        controller = AutonomousControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass