#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from cv_bridge import CvBridge
import math
import traceback

class ObjectFollowingNode:
    def __init__(self):
        rospy.init_node('object_following_node')
        
        # Parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot')
        self.target_area = rospy.get_param('~target_area', 5000)  # Target object area in pixels
        self.min_area = rospy.get_param('~min_area', 1000)
        self.max_area = rospy.get_param('~max_area', 15000)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.3)  # meters
        
        # Control parameters
        self.linear_gain = rospy.get_param('~linear_gain', 0.5)
        self.angular_gain = rospy.get_param('~angular_gain', 2.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.4)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
        
        # State variables
        self.bridge = CvBridge()
        self.object_detected = False
        self.object_center_x = 0
        self.object_area = 0
        self.obstacle_detected = False
        self.image_width = 640
        self.image_height = 480
        
        # HSV color range for object detection (example: red ball)
        self.lower_hsv = np.array([0, 100, 100])
        self.upper_hsv = np.array([10, 255, 255])
        
        # Publishers
        self.cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd', 
            WheelsCmdStamped, 
            queue_size=1
        )
        
        self.twist_pub = rospy.Publisher(
            f'/{self.vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )
        
        # Subscribers
        self.image_sub = rospy.Subscribe(
            f'/{self.vehicle_name}/camera_node/image/compressed',
            CompressedImage,
            self.image_callback
        )
        
        # For obstacle detection using camera (simple distance estimation)
        self.depth_estimation_enabled = True
        
        rospy.loginfo("Object Following Node initialized")
    
    def image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr("Failed to decode image - received None")
                return
            
            self.image_width = cv_image.shape[1]
            self.image_height = cv_image.shape[0]
            
            # Process image for object detection
            self.detect_object(cv_image)
            
            # Process image for obstacle detection
            self.detect_obstacles(cv_image)
            
            # Generate control commands
            self.generate_control_commands()
            
        except Exception as e:
            rospy.logerr(f"ERROR in image_callback: {str(e)}")
            rospy.logerr(f"Message size: {len(msg.data) if hasattr(msg, 'data') else 'No data'}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            # Publish emergency stop
            try:
                cmd = Twist2DStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.v = 0.0
                cmd.omega = 0.0
                self.twist_pub.publish(cmd)
            except:
                rospy.logerr("Failed to publish emergency stop!")
    
    def detect_object(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for target object color
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.object_detected = False
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_area:
                self.object_detected = True
                self.object_area = area
                
                # Get center of object
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    self.object_center_x = int(M["m10"] / M["m00"])
                    
                rospy.logdebug(f"Object detected: area={area}, center_x={self.object_center_x}")
    
    def detect_obstacles(self, image):
        # Simple obstacle detection using image analysis
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Focus on lower portion of image (ground level)
        roi_height = int(self.image_height * 0.3)
        roi = gray[self.image_height - roi_height:, :]
        
        # Use edge detection to find obstacles
        edges = cv2.Canny(roi, 50, 150)
        
        # Count edge pixels in center region (potential obstacle)
        center_width = int(self.image_width * 0.4)
        center_start = int(self.image_width * 0.3)
        center_edges = edges[:, center_start:center_start + center_width]
        
        edge_density = np.sum(center_edges) / (center_width * roi_height)
        
        # If too many edges in center, consider it an obstacle
        self.obstacle_detected = edge_density > 0.1
        
        if self.obstacle_detected:
            rospy.logwarn("Obstacle detected!")
    
    def generate_control_commands(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        
        if self.obstacle_detected:
            # Stop or turn away from obstacle
            cmd.v = 0.0
            cmd.omega = 1.0  # Turn right to avoid obstacle
            rospy.logwarn("Avoiding obstacle")
            
        elif self.object_detected:
            # Calculate error from image center
            image_center = self.image_width / 2
            error_x = self.object_center_x - image_center
            
            # Normalize error
            normalized_error = error_x / image_center
            
            # Calculate area error (distance control)
            area_error = (self.target_area - self.object_area) / self.target_area
            
            # Generate control commands
            linear_speed = self.linear_gain * area_error
            angular_speed = -self.angular_gain * normalized_error
            
            # Clamp speeds
            linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
            
            cmd.v = linear_speed
            cmd.omega = angular_speed
            
            rospy.loginfo(f"Following object: v={linear_speed:.2f}, omega={angular_speed:.2f}")
            
        else:
            # No object detected, stop or search
            cmd.v = 0.0
            cmd.omega = 0.5  # Slow rotation to search for object
            rospy.loginfo("Searching for object")
        
        # Publish command
        self.twist_pub.publish(cmd)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectFollowingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass