#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import traceback

class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node('obstacle_detection_node')
        
        # Parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot')
        self.obstacle_distance_threshold = rospy.get_param('~obstacle_distance_threshold', 0.5)
        self.detection_height_ratio = rospy.get_param('~detection_height_ratio', 0.4)
        self.detection_width_ratio = rospy.get_param('~detection_width_ratio', 0.6)
        
        # State variables
        self.bridge = CvBridge()
        self.image_width = 640
        self.image_height = 480
        
        # Publishers
        self.obstacle_pub = rospy.Publisher(
            f'/{self.vehicle_name}/obstacle_detection/obstacle_detected',
            Bool,
            queue_size=1
        )
        
        self.distance_pub = rospy.Publisher(
            f'/{self.vehicle_name}/obstacle_detection/distance',
            Float32,
            queue_size=1
        )
        
        self.debug_image_pub = rospy.Publisher(
            f'/{self.vehicle_name}/obstacle_detection/debug_image/compressed',
            CompressedImage,
            queue_size=1
        )
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            f'/{self.vehicle_name}/camera_node/image/compressed',
            CompressedImage,
            self.image_callback
        )
        
        rospy.loginfo("Obstacle Detection Node initialized")
    
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
            
            # Detect obstacles
            obstacle_detected, distance, debug_image = self.detect_obstacles(cv_image)
            
            # Publish results
            self.obstacle_pub.publish(Bool(data=obstacle_detected))
            self.distance_pub.publish(Float32(data=distance))
            
            # Publish debug image
            if self.debug_image_pub.get_num_connections() > 0:
                self.publish_debug_image(debug_image)
                
        except Exception as e:
            rospy.logerr(f"ERROR in image_callback: {str(e)}")
            rospy.logerr(f"Message size: {len(msg.data) if hasattr(msg, 'data') else 'No data'}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def detect_obstacles(self, image):
        try:
            debug_image = image.copy()
            
            # Define region of interest (ROI)
            roi_height = int(self.image_height * self.detection_height_ratio)
            roi_width = int(self.image_width * self.detection_width_ratio)
            roi_x = int((self.image_width - roi_width) / 2)
            roi_y = self.image_height - roi_height
            
            # Draw ROI on debug image
            cv2.rectangle(debug_image, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (0, 255, 0), 2)
            
            # Extract ROI
            roi = image[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
            
            # Convert to grayscale
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray_roi, (5, 5), 0)
            
            # Edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Morphological operations to connect edges
            kernel = np.ones((3, 3), np.uint8)
            edges = cv2.dilate(edges, kernel, iterations=1)
            edges = cv2.erode(edges, kernel, iterations=1)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            obstacle_detected = False
            min_distance = float('inf')
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter small contours
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Estimate distance based on object size and position
                    # This is a simple heuristic - for more accuracy, use stereo vision or LIDAR
                    distance = self.estimate_distance(w, h, y + roi_y)
                    
                    if distance < self.obstacle_distance_threshold:
                        obstacle_detected = True
                        min_distance = min(min_distance, distance)
                        
                        # Draw obstacle on debug image
                        cv2.rectangle(debug_image, 
                                    (roi_x + x, roi_y + y), 
                                    (roi_x + x + w, roi_y + y + h), 
                                    (0, 0, 255), 2)
                        cv2.putText(debug_image, f"{distance:.2f}m", 
                                  (roi_x + x, roi_y + y - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            if not obstacle_detected:
                min_distance = self.obstacle_distance_threshold
            
            # Add status text to debug image
            status_text = "OBSTACLE DETECTED" if obstacle_detected else "PATH CLEAR"
            color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            cv2.putText(debug_image, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            return obstacle_detected, min_distance, debug_image
            
        except Exception as e:
            rospy.logerr(f"ERROR in detect_obstacles: {str(e)}")
            rospy.logerr(f"Image shape: {image.shape if image is not None else 'None'}")
            rospy.logerr(f"Detection parameters: height_ratio={self.detection_height_ratio}, width_ratio={self.detection_width_ratio}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            return False, self.obstacle_distance_threshold, image
    
    def estimate_distance(self, width, height, bottom_y):
        # Simple distance estimation based on object size and position
        # This is a heuristic method - calibrate based on your specific setup
        
        # Assume objects closer to bottom of image are closer to robot
        normalized_y = bottom_y / self.image_height
        
        # Assume larger objects are closer
        object_size = width * height
        normalized_size = object_size / (self.image_width * self.image_height)
        
        # Combine factors for distance estimation
        # These coefficients should be calibrated for your specific camera setup
        distance = 2.0 - (normalized_y * 1.5) - (normalized_size * 10.0)
        
        # Clamp distance to reasonable range
        distance = max(0.1, min(5.0, distance))
        
        return distance
    
    def publish_debug_image(self, image):
        try:
            # Encode image as JPEG
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 80]
            _, encoded_img = cv2.imencode('.jpg', image, encode_params)
            
            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = encoded_img.tobytes()
            
            self.debug_image_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"ERROR in publish_debug_image: {str(e)}")
            rospy.logerr(f"Image shape: {image.shape if image is not None else 'None'}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass