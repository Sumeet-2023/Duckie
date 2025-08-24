#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import json
import traceback

class ColorObjectDetectionNode:
    def __init__(self):
        rospy.init_node('color_object_detection_node', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Get target color from parameter
        self.target_color = rospy.get_param('~target_color', 'red_ball')
        rospy.loginfo(f"Color Object Detection Node initialized for target: {self.target_color}")
        
        # Publishers
        self.detection_pub = rospy.Publisher('/pinkduckie/object_detection/detections', String, queue_size=10)
        self.detection_data_pub = rospy.Publisher('/pinkduckie/object_detection/detection_data', Float32MultiArray, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber('/pinkduckie/camera_node/image/compressed', CompressedImage, self.image_callback)
        
        # Detection parameters
        self.min_area_base = 50  # Base minimum area for contours
        self.max_area_ratio = 0.3  # Maximum area as fraction of image
        
    def is_valid_contour(self, contour, image_area, min_area):
        """Enhanced contour validation with multiple criteria"""
        area = cv2.contourArea(contour)
        
        # Check minimum area (adaptive based on image size)
        adaptive_min_area = max(min_area, image_area // 10000)
        if area < adaptive_min_area:
            return False, f"area = {area:.1f} (too small, min: {adaptive_min_area})"
        
        # Check maximum area (avoid detecting entire image)
        max_area = image_area * self.max_area_ratio
        if area > max_area:
            return False, f"area = {area:.1f} (too large, max: {max_area:.1f})"
        
        # Check aspect ratio
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / h if h > 0 else 0
        if aspect_ratio > 8 or aspect_ratio < 0.125:  # Too elongated
            return False, f"area = {area:.1f} (bad aspect ratio: {aspect_ratio:.2f})"
        
        # Check solidity (how "solid" the shape is)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = area / hull_area
            if solidity < 0.2:  # Too irregular/fragmented
                return False, f"area = {area:.1f} (too irregular, solidity: {solidity:.2f})"
        
        return True, f"area = {area:.1f}"
    
    def get_color_ranges(self, color_name):
        """Get HSV ranges for different colors and strictness levels"""
        color_ranges = {
            'red': {
                'strict': [
                    (np.array([0, 120, 120]), np.array([8, 255, 255])),
                    (np.array([165, 120, 120]), np.array([180, 255, 255]))
                ],
                'relaxed': [
                    (np.array([0, 80, 80]), np.array([12, 255, 255])),
                    (np.array([158, 80, 80]), np.array([180, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([0, 50, 50]), np.array([15, 255, 255])),
                    (np.array([150, 50, 50]), np.array([180, 255, 255]))
                ]
            },
            'blue': {
                'strict': [
                    (np.array([100, 150, 150]), np.array([130, 255, 255]))
                ],
                'relaxed': [
                    (np.array([90, 100, 100]), np.array([130, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([80, 50, 50]), np.array([140, 255, 255]))
                ]
            },
            'green': {
                'strict': [
                    (np.array([40, 120, 120]), np.array([80, 255, 255]))
                ],
                'relaxed': [
                    (np.array([35, 80, 80]), np.array([85, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([30, 50, 50]), np.array([90, 255, 255]))
                ]
            },
            'yellow': {
                'strict': [
                    (np.array([20, 150, 150]), np.array([30, 255, 255]))
                ],
                'relaxed': [
                    (np.array([15, 100, 100]), np.array([35, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([10, 50, 50]), np.array([40, 255, 255]))
                ]
            },
            'orange': {
                'strict': [
                    (np.array([8, 150, 150]), np.array([20, 255, 255]))
                ],
                'relaxed': [
                    (np.array([5, 100, 100]), np.array([25, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([2, 50, 50]), np.array([30, 255, 255]))
                ]
            },
            'purple': {
                'strict': [
                    (np.array([130, 120, 120]), np.array([160, 255, 255]))
                ],
                'relaxed': [
                    (np.array([120, 80, 80]), np.array([170, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([110, 50, 50]), np.array([180, 255, 255]))
                ]
            },
            'cyan': {
                'strict': [
                    (np.array([80, 150, 150]), np.array([100, 255, 255]))
                ],
                'relaxed': [
                    (np.array([75, 100, 100]), np.array([105, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([70, 50, 50]), np.array([110, 255, 255]))
                ]
            },
            'pink': {
                'strict': [
                    (np.array([160, 80, 180]), np.array([180, 255, 255])),
                    (np.array([0, 80, 180]), np.array([10, 255, 255]))
                ],
                'relaxed': [
                    (np.array([150, 50, 150]), np.array([180, 255, 255])),
                    (np.array([0, 50, 150]), np.array([15, 255, 255]))
                ],
                'very_relaxed': [
                    (np.array([140, 30, 120]), np.array([180, 255, 255])),
                    (np.array([0, 30, 120]), np.array([20, 255, 255]))
                ]
            }
        }
        return color_ranges.get(color_name.lower(), color_ranges['red'])

    def detect_color_objects(self, image, target_color):
        """Universal color object detection with noise reduction"""
        height, width = image.shape[:2]
        image_area = height * width
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        # Get color ranges for the target color
        color_ranges = self.get_color_ranges(target_color)
        
        # Create masks for each strictness level
        masks = {}
        pixel_counts = {}
        
        for strictness in ['strict', 'relaxed', 'very_relaxed']:
            ranges = color_ranges[strictness]
            combined_mask = None
            
            for lower, upper in ranges:
                mask = cv2.inRange(blurred, lower, upper)
                if combined_mask is None:
                    combined_mask = mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # Apply morphological operations to clean up noise
            kernel = np.ones((3, 3), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            masks[strictness] = combined_mask
            pixel_counts[strictness] = cv2.countNonZero(combined_mask)
        
        # Choose the best mask based on pixel count and preference for less noisy detection
        if pixel_counts['strict'] > 500:  # Prefer strict if enough pixels
            best_mask = masks['strict']
            best_range_name = "strict"
        elif pixel_counts['relaxed'] > 200:  # Fall back to relaxed
            best_mask = masks['relaxed']
            best_range_name = "relaxed"
        else:  # Only use very relaxed as last resort
            best_mask = masks['very_relaxed']
            best_range_name = "very relaxed"
        
        # Find contours
        contours, _ = cv2.findContours(best_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        valid_contours = 0
        
        # Process each contour with improved filtering
        for i, contour in enumerate(contours):
            is_valid, reason = self.is_valid_contour(contour, image_area, self.min_area_base)
            
            if is_valid:
                valid_contours += 1
                # Calculate center and bounding box
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    x, y, w, h = cv2.boundingRect(contour)
                    area = cv2.contourArea(contour)
                    
                    detection = {
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area,
                        'bounding_box': [x, y, w, h],
                        'confidence': min(1.0, area / 1000.0)  # Simple confidence based on size
                    }
                    detections.append(detection)
                    
                    rospy.loginfo(f"✅ Valid contour {i}: {reason}, center: ({center_x}, {center_y}), bbox: {w}x{h}")
            else:
                if i < 20:  # Log first 20 rejected contours to avoid spam
                    rospy.loginfo(f"❌ Rejected contour {i}: {reason}")
        
        rospy.loginfo(f"🎯 Used {best_range_name} range for {target_color}. Found {len(contours)} total contours, {valid_contours} valid")
        rospy.loginfo(f"📊 {target_color.title()} pixels - Strict: {pixel_counts['strict']}, Relaxed: {pixel_counts['relaxed']}, Very relaxed: {pixel_counts['very_relaxed']}")
        
        return detections
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                rospy.logwarn("Failed to decode image")
                return
            
            # Extract color from target (e.g., "red_ball" -> "red")
            target_color = self.target_color.split('_')[0].lower()
            
            # Detect objects based on target color
            supported_colors = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'cyan', 'pink']
            if target_color in supported_colors:
                detections = self.detect_color_objects(image, target_color)
            else:
                rospy.logwarn(f"Unsupported target color: {target_color}. Supported colors: {supported_colors}")
                return
            
            # Publish detection results
            if detections:
                # Publish JSON string for compatibility
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
                
                # Publish array data for autonomous controller
                detection_data_msg = Float32MultiArray()
                if detections:
                    best_detection = max(detections, key=lambda d: d['area'])  # Largest object
                    detection_data_msg.data = [
                        float(best_detection['center_x']),
                        float(best_detection['center_y']),
                        float(best_detection['area']),
                        float(best_detection['confidence'])
                    ]
                    self.detection_data_pub.publish(detection_data_msg)
                    
                rospy.logdebug(f"Published {len(detections)} detections")
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            rospy.logerr(f"Message size: {len(msg.data) if hasattr(msg, 'data') else 'No data'}")

if __name__ == '__main__':
    try:
        node = ColorObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass