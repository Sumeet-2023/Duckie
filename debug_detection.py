#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class DetectionDebugger:
    def __init__(self):
        rospy.init_node('detection_debugger', anonymous=True)
        self.bridge = CvBridge()
        
        # Get target color from parameter
        self.target_color = rospy.get_param('~target_color', 'red_ball')
        color_name = self.target_color.split('_')[0].lower()
        
        # Subscribe to camera
        self.image_sub = rospy.Subscriber('/pinkduckie/camera_node/image/compressed', 
                                        CompressedImage, self.debug_callback)
        
        rospy.loginfo(f"🔍 Detection debugger started - place a {color_name.upper()} object in front of camera")
        
    def get_color_ranges(self, color_name):
        """Get HSV ranges for different colors - same as main detection node"""
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
                'strict': [(np.array([100, 150, 150]), np.array([130, 255, 255]))],
                'relaxed': [(np.array([90, 100, 100]), np.array([130, 255, 255]))],
                'very_relaxed': [(np.array([80, 50, 50]), np.array([140, 255, 255]))]
            },
            'green': {
                'strict': [(np.array([40, 120, 120]), np.array([80, 255, 255]))],
                'relaxed': [(np.array([35, 80, 80]), np.array([85, 255, 255]))],
                'very_relaxed': [(np.array([30, 50, 50]), np.array([90, 255, 255]))]
            },
            'yellow': {
                'strict': [(np.array([20, 150, 150]), np.array([30, 255, 255]))],
                'relaxed': [(np.array([15, 100, 100]), np.array([35, 255, 255]))],
                'very_relaxed': [(np.array([10, 50, 50]), np.array([40, 255, 255]))]
            },
            'orange': {
                'strict': [(np.array([8, 150, 150]), np.array([20, 255, 255]))],
                'relaxed': [(np.array([5, 100, 100]), np.array([25, 255, 255]))],
                'very_relaxed': [(np.array([2, 50, 50]), np.array([30, 255, 255]))]
            },
            'purple': {
                'strict': [(np.array([130, 120, 120]), np.array([160, 255, 255]))],
                'relaxed': [(np.array([120, 80, 80]), np.array([170, 255, 255]))],
                'very_relaxed': [(np.array([110, 50, 50]), np.array([180, 255, 255]))]
            },
            'cyan': {
                'strict': [(np.array([80, 150, 150]), np.array([100, 255, 255]))],
                'relaxed': [(np.array([75, 100, 100]), np.array([105, 255, 255]))],
                'very_relaxed': [(np.array([70, 50, 50]), np.array([110, 255, 255]))]
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
        
    def is_valid_contour(self, contour, image_area, min_area=50):
        """Enhanced contour validation matching the main detection node"""
        area = cv2.contourArea(contour)
        
        # Adaptive minimum area
        adaptive_min_area = max(min_area, image_area // 10000)
        if area < adaptive_min_area:
            return False, f"area = {area:.1f} (too small, min: {adaptive_min_area})"
        
        # Maximum area check
        max_area = image_area * 0.3
        if area > max_area:
            return False, f"area = {area:.1f} (too large)"
        
        # Aspect ratio check
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / h if h > 0 else 0
        if aspect_ratio > 8 or aspect_ratio < 0.125:
            return False, f"area = {area:.1f} (bad aspect ratio: {aspect_ratio:.2f})"
        
        # Solidity check
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = area / hull_area
            if solidity < 0.2:
                return False, f"area = {area:.1f} (irregular, solidity: {solidity:.2f})"
        
        return True, f"area = {area:.1f}"
        
    def debug_callback(self, msg):
        try:
            # Convert image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
                
            height, width = image.shape[:2]
            image_area = height * width
            rospy.loginfo(f"📷 Image: {width}x{height} pixels")
            
            # Get target color
            color_name = self.target_color.split('_')[0].lower()
            color_ranges = self.get_color_ranges(color_name)
            
            # Convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
            
            # Test different ranges for the target color
            best_range = None
            best_count = 0
            
            for strictness in ['strict', 'relaxed', 'very_relaxed']:
                ranges = color_ranges[strictness]
                combined_mask = None
                
                for i, (lower, upper) in enumerate(ranges):
                    mask = cv2.inRange(blurred, lower, upper)
                    if combined_mask is None:
                        combined_mask = mask
                    else:
                        combined_mask = cv2.bitwise_or(combined_mask, mask)
                    
                    pixel_count = cv2.countNonZero(mask)
                    rospy.loginfo(f"🎨 {color_name.title()} Range {i+1} ({strictness}): {pixel_count} pixels")
                
                # Apply morphological operations
                kernel = np.ones((3, 3), np.uint8)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
                
                total_pixel_count = cv2.countNonZero(combined_mask)
                rospy.loginfo(f"🎯 {color_name.title()} Combined ({strictness}): {total_pixel_count} pixels (after morphology)")
                
                if total_pixel_count > best_count:
                    best_count = total_pixel_count
                    best_range = f"{color_name.title()} Combined ({strictness})"
                    final_mask = combined_mask
            
            rospy.loginfo(f"🏆 Best range: {best_range} with {best_count} pixels")
            
            # Find and analyze contours
            contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            rospy.loginfo(f"🔍 Found {len(contours)} contours")
            
            valid_count = 0
            for i, contour in enumerate(contours[:20]):  # Limit output
                is_valid, reason = self.is_valid_contour(contour, image_area)
                
                if is_valid:
                    valid_count += 1
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                        x, y, w, h = cv2.boundingRect(contour)
                        rospy.loginfo(f"✅ Contour {i}: {reason}")
                        rospy.loginfo(f"📍 Object center: ({center_x}, {center_y})")
                        rospy.loginfo(f"📦 Bounding box: {w}x{h}")
                else:
                    rospy.loginfo(f"❌ Contour {i}: {reason}")
            
            rospy.loginfo(f"✨ Valid contours: {valid_count}/{len(contours)}")
            
            # Test other colors for comparison
            other_colors = {
                'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
                'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
                'yellow': (np.array([20, 50, 50]), np.array([30, 255, 255])),
                'orange': (np.array([8, 50, 50]), np.array([20, 255, 255])),
                'purple': (np.array([130, 50, 50]), np.array([160, 255, 255])),
                'cyan': (np.array([80, 50, 50]), np.array([100, 255, 255]))
            }
            
            for color, (lower, upper) in other_colors.items():
                if color != color_name:  # Skip the target color
                    mask = cv2.inRange(blurred, lower, upper)
                    pixels = cv2.countNonZero(mask)
                    rospy.loginfo(f"🌈 {color.title()}: {pixels} pixels detected")
            
            rospy.loginfo("=" * 50)
            
        except Exception as e:
            rospy.logerr(f"Debug error: {e}")

if __name__ == '__main__':
    try:
        debugger = DetectionDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass