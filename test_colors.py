#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import sys

class ColorTestNode:
    def __init__(self, test_color):
        rospy.init_node('color_test_node', anonymous=True)
        self.bridge = CvBridge()
        self.test_color = test_color
        self.frame_count = 0
        
        # Subscribe to camera
        self.image_sub = rospy.Subscriber('/pinkduckie/camera_node/image/compressed', 
                                        CompressedImage, self.test_callback)
        
        rospy.loginfo(f"🧪 Testing {test_color.upper()} detection - move objects in front of camera")
        rospy.loginfo("Press Ctrl+C to test next color or exit")
        
    def get_color_ranges(self, color_name):
        """Get HSV ranges for testing"""
        color_ranges = {
            'red': [
                (np.array([0, 80, 80]), np.array([12, 255, 255])),
                (np.array([158, 80, 80]), np.array([180, 255, 255]))
            ],
            'blue': [(np.array([90, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([35, 80, 80]), np.array([85, 255, 255]))],
            'yellow': [(np.array([15, 100, 100]), np.array([35, 255, 255]))],
            'orange': [(np.array([5, 100, 100]), np.array([25, 255, 255]))],
            'purple': [(np.array([120, 80, 80]), np.array([170, 255, 255]))],
            'cyan': [(np.array([75, 100, 100]), np.array([105, 255, 255]))],
            'pink': [
                (np.array([150, 50, 150]), np.array([180, 255, 255])),
                (np.array([0, 50, 150]), np.array([15, 255, 255]))
            ]
        }
        return color_ranges.get(color_name.lower(), color_ranges['red'])
    
    def test_callback(self, msg):
        try:
            self.frame_count += 1
            if self.frame_count % 10 != 0:  # Process every 10th frame
                return
                
            # Convert image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
            
            # Convert to HSV and blur
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
            
            # Get ranges for test color
            ranges = self.get_color_ranges(self.test_color)
            
            # Create combined mask
            combined_mask = None
            total_pixels = 0
            
            for lower, upper in ranges:
                mask = cv2.inRange(blurred, lower, upper)
                pixels = cv2.countNonZero(mask)
                total_pixels += pixels
                
                if combined_mask is None:
                    combined_mask = mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # Apply morphological operations
            kernel = np.ones((3, 3), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter valid contours
            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 50:  # Minimum area
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h if h > 0 else 0
                    if 0.2 < aspect_ratio < 5.0:  # Reasonable aspect ratio
                        valid_contours.append(contour)
            
            # Log results
            if total_pixels > 100 or len(valid_contours) > 0:
                rospy.loginfo(f"🎨 {self.test_color.upper()}: {total_pixels} pixels, {len(contours)} contours, {len(valid_contours)} valid")
                
                if len(valid_contours) > 0:
                    largest = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        area = cv2.contourArea(largest)
                        rospy.loginfo(f"   📍 Largest object: center=({cx}, {cy}), area={area:.1f}")
            
        except Exception as e:
            rospy.logerr(f"Test error: {e}")

def main():
    colors_to_test = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'cyan', 'pink']
    
    if len(sys.argv) > 1:
        # Test specific color
        test_color = sys.argv[1].lower()
        if test_color in colors_to_test:
            try:
                tester = ColorTestNode(test_color)
                rospy.spin()
            except rospy.ROSInterruptException:
                rospy.loginfo(f"✅ {test_color.upper()} test completed")
        else:
            rospy.logerr(f"❌ Unsupported color: {test_color}")
            rospy.loginfo(f"Supported colors: {colors_to_test}")
    else:
        # Test all colors sequentially
        rospy.loginfo("🌈 Testing all colors sequentially...")
        rospy.loginfo("Place objects of each color in front of camera when prompted")
        
        for color in colors_to_test:
            rospy.loginfo(f"\n{'='*50}")
            rospy.loginfo(f"🎯 Testing {color.upper()} - place {color} object in view")
            rospy.loginfo("Press Ctrl+C when ready for next color")
            
            try:
                tester = ColorTestNode(color)
                rospy.spin()
            except rospy.ROSInterruptException:
                rospy.loginfo(f"✅ {color.upper()} test completed")
                continue
        
        rospy.loginfo("\n🎉 All color tests completed!")

if __name__ == '__main__':
    main()