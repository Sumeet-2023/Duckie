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
        rospy.init_node('color_object_detection_node')
        
        # Parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot')
        
        # State variables
        self.bridge = CvBridge()
        
        # Color ranges for different objects (HSV)
        self.color_ranges = {
            'red_ball': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 100, 100]), 
                'upper2': np.array([180, 255, 255])
            },
            'blue_ball': {
                'lower': np.array([100, 100, 100]),
                'upper': np.array([130, 255, 255])
            },
            'green_ball': {
                'lower': np.array([40, 100, 100]),
                'upper': np.array([80, 255, 255])
            },
            'yellow_duckie': {
                'lower': np.array([20, 100, 100]),
                'upper': np.array([30, 255, 255])
            }
        }
        
        self.target_object = rospy.get_param('~target_object', 'red_ball')
        
        # Publishers
        self.detection_pub = rospy.Publisher(
            f'/{self.vehicle_name}/object_detection/detections',
            String,
            queue_size=1
        )
        
        self.detection_data_pub = rospy.Publisher(
            f'/{self.vehicle_name}/object_detection/detection_data',
            Float32MultiArray,
            queue_size=1
        )
        
        self.debug_image_pub = rospy.Publisher(
            f'/{self.vehicle_name}/object_detection/debug_image/compressed',
            CompressedImage,
            queue_size=1
        )
        
        # Subscribers
        self.image_sub = rospy.Subscribe(
            f'/{self.vehicle_name}/camera_node/image/compressed',
            CompressedImage,
            self.image_callback
        )
        
        rospy.loginfo(f"Color Object Detection Node initialized for target: {self.target_object}")
    
    def image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr("Failed to decode image - received None")
                return
                
            # Detect objects
            detections, debug_image = self.detect_objects(cv_image)
            
            # Publish detection results
            self.publish_detections(detections)
            
            # Publish debug image
            if self.debug_image_pub.get_num_connections() > 0:
                self.publish_debug_image(debug_image)
                
        except Exception as e:
            rospy.logerr(f"ERROR in image_callback: {str(e)}")
            rospy.logerr(f"Message size: {len(msg.data) if hasattr(msg, 'data') else 'No data'}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def detect_objects(self, image):
        try:
            debug_image = image.copy()
            detections = []
            
            # Convert BGR to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Get color range for target object
            if self.target_object in self.color_ranges:
                color_range = self.color_ranges[self.target_object]
                
                # Create mask - handle special case for red which spans 0 and 180 in HSV
                if self.target_object == 'red_ball':
                    mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
                    mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
                
                # Noise reduction
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Process contours
                for contour in contours:
                    area = cv2.contourArea(contour)
                    
                    # Filter by minimum area
                    if area > 1000:
                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Calculate center
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Calculate relative position (normalized to image center)
                        image_center_x = image.shape[1] // 2
                        image_center_y = image.shape[0] // 2
                        
                        relative_x = (center_x - image_center_x) / image_center_x
                        relative_y = (center_y - image_center_y) / image_center_y
                        
                        # Estimate distance based on object size
                        distance = self.estimate_distance_from_area(area)
                        
                        detection = {
                            'object_type': self.target_object,
                            'center_x': center_x,
                            'center_y': center_y,
                            'relative_x': relative_x,
                            'relative_y': relative_y,
                            'area': area,
                            'distance': distance,
                            'bounding_box': [x, y, w, h]
                        }
                        
                        detections.append(detection)
                        
                        # Draw on debug image
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(debug_image, (center_x, center_y), 5, (255, 0, 0), -1)
                        
                        # Add text labels
                        label = f"{self.target_object}: {distance:.2f}m"
                        cv2.putText(debug_image, label, (x, y - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Show mask in corner of debug image for debugging
                mask_resized = cv2.resize(mask, (160, 120))
                mask_colored = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)
                debug_image[10:130, 10:170] = mask_colored
            else:
                rospy.logerr(f"Unknown target object: {self.target_object}")
            
            # Add detection count to debug image
            cv2.putText(debug_image, f"Detections: {len(detections)}", 
                       (10, debug_image.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            return detections, debug_image
            
        except Exception as e:
            rospy.logerr(f"ERROR in detect_objects: {str(e)}")
            rospy.logerr(f"Image shape: {image.shape if image is not None else 'None'}")
            rospy.logerr(f"Target object: {self.target_object}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            return [], image
    
    def estimate_distance_from_area(self, area):
        # Simple distance estimation based on object area
        # This should be calibrated for your specific objects and camera
        
        # Assuming a known object size (e.g., standard ball diameter)
        # These values should be calibrated experimentally
        reference_area = 5000  # Area of object at 1 meter distance
        reference_distance = 1.0
        
        if area > 0:
            # Inverse square law approximation
            estimated_distance = reference_distance * np.sqrt(reference_area / area)
            return max(0.1, min(5.0, estimated_distance))  # Clamp to reasonable range
        else:
            return 5.0
    
    def publish_detections(self, detections):
        try:
            # Publish as JSON string
            detection_json = json.dumps(detections)
            self.detection_pub.publish(String(data=detection_json))
            
            # Publish as Float32MultiArray for easier processing
            if detections:
                # Take the largest detection (closest object)
                largest_detection = max(detections, key=lambda d: d['area'])
                
                data = Float32MultiArray()
                data.data = [
                    largest_detection['center_x'],
                    largest_detection['center_y'],
                    largest_detection['relative_x'],
                    largest_detection['relative_y'],
                    largest_detection['area'],
                    largest_detection['distance']
                ]
                self.detection_data_pub.publish(data)
        except Exception as e:
            rospy.logerr(f"ERROR in publish_detections: {str(e)}")
            rospy.logerr(f"Detections count: {len(detections) if detections else 0}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
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
        node = ColorObjectDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass