#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import json
import traceback
import sys

class AutonomousControllerNode:
    def __init__(self):
        rospy.init_node('autonomous_controller_node')
        
        # Parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot')
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.4)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
        self.target_distance = rospy.get_param('~target_distance', 1.0)  # meters
        self.obstacle_avoidance_distance = rospy.get_param('~obstacle_avoidance_distance', 0.5)
        
        # Control gains
        self.kp_linear = rospy.get_param('~kp_linear', 0.5)
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)
        self.ki_linear = rospy.get_param('~ki_linear', 0.1)
        self.ki_angular = rospy.get_param('~ki_angular', 0.0)
        
        # State variables
        self.object_detected = False
        self.obstacle_detected = False
        self.object_center_x = 0
        self.object_relative_x = 0
        self.object_distance = 5.0
        self.obstacle_distance = 5.0
        
        # Control state
        self.state = "SEARCHING"  # SEARCHING, FOLLOWING, AVOIDING
        self.last_object_time = rospy.Time.now()
        self.object_timeout = 3.0  # seconds
        
        # PID variables
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.last_time = rospy.Time.now()
        
        # Publishers
        self.cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )
        
        self.state_pub = rospy.Publisher(
            f'/{self.vehicle_name}/autonomous_controller/state',
            String,
            queue_size=1
        )
        
        # Subscribers
        self.object_detection_sub = rospy.Subscribe(
            f'/{self.vehicle_name}/object_detection/detection_data',
            Float32MultiArray,
            self.object_detection_callback
        )
        
        self.obstacle_detection_sub = rospy.Subscribe(
            f'/{self.vehicle_name}/obstacle_detection/obstacle_detected',
            Bool,
            self.obstacle_detection_callback
        )
        
        self.obstacle_distance_sub = rospy.Subscribe(
            f'/{self.vehicle_name}/obstacle_detection/distance',
            Float32,
            self.obstacle_distance_callback
        )
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)
        
        rospy.loginfo("Autonomous Controller Node initialized")
    
    def object_detection_callback(self, msg):
        try:
            if len(msg.data) >= 6:
                self.object_center_x = msg.data[0]
                center_y = msg.data[1]
                self.object_relative_x = msg.data[2]
                relative_y = msg.data[3]
                area = msg.data[4]
                self.object_distance = msg.data[5]
                
                self.object_detected = True
                self.last_object_time = rospy.Time.now()
                
                rospy.logdebug(f"Object detected: distance={self.object_distance:.2f}m, "
                             f"relative_x={self.object_relative_x:.2f}")
            else:
                rospy.logwarn(f"Invalid object detection data length: {len(msg.data)}, expected 6")
                
        except Exception as e:
            rospy.logerr(f"ERROR in object_detection_callback: {str(e)}")
            rospy.logerr(f"Data received: {msg.data if hasattr(msg, 'data') else 'No data'}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def obstacle_detection_callback(self, msg):
        try:
            self.obstacle_detected = msg.data
            
            if self.obstacle_detected:
                rospy.logwarn("Obstacle detected by controller")
                
        except Exception as e:
            rospy.logerr(f"ERROR in obstacle_detection_callback: {str(e)}")
            rospy.logerr(f"Message type: {type(msg)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def obstacle_distance_callback(self, msg):
        try:
            self.obstacle_distance = msg.data
            
        except Exception as e:
            rospy.logerr(f"ERROR in obstacle_distance_callback: {str(e)}")
            rospy.logerr(f"Message type: {type(msg)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def control_callback(self, event):
        try:
            current_time = rospy.Time.now()
            
            # Check if object detection is stale
            time_since_object = (current_time - self.last_object_time).to_sec()
            if time_since_object > self.object_timeout:
                self.object_detected = False
            
            # State machine
            self.update_state()
            
            # Generate control commands based on state
            cmd = self.generate_control_command()
            
            # Publish command
            self.cmd_pub.publish(cmd)
            
            # Publish state
            self.state_pub.publish(String(data=self.state))
            
            self.last_time = current_time
            
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR in control_callback: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
            # Publish emergency stop
            try:
                emergency_cmd = Twist2DStamped()
                emergency_cmd.header.stamp = rospy.Time.now()
                emergency_cmd.v = 0.0
                emergency_cmd.omega = 0.0
                self.cmd_pub.publish(emergency_cmd)
                rospy.logwarn("Emergency stop published due to control error")
            except:
                rospy.logerr("Failed to publish emergency stop command!")
    
    def update_state(self):
        if self.obstacle_detected and self.obstacle_distance < self.obstacle_avoidance_distance:
            self.state = "AVOIDING"
        elif self.object_detected:
            self.state = "FOLLOWING"
        else:
            self.state = "SEARCHING"
    
    def generate_control_command(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        
        dt = (rospy.Time.now() - self.last_time).to_sec()
        if dt <= 0:
            dt = 0.1
        
        if self.state == "AVOIDING":
            # Obstacle avoidance behavior
            cmd.v = 0.0
            cmd.omega = 1.5  # Turn right to avoid obstacle
            
            # Reset PID integrals
            self.linear_error_integral = 0.0
            self.angular_error_integral = 0.0
            
            rospy.loginfo("State: AVOIDING - Turning away from obstacle")
            
        elif self.state == "FOLLOWING":
            # Object following behavior with PID control
            
            # Distance control (linear velocity)
            distance_error = self.object_distance - self.target_distance
            self.linear_error_integral += distance_error * dt
            
            linear_velocity = (self.kp_linear * distance_error + 
                             self.ki_linear * self.linear_error_integral)
            
            # Steering control (angular velocity)
            steering_error = -self.object_relative_x  # Negative because we want to turn toward object
            self.angular_error_integral += steering_error * dt
            
            angular_velocity = (self.kp_angular * steering_error + 
                              self.ki_angular * self.angular_error_integral)
            
            # Apply speed limits
            cmd.v = np.clip(linear_velocity, -self.max_linear_speed, self.max_linear_speed)
            cmd.omega = np.clip(angular_velocity, -self.max_angular_speed, self.max_angular_speed)
            
            # Safety: reduce speed when turning sharply
            if abs(cmd.omega) > 1.0:
                cmd.v *= 0.5
            
            rospy.loginfo(f"State: FOLLOWING - v={cmd.v:.2f}, ω={cmd.omega:.2f}, "
                         f"dist_err={distance_error:.2f}, steer_err={steering_error:.2f}")
            
        else:  # SEARCHING
            # Search behavior - slow rotation
            cmd.v = 0.0
            cmd.omega = 0.5
            
            # Reset PID integrals
            self.linear_error_integral = 0.0
            self.angular_error_integral = 0.0
            
            rospy.loginfo("State: SEARCHING - Looking for target object")
        
        return cmd
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutonomousControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass