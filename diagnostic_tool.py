#!/usr/bin/env python3

import rospy
import subprocess
import datetime
import os
import sys

def create_error_report():
    """Generate a comprehensive error report for debugging"""
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = f"/tmp/duckiebot_error_report_{timestamp}.txt"
    
    with open(report_file, 'w') as f:
        f.write("=" * 50 + "\n")
        f.write("DUCKIEBOT AUTONOMOUS ERROR REPORT\n")
        f.write(f"Generated: {datetime.datetime.now()}\n")
        f.write("=" * 50 + "\n\n")
        
        # System Information
        f.write("SYSTEM INFORMATION:\n")
        f.write("-" * 20 + "\n")
        try:
            f.write(f"ROS Version: {subprocess.check_output(['rosversion', '-d']).decode().strip()}\n")
        except:
            f.write("ROS Version: ERROR - ROS not found\n")
        
        try:
            f.write(f"Python Version: {sys.version}\n")
        except:
            f.write("Python Version: ERROR\n")
            
        try:
            import cv2
            f.write(f"OpenCV Version: {cv2.__version__}\n")
        except:
            f.write("OpenCV Version: ERROR - Not installed\n")
            
        try:
            import numpy as np
            f.write(f"NumPy Version: {np.__version__}\n")
        except:
            f.write("NumPy Version: ERROR - Not installed\n")
        
        f.write("\n")
        
        # ROS Node Status
        f.write("ROS NODE STATUS:\n")
        f.write("-" * 20 + "\n")
        try:
            nodes = subprocess.check_output(['rosnode', 'list']).decode().strip().split('\n')
            autonomous_nodes = [n for n in nodes if 'autonomous' in n or 'object' in n or 'obstacle' in n]
            
            if autonomous_nodes:
                f.write("Active Autonomous Nodes:\n")
                for node in autonomous_nodes:
                    f.write(f"  ✓ {node}\n")
            else:
                f.write("❌ No autonomous nodes found\n")
                
        except Exception as e:
            f.write(f"❌ ERROR checking nodes: {str(e)}\n")
        
        f.write("\n")
        
        # Topic Status
        f.write("ROS TOPIC STATUS:\n")
        f.write("-" * 20 + "\n")
        try:
            topics = subprocess.check_output(['rostopic', 'list']).decode().strip().split('\n')
            
            critical_topics = [
                '/camera_node/image/compressed',
                '/object_detection/detection_data', 
                '/obstacle_detection/obstacle_detected',
                '/car_cmd_switch_node/cmd'
            ]
            
            for topic in critical_topics:
                matching_topics = [t for t in topics if topic in t]
                if matching_topics:
                    f.write(f"  ✓ {topic} (found: {matching_topics})\n")
                else:
                    f.write(f"  ❌ {topic} (missing)\n")
                    
        except Exception as e:
            f.write(f"❌ ERROR checking topics: {str(e)}\n")
        
        f.write("\n")
        
        # Camera Status
        f.write("CAMERA STATUS:\n")
        f.write("-" * 20 + "\n")
        
        camera_devices = ['/dev/video0', '/dev/video1']
        for device in camera_devices:
            if os.path.exists(device):
                f.write(f"  ✓ Camera found: {device}\n")
            else:
                f.write(f"  ❌ Camera missing: {device}\n")
        
        f.write("\n")
        
        # Recent Log Errors
        f.write("RECENT ROS LOG ERRORS:\n")
        f.write("-" * 20 + "\n")
        try:
            # Get recent rosout messages
            result = subprocess.run(['rostopic', 'echo', '/rosout', '-n', '10'], 
                                  capture_output=True, text=True, timeout=5)
            if result.stdout:
                f.write("Recent log messages:\n")
                f.write(result.stdout)
            else:
                f.write("No recent log messages found\n")
        except subprocess.TimeoutExpired:
            f.write("Timeout waiting for log messages\n")
        except Exception as e:
            f.write(f"ERROR reading logs: {str(e)}\n")
        
        f.write("\n")
        
        # Package Status
        f.write("PACKAGE STATUS:\n")
        f.write("-" * 20 + "\n")
        try:
            result = subprocess.run(['rospack', 'find', 'duckiebot_autonomous'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                f.write(f"  ✓ Package found: {result.stdout.strip()}\n")
            else:
                f.write("  ❌ Package not found\n")
        except Exception as e:
            f.write(f"  ❌ ERROR checking package: {str(e)}\n")
        
        f.write("\n")
        
        # Troubleshooting Tips
        f.write("TROUBLESHOOTING TIPS:\n")
        f.write("-" * 20 + "\n")
        f.write("1. Check if all nodes are running: rosnode list\n")
        f.write("2. Check topic data flow: rostopic hz /topic_name\n")
        f.write("3. View camera feed: rqt_image_view\n")
        f.write("4. Check for errors: rosrun rqt_console rqt_console\n")
        f.write("5. Restart nodes: rosnode kill /node_name\n")
        f.write("6. Check camera permissions: ls -la /dev/video*\n")
        f.write("7. Monitor CPU usage: htop\n")
        f.write("8. Check disk space: df -h\n")
        
        f.write("\n")
        f.write("=" * 50 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 50 + "\n")
    
    print(f"Error report generated: {report_file}")
    print("Please share this file when asking for help.")
    return report_file

def check_system_health():
    """Quick system health check"""
    print("🔍 DUCKIEBOT AUTONOMOUS HEALTH CHECK")
    print("=" * 40)
    
    # Check ROS
    try:
        subprocess.check_output(['rosversion', '-d'])
        print("✅ ROS is running")
    except:
        print("❌ ROS is not running")
        return False
    
    # Check camera
    if os.path.exists('/dev/video0'):
        print("✅ Camera device found")
    else:
        print("❌ Camera device not found")
    
    # Check nodes
    try:
        nodes = subprocess.check_output(['rosnode', 'list']).decode()
        if 'autonomous' in nodes:
            print("✅ Autonomous nodes detected")
        else:
            print("⚠️  No autonomous nodes running")
    except:
        print("❌ Cannot check node status")
    
    # Check topics
    try:
        topics = subprocess.check_output(['rostopic', 'list']).decode()
        if 'camera' in topics:
            print("✅ Camera topics found")
        else:
            print("❌ No camera topics found")
    except:
        print("❌ Cannot check topic status")
    
    print("\nFor detailed diagnostics, run: python3 diagnostic_tool.py --full-report")
    return True

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Duckiebot Autonomous Diagnostic Tool')
    parser.add_argument('--full-report', action='store_true', 
                      help='Generate full error report')
    parser.add_argument('--health-check', action='store_true', 
                      help='Quick health check')
    
    args = parser.parse_args()
    
    if args.full_report:
        create_error_report()
    elif args.health_check:
        check_system_health()
    else:
        print("Usage:")
        print("  python3 diagnostic_tool.py --health-check     # Quick check")
        print("  python3 diagnostic_tool.py --full-report      # Full diagnostic")