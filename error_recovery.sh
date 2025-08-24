#!/bin/bash

# Error Recovery Script for Duckiebot Autonomous System
# This script helps recover from common errors

echo "🚨 DUCKIEBOT AUTONOMOUS ERROR RECOVERY"
echo "======================================"

# Function to log with timestamp
log_with_time() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Check if ROS is running
check_ros() {
    if pgrep -x "roscore" > /dev/null; then
        log_with_time "✅ ROS is running"
        return 0
    else
        log_with_time "❌ ROS is not running"
        return 1
    fi
}

# Emergency stop - kill all autonomous nodes
emergency_stop() {
    log_with_time "🛑 EMERGENCY STOP - Killing all autonomous nodes"
    
    # Kill specific nodes
    rosnode kill /autonomous_controller_node 2>/dev/null || true
    rosnode kill /color_object_detection_node 2>/dev/null || true
    rosnode kill /obstacle_detection_node 2>/dev/null || true
    rosnode kill /object_following_node 2>/dev/null || true
    
    # Send stop command
    rostopic pub -1 /duckiebot/car_cmd_switch_node/cmd duckietown_msgs/Twist2DStamped "
    header:
      stamp: now
    v: 0.0
    omega: 0.0" 2>/dev/null || true
    
    log_with_time "✅ Emergency stop completed"
}

# Restart camera node
restart_camera() {
    log_with_time "📷 Restarting camera node"
    
    # Kill camera node
    rosnode kill /duckiebot/camera_node 2>/dev/null || true
    sleep 2
    
    # Check camera device
    if [ -c /dev/video0 ]; then
        log_with_time "✅ Camera device found"
    else
        log_with_time "❌ Camera device not found - check connection"
        return 1
    fi
    
    log_with_time "Camera node restart initiated"
}

# Clean restart of autonomous system
clean_restart() {
    log_with_time "🔄 Performing clean restart"
    
    # Stop all autonomous nodes
    emergency_stop
    sleep 3
    
    # Clear parameter server
    rosparam delete /autonomous_controller_node 2>/dev/null || true
    rosparam delete /color_object_detection_node 2>/dev/null || true
    rosparam delete /obstacle_detection_node 2>/dev/null || true
    
    sleep 2
    
    # Restart the system
    log_with_time "🚀 Restarting autonomous system"
    roslaunch duckiebot_autonomous autonomous_following.launch veh:=${VEHICLE_NAME:-duckiebot} &
    
    sleep 5
    log_with_time "✅ Clean restart completed"
}

# Check system resources
check_resources() {
    log_with_time "📊 Checking system resources"
    
    # CPU usage
    cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
    log_with_time "CPU Usage: ${cpu_usage}%"
    
    # Memory usage
    mem_usage=$(free | grep Mem | awk '{printf("%.1f", $3/$2 * 100.0)}')
    log_with_time "Memory Usage: ${mem_usage}%"
    
    # Disk space
    disk_usage=$(df -h / | awk 'NR==2{printf "%s", $5}')
    log_with_time "Disk Usage: ${disk_usage}"
    
    # Temperature (if available)
    if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
        temp=$(cat /sys/class/thermal/thermal_zone0/temp)
        temp_c=$((temp/1000))
        log_with_time "Temperature: ${temp_c}°C"
        
        if [ $temp_c -gt 70 ]; then
            log_with_time "⚠️  High temperature detected!"
        fi
    fi
}

# Main menu
show_menu() {
    echo ""
    echo "Select recovery action:"
    echo "1) Emergency Stop (kill all nodes)"
    echo "2) Restart Camera"
    echo "3) Clean Restart System"
    echo "4) Check System Resources"
    echo "5) Generate Diagnostic Report"
    echo "6) View Recent Errors"
    echo "7) Exit"
    echo ""
    read -p "Enter choice [1-7]: " choice
    
    case $choice in
        1)
            emergency_stop
            ;;
        2)
            if check_ros; then
                restart_camera
            else
                log_with_time "❌ ROS not running - start ROS first"
            fi
            ;;
        3)
            if check_ros; then
                clean_restart
            else
                log_with_time "❌ ROS not running - start ROS first"
            fi
            ;;
        4)
            check_resources
            ;;
        5)
            if command -v python3 &> /dev/null; then
                python3 diagnostic_tool.py --full-report
            else
                log_with_time "❌ Python3 not found"
            fi
            ;;
        6)
            log_with_time "Recent ROS errors:"
            rostopic echo /rosout -n 5 | grep -i error || log_with_time "No recent errors found"
            ;;
        7)
            log_with_time "Exiting recovery script"
            exit 0
            ;;
        *)
            log_with_time "Invalid choice"
            ;;
    esac
}

# Check if running as argument
if [ $# -eq 1 ]; then
    case $1 in
        --emergency-stop)
            emergency_stop
            exit 0
            ;;
        --restart-camera)
            restart_camera
            exit 0
            ;;
        --clean-restart)
            clean_restart
            exit 0
            ;;
        --check-resources)
            check_resources
            exit 0
            ;;
        *)
            echo "Usage: $0 [--emergency-stop|--restart-camera|--clean-restart|--check-resources]"
            exit 1
            ;;
    esac
fi

# Interactive mode
while true; do
    show_menu
done