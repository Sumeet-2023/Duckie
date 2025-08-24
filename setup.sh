#!/bin/bash

# Duckiebot Autonomous Setup Script
# This script helps set up the autonomous object-following system

set -e

echo "🤖 Duckiebot Autonomous Setup Script"
echo "====================================="

# Check if running on Jetson Nano
if [ -f /etc/nv_tegra_release ]; then
    echo "✅ Jetson Nano detected"
else
    echo "⚠️  Warning: Not running on Jetson Nano"
fi

# Check ROS installation
if command -v roscore &> /dev/null; then
    echo "✅ ROS is installed"
    ROS_VERSION=$(rosversion -d)
    echo "   ROS Version: $ROS_VERSION"
else
    echo "❌ ROS is not installed or not in PATH"
    exit 1
fi

# Check for required ROS packages
echo "🔍 Checking ROS dependencies..."

REQUIRED_PACKAGES=("cv_bridge" "image_transport" "sensor_msgs" "geometry_msgs")
MISSING_PACKAGES=()

for package in "${REQUIRED_PACKAGES[@]}"; do
    if rospack find $package &> /dev/null; then
        echo "   ✅ $package"
    else
        echo "   ❌ $package (missing)"
        MISSING_PACKAGES+=($package)
    fi
done

# Check for duckietown_msgs
if rospack find duckietown_msgs &> /dev/null; then
    echo "   ✅ duckietown_msgs"
else
    echo "   ⚠️  duckietown_msgs not found"
    echo "   Install dt-ros-commons from: https://github.com/duckietown/dt-ros-commons"
fi

# Check Python dependencies
echo "🐍 Checking Python dependencies..."

PYTHON_PACKAGES=("cv2" "numpy" "rospy")
for package in "${PYTHON_PACKAGES[@]}"; do
    if python3 -c "import $package" &> /dev/null; then
        echo "   ✅ $package"
    else
        echo "   ❌ $package (missing)"
        if [ "$package" == "cv2" ]; then
            echo "      Install with: pip3 install opencv-python"
        elif [ "$package" == "numpy" ]; then
            echo "      Install with: pip3 install numpy"
        fi
    fi
done

# Setup workspace if needed
if [ ! -d "$HOME/catkin_ws" ]; then
    echo "📁 Creating catkin workspace..."
    mkdir -p $HOME/catkin_ws/src
    cd $HOME/catkin_ws
    catkin_make
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
else
    echo "✅ Catkin workspace exists"
fi

# Make scripts executable
echo "🔧 Making Python scripts executable..."
chmod +x src/*.py
chmod +x diagnostic_tool.py
chmod +x error_recovery.sh

# Set up environment variables
echo "🌍 Setting up environment..."

if ! grep -q "VEHICLE_NAME" $HOME/.bashrc; then
    echo "export VEHICLE_NAME=duckiebot" >> $HOME/.bashrc
    echo "Added VEHICLE_NAME to .bashrc"
fi

# Camera setup for Jetson Nano
echo "📷 Camera setup..."
if [ -c /dev/video0 ]; then
    echo "✅ Camera detected at /dev/video0"
else
    echo "⚠️  No camera detected at /dev/video0"
    echo "   Make sure camera is connected and drivers are installed"
fi

# Performance optimization for Jetson Nano
echo "⚡ Performance optimization..."

# Set CPU governor to performance mode (requires sudo)
echo "To optimize performance, run as root:"
echo "  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"

# Jetson fan control (if available)
if [ -f /sys/class/thermal/thermal_zone*/temp ]; then
    echo "🌡️  Thermal monitoring available"
fi

# Create a quick test script
cat > test_camera.py << 'EOF'
#!/usr/bin/env python3
import cv2
import sys

def test_camera():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return False
    
    ret, frame = cap.read()
    if not ret:
        print("❌ Cannot read from camera")
        cap.release()
        return False
    
    print(f"✅ Camera working - Resolution: {frame.shape[1]}x{frame.shape[0]}")
    cap.release()
    return True

if __name__ == "__main__":
    test_camera()
EOF

chmod +x test_camera.py

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Source your environment: source ~/.bashrc"
echo "2. Test camera: python3 test_camera.py"
echo "3. Set your vehicle name: export VEHICLE_NAME=your_robot_name"
echo "4. Build the package: cd ~/catkin_ws && catkin_make"
echo "5. Launch the system: roslaunch duckiebot_autonomous autonomous_following.launch"
echo ""
echo "🔧 Troubleshooting tools:"
echo "- Quick health check: python3 diagnostic_tool.py --health-check"
echo "- Full diagnostic: python3 diagnostic_tool.py --full-report"
echo "- Emergency recovery: ./error_recovery.sh"
echo ""
echo "For more information, see README.md"