#!/bin/bash

# Docker Duckiebot Autonomous Setup Script
echo "🐳 Docker Duckiebot Autonomous Setup Script"
echo "============================================"

# Get the current directory (where the script is located)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "📁 Working from: $SCRIPT_DIR"

# Verify we're in Docker
if [ -f /.dockerenv ]; then
    echo "✅ Running in Docker container"
else
    echo "⚠️  Warning: Not running in Docker container"
fi

# Set environment variables
export VEHICLE_NAME=${VEHICLE_NAME:-pinkduckie}
echo "🤖 Vehicle name: $VEHICLE_NAME"

# Check ROS installation
if command -v roscore &> /dev/null; then
    echo "✅ ROS is installed"
    echo "   ROS Version: $(rosversion -d)"
else
    echo "❌ ROS is not installed"
    exit 1
fi

# Source ROS environment
source /opt/ros/noetic/setup.bash
if [ -f /code/catkin_ws/devel/setup.bash ]; then
    source /code/catkin_ws/devel/setup.bash
    echo "✅ Sourced existing workspace"
fi

# Check if we're in the right location
if [ ! -f "$SCRIPT_DIR/package.xml" ]; then
    echo "❌ package.xml not found. Are you in the right directory?"
    exit 1
fi

# Create symbolic link or copy to the catkin workspace
PACKAGE_NAME="duckiebot_autonomous"
TARGET_DIR="/code/catkin_ws/src/$PACKAGE_NAME"

echo "📦 Setting up package in catkin workspace..."

# Remove existing package if it exists
if [ -d "$TARGET_DIR" ]; then
    rm -rf "$TARGET_DIR"
fi

# Create directory and copy files
mkdir -p "$TARGET_DIR"
cp -r "$SCRIPT_DIR"/* "$TARGET_DIR/"
echo "   ✅ Package copied to $TARGET_DIR"

# Make Python scripts executable
echo "🔧 Making Python scripts executable..."
if [ -d "$TARGET_DIR/src" ]; then
    chmod +x "$TARGET_DIR/src"/*.py
    echo "   ✅ Python scripts made executable"
else
    echo "   ⚠️  No src directory found"
fi

# Make launch files accessible
if [ -d "$TARGET_DIR/launch" ]; then
    echo "   ✅ Launch files found"
else
    echo "   ⚠️  No launch directory found"
fi

# Build the package
echo "🔨 Building catkin workspace..."
cd /code/catkin_ws
catkin_make --pkg $PACKAGE_NAME

# Source the workspace
source /code/catkin_ws/devel/setup.bash

# Add to bashrc for persistence
echo "export VEHICLE_NAME=$VEHICLE_NAME" >> ~/.bashrc
echo "source /code/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "✅ Docker setup complete!"
echo ""
echo "🚀 To run the autonomous system:"
echo "1. Make sure you're in the container: docker exec -it duckiebot-interface bash"
echo "2. Navigate to workspace: cd /code/catkin_ws"
echo "3. Source environment: source devel/setup.bash"
echo "4. Launch system:"
echo "   roslaunch $PACKAGE_NAME autonomous_following.launch veh:=$VEHICLE_NAME"
echo ""
echo "🔧 Diagnostic commands:"
echo "   cd $TARGET_DIR && python3 diagnostic_tool.py --health-check"
echo "   cd $TARGET_DIR && ./error_recovery.sh"
echo ""
echo "📋 Quick commands to copy-paste:"
echo "cd /code/catkin_ws && source devel/setup.bash"
echo "roslaunch $PACKAGE_NAME autonomous_following.launch veh:=$VEHICLE_NAME"