#!/bin/bash

# Fix permissions for Python scripts
echo "🔧 Fixing executable permissions for Python scripts..."

# Make Python scripts executable
chmod +x src/*.py
chmod +x diagnostic_tool.py
chmod +x error_recovery.sh
chmod +x setup.sh

# Make this script executable too
chmod +x fix_permissions.sh

echo "✅ Permissions fixed!"
echo ""
echo "Scripts made executable:"
echo "  - src/autonomous_controller_node.py"
echo "  - src/color_object_detection_node.py" 
echo "  - src/obstacle_detection_node.py"
echo "  - src/object_following_node.py"
echo "  - diagnostic_tool.py"
echo "  - error_recovery.sh"
echo "  - setup.sh"
echo "  - fix_permissions.sh"
