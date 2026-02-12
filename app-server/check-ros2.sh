#!/bin/bash
# Diagnostic script to check ROS2 connectivity

set -e

echo "=== ROS2 Connectivity Diagnostics ==="
echo ""

# Check ROS_DOMAIN_ID
echo "1. ROS_DOMAIN_ID:"
echo "   Host: ${ROS_DOMAIN_ID:-0} (default)"
docker exec ros2_bridge bash -c 'echo "   Docker: $ROS_DOMAIN_ID"' 2>/dev/null || echo "   Docker: (container not running)"
echo ""

# Check if ROS2 is available
echo "2. ROS2 Availability:"
if python3 -c "import rclpy" 2>/dev/null; then
    echo "   ✓ ROS2 Python packages available"
else
    echo "   ✗ ROS2 Python packages NOT available"
    echo "   This is OK - app-server can run without ROS2"
fi
echo ""

# Check topics in Docker
echo "3. Topics in Docker container:"
docker exec ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null || echo "   (Could not check - container may not be running)"
echo ""

# Check if app-server can see topics (if ROS2 is available)
if python3 -c "import rclpy" 2>/dev/null; then
    echo "4. Topics visible from host (if ROS2 installed locally):"
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    if command -v ros2 &> /dev/null; then
        ros2 topic list 2>/dev/null || echo "   (Could not list topics - ROS2 may not be properly configured)"
    else
        echo "   (ros2 command not available - ROS2 not installed locally)"
    fi
else
    echo "4. Topics visible from host:"
    echo "   (ROS2 not available locally - cannot check)"
fi
echo ""

# Check app-server status
echo "5. App-server status:"
if curl -s http://localhost:8001/ > /dev/null 2>&1; then
    echo "   ✓ App-server is running"
    curl -s http://localhost:8001/ | python3 -m json.tool 2>/dev/null || curl -s http://localhost:8001/
else
    echo "   ✗ App-server is NOT running"
fi
echo ""

echo "=== Recommendations ==="
echo ""
echo "If messages aren't appearing:"
echo "1. Make sure ROS_DOMAIN_ID matches (see above)"
echo "2. On macOS, Docker ROS2 may not be visible to host"
echo "3. Try running app-server INSIDE Docker (see RUNNING_TALKER.md)"
echo "4. Or install ROS2 locally: brew install ros-humble-desktop"
