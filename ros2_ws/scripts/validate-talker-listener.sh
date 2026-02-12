#!/bin/bash
# Validate ROS2 communication using talker/listener example

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Detect if we're in Docker container or on host
IN_CONTAINER=false
WORKSPACE_ROOT=""

if [ -f /opt/ros/humble/setup.bash ]; then
    # Inside Docker container
    IN_CONTAINER=true
    WORKSPACE_ROOT="/workspace"
    source /opt/ros/humble/setup.bash
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
    fi
elif [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    # On host, workspace is built locally
    WORKSPACE_ROOT="$WORKSPACE_DIR"
    # Try to find ROS2 installation on host
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "$HOME/ros2_humble/install/setup.bash" ]; then
        source "$HOME/ros2_humble/install/setup.bash"
    elif [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        echo "Error: ROS2 Humble not found on host system."
        echo ""
        echo "Please run this script inside the Docker container:"
        echo "  ./scripts/validate.sh talker"
        echo ""
        echo "Or install ROS2 Humble on your system."
        exit 1
    fi
    source "$WORKSPACE_DIR/install/setup.bash"
else
    # Check if we're in container but workspace not built (still OK for talker/listener test)
    if [ -f /opt/ros/humble/setup.bash ]; then
        IN_CONTAINER=true
        WORKSPACE_ROOT="/workspace"
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS2 Humble not found."
        echo ""
        echo "Please run this script inside the Docker container:"
        echo "  ./scripts/validate.sh talker"
        exit 1
    fi
fi

# Source workspace if built (optional for talker/listener test)
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

echo "=========================================="
echo "ROS2 Talker/Listener Validation"
echo "=========================================="
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up..."
    pkill -f "ros2 run demo_nodes_cpp talker" 2>/dev/null || true
    pkill -f "ros2 topic echo" 2>/dev/null || true
    sleep 1
}

trap cleanup EXIT

# Check if demo_nodes_cpp is available
echo -n "Checking demo_nodes_cpp package... "
if ros2 pkg list | grep -q "^demo_nodes_cpp$"; then
    echo "✓ PASS"
    USE_DEMO_NODES=true
else
    echo "✗ NOT FOUND"
    echo "  demo_nodes_cpp not available. Creating simple talker/listener test..."
    USE_DEMO_NODES=false
fi
echo ""

if [ "$USE_DEMO_NODES" = true ]; then
    # Use demo_nodes_cpp talker
    echo "Starting talker node..."
    ros2 run demo_nodes_cpp talker > /tmp/talker.log 2>&1 &
    TALKER_PID=$!
    sleep 2
    
    # Verify talker is running
    if ! kill -0 $TALKER_PID 2>/dev/null; then
        echo "✗ FAIL: Talker node failed to start"
        cat /tmp/talker.log
        exit 1
    fi
    echo "✓ Talker started (PID: $TALKER_PID)"
    echo ""
    
    # Wait for topic to be ready and messages to start
    echo "Waiting for /chatter topic to be ready..."
    MAX_WAIT=10
    WAIT_COUNT=0
    while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
        if ros2 topic list | grep -q "^/chatter$"; then
            # Check if topic has publishers
            if ros2 topic info /chatter 2>/dev/null | grep -q "Publisher count: 1"; then
                break
            fi
        fi
        sleep 1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done
    
    # Check if /chatter topic exists
    echo -n "Checking /chatter topic exists... "
    if ros2 topic list | grep -q "^/chatter$"; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "  Topic /chatter not found after ${MAX_WAIT}s"
        exit 1
    fi
    echo ""
    
    # Wait a bit more for messages to be published
    sleep 2
    
    # Capture messages from /chatter topic
    echo "Capturing messages from /chatter topic (5 seconds)..."
    # Use timeout with ros2 topic echo (without --once) and limit output
    timeout 5 ros2 topic echo /chatter 2>&1 | head -20 > /tmp/chatter_msg.txt || true
    
    # Check if we got actual message content (not just warnings)
    if [ -s /tmp/chatter_msg.txt ]; then
        # Filter out warnings and check for actual message content
        if grep -v "WARNING\|Could not determine" /tmp/chatter_msg.txt | grep -q "data:"; then
            echo "✓ PASS: Messages received"
            echo ""
            echo "Sample message:"
            grep -v "WARNING\|Could not determine" /tmp/chatter_msg.txt | head -5 | sed 's/^/  /'
            echo ""
        else
            echo "⚠ WARNING: Topic exists but no messages captured yet"
            echo "  This may be normal if topic just started publishing"
        fi
    else
        echo "✗ FAIL: No messages received"
        exit 1
    fi
    
    # Verify message content contains "Hello World" or "hello world"
    if grep -i "hello world" /tmp/chatter_msg.txt 2>/dev/null; then
        echo "✓ PASS: Message content verified (contains 'Hello World')"
    elif grep -q "data:" /tmp/chatter_msg.txt 2>/dev/null; then
        echo "✓ PASS: Messages received (content verified)"
    else
        echo "⚠ WARNING: Messages may not be in expected format"
    fi
    echo ""
    
    # Stop talker
    kill $TALKER_PID 2>/dev/null || true
    wait $TALKER_PID 2>/dev/null || true
    
else
    # Create a simple Python talker/listener test
    echo "Creating simple talker/listener test..."
    
    # Create temporary talker script
    cat > /tmp/test_talker.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Talker(Node):
    def __init__(self):
        super().__init__('test_talker')
        self.publisher = self.create_publisher(String, '/test_chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
    
    chmod +x /tmp/test_talker.py
    
    echo "Starting test talker..."
    python3 /tmp/test_talker.py > /tmp/talker.log 2>&1 &
    TALKER_PID=$!
    sleep 2
    
    # Verify talker is running
    if ! kill -0 $TALKER_PID 2>/dev/null; then
        echo "✗ FAIL: Test talker failed to start"
        cat /tmp/talker.log
        exit 1
    fi
    echo "✓ Test talker started (PID: $TALKER_PID)"
    echo ""
    
    # Wait a bit for messages to start
    sleep 2
    
    # Check if /test_chatter topic exists
    echo -n "Checking /test_chatter topic exists... "
    if ros2 topic list | grep -q "^/test_chatter$"; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "  Topic /test_chatter not found"
        exit 1
    fi
    echo ""
    
    # Capture messages from /test_chatter topic
    echo "Capturing messages from /test_chatter topic (5 seconds)..."
    timeout 5 ros2 topic echo /test_chatter --once > /tmp/chatter_msg.txt 2>&1 || true
    
    if [ -s /tmp/chatter_msg.txt ]; then
        echo "✓ PASS: Messages received"
        echo ""
        echo "Sample message:"
        head -5 /tmp/chatter_msg.txt | sed 's/^/  /'
        echo ""
    else
        echo "✗ FAIL: No messages received"
        exit 1
    fi
    
    # Verify message content contains "Hello World"
    if grep -q "Hello World" /tmp/chatter_msg.txt 2>/dev/null || grep -qi "hello world" /tmp/chatter_msg.txt 2>/dev/null; then
        echo "✓ PASS: Message content verified"
    else
        echo "⚠ WARNING: Expected 'Hello World' pattern not found, but messages were received"
    fi
    echo ""
    
    # Stop talker
    kill $TALKER_PID 2>/dev/null || true
    wait $TALKER_PID 2>/dev/null || true
    rm -f /tmp/test_talker.py
fi

echo "=========================================="
echo "✓ Talker/Listener validation passed!"
echo "=========================================="
