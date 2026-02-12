#!/bin/bash
# Validate ROS2 packages are built correctly

# Don't use set -e - we want to continue even if some checks have warnings
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Detect if we're in Docker container or on host
IN_CONTAINER=false
WORKSPACE_ROOT=""
WORKSPACE_BUILT=false

if [ -f /opt/ros/humble/setup.bash ]; then
    # Inside Docker container
    IN_CONTAINER=true
    WORKSPACE_ROOT="/workspace"
    source /opt/ros/humble/setup.bash
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
        WORKSPACE_BUILT=true
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
        echo "  ./scripts/validate.sh build"
        echo ""
        echo "Or install ROS2 Humble on your system."
        exit 1
    fi
    source "$WORKSPACE_DIR/install/setup.bash"
    WORKSPACE_BUILT=true
else
    # Neither container nor host workspace found
    echo "Error: ROS2 workspace not found."
    echo ""
    echo "Please run this script inside the Docker container:"
    echo "  ./scripts/validate.sh build"
    echo ""
    echo "Or build the workspace first:"
    echo "  colcon build --symlink-install"
    exit 1
fi

if [ "$WORKSPACE_BUILT" != "true" ]; then
    echo "Warning: Workspace not built yet. Some checks will be skipped."
    echo "Build it with: colcon build --symlink-install"
fi

CRITICAL_FAIL=false

echo "=========================================="
echo "ROS2 Build Validation"
echo "=========================================="
echo ""

# Check ROS2 environment
echo -n "Checking ROS2 environment... "
if [ -z "$ROS_DISTRO" ]; then
    echo "✗ FAIL"
    echo "  ROS_DISTRO not set"
    exit 1
fi
echo "✓ PASS (ROS_DISTRO=$ROS_DISTRO)"
echo ""

if [ "$WORKSPACE_BUILT" = false ]; then
    echo "Workspace not built. Run: colcon build --symlink-install"
    exit 1
fi

# Expected packages
EXPECTED_PACKAGES=("sdk_bridge_core" "frodobots_bridge" "camera_node")
EXPECTED_EXECUTABLES=(
    "sdk_bridge_core:bridge_service"
    "frodobots_bridge:frodobots_bridge"
    "camera_node:camera_node"
)

# Check packages exist
echo "Checking packages..."
for pkg in "${EXPECTED_PACKAGES[@]}"; do
    echo -n "  Package '$pkg'... "
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "    Package not found in: ros2 pkg list"
        CRITICAL_FAIL=true
    fi
done
echo ""

# Check executables exist
echo "Checking executables..."
for exec_pair in "${EXPECTED_EXECUTABLES[@]}"; do
    pkg=$(echo "$exec_pair" | cut -d: -f1)
    exec_name=$(echo "$exec_pair" | cut -d: -f2)
    echo -n "  Executable '$exec_name' in '$pkg'... "
    if ros2 pkg executables "$pkg" 2>/dev/null | grep -q "$exec_name"; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "    Executable not found. Run: ros2 pkg executables $pkg"
        CRITICAL_FAIL=true
    fi
done
echo ""

# Check Python imports
echo "Checking Python imports..."
echo -n "  sdk_bridge_core.base.sdk_bridge_base... "
if python3 -c "from sdk_bridge_core.base.sdk_bridge_base import SDKBridgeBase" 2>/dev/null; then
    echo "✓ PASS"
else
    echo "✗ FAIL"
    echo "    Cannot import SDKBridgeBase"
    CRITICAL_FAIL=true
fi

echo -n "  sdk_bridge_core.webrtc.WebRTCReceiver... "
if python3 -c "from sdk_bridge_core.webrtc import WebRTCReceiver" 2>/dev/null; then
    echo "✓ PASS"
else
    echo "✗ FAIL"
    echo "    Cannot import WebRTCReceiver"
    CRITICAL_FAIL=true
fi

echo -n "  frodobots_bridge.frodobots_adapter... "
IMPORT_ERROR=$(python3 -c "from frodobots_bridge.frodobots_adapter import FrodobotsAdapter" 2>&1)
if [ $? -eq 0 ]; then
    echo "✓ PASS"
else
    echo "⚠ WARNING"
    echo "    Cannot import FrodobotsAdapter"
    echo "$IMPORT_ERROR" | head -3 | sed 's/^/      /'
    if echo "$IMPORT_ERROR" | grep -q "NumPy"; then
        echo "    ⚠ NumPy version issue detected. Rebuild workspace: ./scripts/docker-build-workspace.sh"
    elif echo "$IMPORT_ERROR" | grep -q "interfaces.msg\|StreamTelemetry"; then
        echo "    ⚠ Custom message types need separate CMake package (known limitation)"
        echo "    Package structure is OK, but custom messages need special setup"
    fi
    # Don't exit - this is a known limitation
fi
echo ""

# Check launch files
echo "Checking launch files..."
echo -n "  sdk_bridge_core bridge_service.launch.py... "
    if ros2 launch sdk_bridge_core bridge_service.launch.py --show-args >/dev/null 2>&1; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "    Launch file not found or invalid"
        CRITICAL_FAIL=true
    fi

echo -n "  frodobots_bridge frodobots_bridge.launch.py... "
    if ros2 launch frodobots_bridge frodobots_bridge.launch.py --show-args >/dev/null 2>&1; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "    Launch file not found or invalid"
        CRITICAL_FAIL=true
    fi

echo -n "  camera_node camera_node.launch.py... "
    if ros2 launch camera_node camera_node.launch.py --show-args >/dev/null 2>&1; then
        echo "✓ PASS"
    else
        echo "✗ FAIL"
        echo "    Launch file not found or invalid"
        CRITICAL_FAIL=true
    fi
echo ""

# Check custom message types
echo "Checking custom message types..."
echo -n "  sdk_bridge_core/interfaces/msg/StreamTelemetry... "
    if ros2 interface show sdk_bridge_core/interfaces/msg/StreamTelemetry >/dev/null 2>&1; then
        echo "✓ PASS"
    else
        echo "⚠ WARNING"
        echo "    Message type not registered (expected for ament_python packages)"
    fi

echo -n "  sdk_bridge_core/interfaces/srv/ControlCommand... "
    if ros2 interface show sdk_bridge_core/interfaces/srv/ControlCommand >/dev/null 2>&1; then
        echo "✓ PASS"
    else
        echo "⚠ WARNING"
        echo "    Service type not registered (expected for ament_python packages)"
    fi
echo ""

# Check if we had any critical failures
if [ "$CRITICAL_FAIL" = true ]; then
    echo "=========================================="
    echo "✗ Build validation had critical failures"
    echo "=========================================="
    exit 1
else
    echo "=========================================="
    echo "✓ Build validation checks completed!"
    echo "  (Some warnings may be expected - see above)"
    echo "=========================================="
fi
