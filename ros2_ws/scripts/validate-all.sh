#!/bin/bash
# Master validation script - runs all validation checks

# Don't use set -e here - we want to run all validations even if one fails

# Determine script directory - works both in container and on host
if [ -f /workspace/scripts/validate-all.sh ]; then
    # Inside container
    SCRIPT_DIR="/workspace/scripts"
    WORKSPACE_DIR="/workspace"
else
    # On host
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
fi

cd "$WORKSPACE_DIR"

echo "=========================================="
echo "ROS2 Complete Validation Suite"
echo "=========================================="
echo ""

# Track overall success
OVERALL_SUCCESS=true

# Run validate-build.sh
echo ">>> Running build validation..."
echo ""
if bash "$SCRIPT_DIR/validate-build.sh"; then
    echo ""
    echo "✓ Build validation passed"
else
    echo ""
    echo "✗ Build validation failed"
    OVERALL_SUCCESS=false
fi
echo ""

# Run validate-talker-listener.sh
echo ">>> Running talker/listener validation..."
echo ""
if bash "$SCRIPT_DIR/validate-talker-listener.sh"; then
    echo ""
    echo "✓ Talker/Listener validation passed"
else
    echo ""
    echo "✗ Talker/Listener validation failed"
    OVERALL_SUCCESS=false
fi
echo ""

# Run validate-packages.sh
echo ">>> Running package validation..."
echo ""
if bash "$SCRIPT_DIR/validate-packages.sh"; then
    echo ""
    echo "✓ Package validation passed"
else
    echo ""
    echo "✗ Package validation failed"
    OVERALL_SUCCESS=false
fi
echo ""

# Final summary
echo "=========================================="
if [ "$OVERALL_SUCCESS" = true ]; then
    echo "✓ All validations passed!"
    echo "=========================================="
    exit 0
else
    echo "✗ Some validations failed"
    echo "=========================================="
    echo ""
    echo "Note: If workspace is not built, run:"
    echo "  ./scripts/docker-shell.sh"
    echo "  cd /workspace"
    echo "  colcon build --symlink-install"
    echo "  exit"
    echo "  ./scripts/validate.sh"
    echo ""
    exit 1
fi
