#!/usr/bin/env bash

# Build script for Mycelix ROS2 Bridge
# Builds all executables and libraries for the robotic consciousness network

set -e  # Exit on error

echo "🏗️  Building Mycelix-Holochain ROS2 Bridge..."
echo "================================================"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 environment not sourced!"
    echo "   Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 $ROS_DISTRO environment detected"

# Clean previous build if requested
if [ "$1" == "clean" ]; then
    echo "🧹 Cleaning previous build..."
    rm -rf build install log
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo ""
echo "⚙️  Configuring CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo ""
echo "🔨 Building executables and libraries..."
make -j$(nproc)

# Return to root directory
cd ..

echo ""
echo "✅ Build complete!"
echo ""
echo "📦 Built executables:"
echo "  • mycelix_bridge_node - Main bridge to Holochain"
echo "  • swarm_coordinator   - Multi-agent coordination"
echo "  • consciousness_visualizer - Real-time field visualization"
echo "  • robot_simulator     - Robot movement simulation"
echo "  • demo_controller     - Demo orchestration"
echo ""
echo "📚 Built libraries:"
echo "  • holochain_agent_lib - Holochain WebSocket integration"
echo "  • federated_learner_lib - Privacy-preserving ML"
echo ""
echo "🚀 To run the demo:"
echo "   ./launch_demo.sh"
echo ""
echo "💡 To build with ROS2 tools (recommended):"
echo "   colcon build --packages-select mycelix_ros2_bridge"