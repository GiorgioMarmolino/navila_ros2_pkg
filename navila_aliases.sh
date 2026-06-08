#!/bin/bash
# =============================================================================
# install_navila.sh
# Setup script for NaVILA ROS2 environment
# Usage: source install_navila.sh  (or: . install_navila.sh)
# =============================================================================

# =============================================================================
# SOURCE ROS2 AND WORKSPACE
# =============================================================================
source /opt/ros/humble/setup.bash
cd /home/ros_ws
source install/setup.bash

echo "[NaVILA] ROS2 environment sourced ✓"

# =============================================================================
# ALIASES
# =============================================================================

# Launch the NaVILA system
# Usage: start_navila [sim:=true/false] [safety:=true/false]
# Examples:
#   start_navila
#   start_navila sim:=false
#   start_navila sim:=true safety:=false
alias start_navila='_start_navila'
_start_navila() {
    local sim="sim"
    local safety="true"

    for arg in "$@"; do
        case $arg in
            sim:=true)     sim="sim"    ;;
            sim:=false)    sim="lab"   ;;
            safety:=true)  safety="true"  ;;
            safety:=false) safety="false" ;;
        esac
    done

    echo "[NaVILA] Starting with sim=$sim safety=$safety"
    ros2 launch navila_ros2_bridge bringup.launch.py \
        env:=$sim \
        enable_safety:=$safety &
    NAVILA_PID=$!
    echo $NAVILA_PID > /tmp/navila.pid
    echo "[NaVILA] PID=$NAVILA_PID"
}

# Reset the goal — node returns to waiting for a new goal
alias reset_navila='ros2 topic pub --once /navila/reset std_msgs/msg/Empty "{}" && echo "[NaVILA] Goal reset"'

# Stop the launch file started with start_navila
alias kill_navila='_kill_navila'
_kill_navila() {
    if [ -f /tmp/navila.pid ]; then
        NAVILA_PID=$(cat /tmp/navila.pid)
        echo "[NaVILA] Killing PID=$NAVILA_PID"
        kill $NAVILA_PID 2>/dev/null
        rm /tmp/navila.pid
        NAVILA_PID=""
    fi

    # Loop fino a quando tutti i nodi sono morti
    local max_attempts=10
    local attempt=0

    while [ $attempt -lt $max_attempts ]; do
        # Prova a killare tutti
        pkill -f "bringup.launch.py"          2>/dev/null || true
        pkill -f "navila_super_node"          2>/dev/null || true
        pkill -f "action_to_cmdvel_node"      2>/dev/null || true
        pkill -f "action_node"                2>/dev/null || true
        pkill -f "instruction_node"           2>/dev/null || true
        pkill -f "goal_instruction"           2>/dev/null || true
        pkill -f "pointcloud_to_laserscan"    2>/dev/null || true
        pkill -f "safety_layer_node"          2>/dev/null || true

        sleep 0.8

        remaining=$(ros2 node list 2>/dev/null | grep -E "action_node|goal_instruction|pointcloud|safety_layer|navila" | wc -l)

        if [ "$remaining" -eq 0 ]; then
            echo "[NaVILA] Stopped ✓"
            navila_status
            return 0
        fi

        attempt=$((attempt + 1))
        echo "[NaVILA] $remaining nodes still active, attempt $attempt/$max_attempts..."

        # Dopo il 5° tentativo usa -9
        if [ $attempt -ge 5 ]; then
            pkill -9 -f "navila_super_node"       2>/dev/null || true
            pkill -9 -f "action_node"             2>/dev/null || true
            pkill -9 -f "goal_instruction"        2>/dev/null || true
            pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
            pkill -9 -f "safety_layer_node"       2>/dev/null || true
        fi
    done

    echo "[NaVILA] Warning: some nodes may still be active after $max_attempts attempts"
    sleep 3.0
    navila_status
    
}

# Send a text goal to the NaVILA node
# Usage: goal "go straight to the end of the corridor"
alias goal='_send_goal'
_send_goal() {
    if [ -z "$1" ]; then
        echo "Usage: goal \"goal text\""
        return 1
    fi
    ros2 topic pub --once /goal_instruction std_msgs/msg/String "data: '$1'"
    echo "[NaVILA] Goal sent: '$1'"
}

# Move the robot manually
# Usage: move_robot [linear] [angular]
alias move_robot='_move_robot'
_move_robot() {
    local lin=${1:-0.5}
    local ang=${2:-0.0}
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $lin, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ang}}"
}

# Stop the robot immediately
alias stop_navila='ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" && echo "[NaVILA] Robot stopped"'

# Show status of main nodes and topics
alias navila_status='_navila_status'
_navila_status() {
    echo "=== NaVILA STATUS ==="
    echo ""
    echo "--- Active nodes ---"
    ros2 node list 2>/dev/null || echo "  no NaVILA nodes found"
    echo ""
    echo "--- Active topics ---"
    ros2 topic list 2>/dev/null || echo "  no topics found"
    echo ""
    echo "--- ROS_DOMAIN_ID: $ROS_DOMAIN_ID ---"
}

# Show safety layer logs in real time
alias safety_log='ros2 topic echo /rosout 2>/dev/null | grep -i "SAFETY\|BLOCK\|OBSTACLE\|TIMEOUT"'

# Show NaVILA action output in real time
alias navila_log='ros2 topic echo /navila/action'

# Show velocity commands sent to the robot in real time
alias cmd_vel_log='ros2 topic echo /cmd_vel'

# Show bridge config
alias bridge_config='cat /home/ros_ws/bridge_config.yaml'

# Rebuild the NaVILA package
alias build_navila='cd /home/ros_ws && rm -rf build log install &&  colcon build --symlink-install && source install/setup.bash && echo "[NaVILA] Build complete ✓"'

alias navila_help='_navila_help'
_navila_help() {
    echo ""
    echo "=== AVAILABLE ALIASES ==="
    echo "  start_navila [sim:=true/false] [safety:=true/false]   — launch the system"
    echo "  kill_navila                                           — stop the system"
    echo "  reset_navila                                          — reset the goal"
    echo "  goal \"text\"                                         — send a goal"
    echo "  move_robot [lin] [ang]                                — move robot manually"
    echo "  stop_navila                                           — stop the robot"
    echo "  navila_status                                         — show system status"
    echo "  safety_log                                            — safety layer logs"
    echo "  navila_log                                            — NaVILA action output"
    echo "  cmd_vel_log                                           — velocity commands"
    echo "  bridge_config                                         — show bridge config"
    echo "  build_navila                                          — rebuild the package"
    echo "  navila_help                                           — show this help"
    echo ""
}

echo ""
echo "=== AVAILABLE ALIASES ==="
echo "  start_navila [sim:=true/false] [safety:=true/false]   — launch the system"
echo "  kill_navila                                           — stop the system"
echo "  reset_navila                                          — reset the goal"
echo "  goal \"text\"                                         — send a goal"
echo "  move_robot [lin] [ang]                                — move robot manually"
echo "  stop_navila                                           — stop the robot and model inference"
echo "  navila_status                                         — show system status"
echo "  safety_log                                            — safety layer logs"
echo "  navila_log                                            — NaVILA action output"
echo "  cmd_vel_log                                           — velocity commands"
echo "  bridge_config                                         — show bridge config"
echo "  build_navila                                          — rebuild the package"
echo ""
