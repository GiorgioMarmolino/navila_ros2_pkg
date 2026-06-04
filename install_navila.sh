#!/bin/bash
# =============================================================================
# install_navila.sh
# Setup script for NaVILA ROS2 environment
# Usage: source install_navila.sh  (oppure: . install_navila.sh)
# =============================================================================

# =============================================================================
# SOURCE ROS2 E WORKSPACE
# =============================================================================
source /opt/ros/humble/setup.bash
cd /home/ros_ws
source install/setup.bash

echo "[NaVILA] ROS2 environment sourced ✓"

# =============================================================================
# ALIAS
# =============================================================================

# Avvia il launch file NaVILA
# Uso: start_navila [sim:=true/false] [safety:=true/false]
# Esempi:
#   start_navila
#   start_navila sim:=false
#   start_navila sim:=true safety:=false
alias start_navila='_start_navila'
_start_navila() {
    local sim="true"
    local safety="true"

    for arg in "$@"; do
        case $arg in
            sim:=true)   sim="true"  ;;
            sim:=false)  sim="false" ;;
            safety:=true)  safety="true"  ;;
            safety:=false) safety="false" ;;
        esac
    done

    echo "[NaVILA] Avvio con sim=$sim safety=$safety"
    ros2 launch navila_ros2_bridge navila.launch.py \
        use_sim_time:=$sim \
        enable_safety:=$safety &
    NAVILA_PID=$!
    echo "[NaVILA] PID=$NAVILA_PID"
}

# Resetta il goal — il nodo torna in attesa
alias reset_navila='ros2 topic pub --once /navila/reset std_msgs/msg/Empty "{}" && echo "[NaVILA] Goal resettato"'

# Interrompe il launch file avviato con start_navila
alias kill_navila='_kill_navila'
_kill_navila() {
    if [ -n "$NAVILA_PID" ]; then
        echo "[NaVILA] Killing PID=$NAVILA_PID"
        kill $NAVILA_PID 2>/dev/null
        NAVILA_PID=""
    else
        echo "[NaVILA] Nessun processo trovato, provo con pkill..."
        pkill -f "navila_super_node" 2>/dev/null
        pkill -f "action_to_cmdvel" 2>/dev/null
        pkill -f "navila.launch.py"  2>/dev/null
    fi
    echo "[NaVILA] Terminato ✓"
}

# Pubblica un goal manualmente
# Uso: goal "vai dritto fino al muro"
alias goal='_send_goal'
_send_goal() {
    if [ -z "$1" ]; then
        echo "Uso: goal \"testo del goal\""
        return 1
    fi
    ros2 topic pub --once /goal_instruction std_msgs/msg/String "data: '$1'"
    echo "[NaVILA] Goal inviato: '$1'"
}

# Manda un comando manuale direttamente a cmd_vel
# Uso: move_robot 0.5 0.0
alias move_robot='_move_robot'
_move_robot() {
    local lin=${1:-0.5}
    local ang=${2:-0.0}
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $lin, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ang}}"
}

# Ferma il robot immediatamente
alias stop_robot='ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" && echo "[NaVILA] Robot fermo"'

# Mostra lo stato dei topic principali
alias navila_status='_navila_status'
_navila_status() {
    echo "=== NaVILA STATUS ==="
    echo ""
    echo "--- Nodi attivi ---"
    ros2 node list 2>/dev/null | grep -E "navila|action_to_cmd|instruction|bridge" || echo "  nessun nodo NaVILA trovato"
    echo ""
    echo "--- Topic frequenze ---"
    timeout 3 ros2 topic hz /navila/action    2>/dev/null | head -2 | sed 's/^/  \/navila\/action: /' &
    timeout 3 ros2 topic hz /cmd_vel          2>/dev/null | head -2 | sed 's/^/  \/cmd_vel: /'        &
    wait
    echo ""
    echo "--- ROS_DOMAIN_ID: $ROS_DOMAIN_ID ---"
}

# Mostra i log di sicurezza del LiDAR
alias safety_log='ros2 topic echo /rosout 2>/dev/null | grep -i "SAFETY\|BLOCK\|OBSTACLE\|TIMEOUT"'

# Mostra ultimo output di NaVILA
alias navila_log='ros2 topic echo /navila/action'

echo ""
echo "=== NaVILA ALIAS DISPONIBILI ==="
echo "  start_navila [sim:=true/false] [safety:=true/false]  — avvia il sistema"
echo "  kill_navila                                           — ferma il sistema"
echo "  reset_navila                                          — resetta il goal"
echo "  goal \"testo\"                                          — invia un goal"
echo "  move_robot [lin] [ang]                                — muovi il robot manualmente"
echo "  stop_robot                                            — ferma il robot"
echo "  navila_status                                         — stato del sistema"
echo "  safety_log                                            — log del safety layer"
echo "  navila_log                                            — output di NaVILA"
echo ""

# =============================================================================
# SPLIT TERMINALE (richiede tmux)
# =============================================================================
if command -v tmux &> /dev/null; then
    echo "[NaVILA] Avvio tmux con layout 3 pannelli..."
    tmux new-session -d -s navila 2>/dev/null || true

    # Crea layout: colonna sx divisa in 2, colonna dx intera
    tmux split-window -h -t navila        # divide in 2 colonne
    tmux split-window -v -t navila:0.0    # divide colonna sx in 2

    # Source in tutti i pannelli
    for pane in 0 1 2; do
        tmux send-keys -t navila:0.$pane \
            "source /opt/ros/humble/setup.bash && cd /home/ros_ws && source install/setup.bash && source install_navila.sh" Enter
    done

    tmux attach -t navila
else
    echo "[NaVILA] tmux non trovato — installa con: apt install tmux"
    echo "[NaVILA] Alias caricati nel terminale corrente ✓"
fi
