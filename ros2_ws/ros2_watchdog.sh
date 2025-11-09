#!/bin/bash

# ROS2 Process Watchdog Script
# Monitors critical ROS2 nodes and services, restarting them if they fail
# Designed for maximum reliability with minimum downtime

set -e

# Configuration
WATCHDOG_LOG="/root/ros2_ws/logs/watchdog.log"
WATCHDOG_PID_FILE="/tmp/ros2_watchdog.pid"
MONITOR_INTERVAL=30  # seconds
MAX_RESTART_ATTEMPTS=3
RESTART_COOLDOWN=60  # seconds

# Critical ROS2 nodes to monitor
CRITICAL_NODES=(
    "robot_automation_server"
    "rest_api_server"
    "websocket_server"
    "n8n_ros2_bridge"
    "robot_status_server"
    "sensor_data_server"
)

# Service restart counters
declare -A restart_counts
declare -A last_restart_time

# Logging function
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [WATCHDOG] $*" | tee -a "$WATCHDOG_LOG"
}

# Check if a ROS2 node is running
check_node() {
    local node_name="$1"
    ros2 node list 2>/dev/null | grep -q "$node_name"
}

# Check if a service is responding
check_service() {
    local service_name="$1"
    ros2 service list 2>/dev/null | grep -q "$service_name"
}

# Check topic activity
check_topic_activity() {
    local topic="$1"
    local timeout="${2:-5}"
    timeout "$timeout" ros2 topic echo "$topic" --once 2>/dev/null | head -1 >/dev/null
}

# Restart a ROS2 launch file
restart_launch() {
    local launch_file="$1"
    log "Attempting to restart launch file: $launch_file"

    # Kill existing processes gracefully first
    pkill -TERM -f "$launch_file" || true
    sleep 5
    pkill -KILL -f "$launch_file" || true

    # Restart the launch
    if ros2 launch "$launch_file" > /dev/null 2>&1 & then
        log "Successfully restarted: $launch_file"
        return 0
    else
        log "ERROR: Failed to restart: $launch_file"
        return 1
    fi
}

# Restart a specific node
restart_node() {
    local node_name="$1"
    local package="$2"
    local executable="$3"

    log "Attempting to restart node: $node_name"

    # Kill existing node
    ros2 lifecycle set "$node_name" shutdown 2>/dev/null || true
    pkill -f "$node_name" || true
    sleep 2

    # Restart node
    if ros2 run "$package" "$executable" __node:="$node_name" > /dev/null 2>&1 & then
        log "Successfully restarted node: $node_name"
        return 0
    else
        log "ERROR: Failed to restart node: $node_name"
        return 1
    fi
}

# Memory usage check
check_memory_usage() {
    local threshold_mb="${1:-1024}"  # Default 1GB
    local current_mb=$(free -m | awk 'NR==2{printf "%.0f", $3}')

    if [ "$current_mb" -gt "$threshold_mb" ]; then
        log "WARNING: High memory usage detected: ${current_mb}MB (threshold: ${threshold_mb}MB)"
        return 1
    fi
    return 0
}

# CPU usage check
check_cpu_usage() {
    local threshold_percent="${1:-80}"  # Default 80%
    local current_percent=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')

    if (( $(echo "$current_percent > $threshold_percent" | bc -l) )); then
        log "WARNING: High CPU usage detected: ${current_percent}% (threshold: ${threshold_percent}%)"
        return 1
    fi
    return 0
}

# Health check function
perform_health_check() {
    log "Performing health check..."

    local issues_found=0

    # Check memory usage
    if ! check_memory_usage 1200; then  # 1.2GB threshold
        ((issues_found++))
    fi

    # Check CPU usage
    if ! check_cpu_usage 85; then  # 85% threshold
        ((issues_found++))
    fi

    # Check critical ROS2 nodes
    for node in "${CRITICAL_NODES[@]}"; do
        if ! check_node "$node"; then
            log "ERROR: Critical node not found: $node"
            ((issues_found++))

            # Check restart limits
            local current_time=$(date +%s)
            local last_restart=${last_restart_time[$node]:-0}
            local time_since_restart=$((current_time - last_restart))

            if [ "$time_since_restart" -lt "$RESTART_COOLDOWN" ]; then
                log "Skipping restart of $node - cooldown period (${RESTART_COOLDOWN}s)"
                continue
            fi

            # Check restart attempts
            local attempts=${restart_counts[$node]:-0}
            if [ "$attempts" -ge "$MAX_RESTART_ATTEMPTS" ]; then
                log "ERROR: Max restart attempts reached for $node (${MAX_RESTART_ATTEMPTS})"
                continue
            fi

            # Attempt restart based on node type
            case "$node" in
                "robot_automation_server")
                    if restart_node "$node" "my_robot_automation" "robot_automation_server.py"; then
                        restart_counts[$node]=$((attempts + 1))
                        last_restart_time[$node]=$current_time
                    fi
                    ;;
                "rest_api_server")
                    if restart_node "$node" "my_robot_automation" "rest_api_server.py"; then
                        restart_counts[$node]=$((attempts + 1))
                        last_restart_time[$node]=$current_time
                    fi
                    ;;
                "websocket_server")
                    if restart_node "$node" "my_robot_automation" "websocket_server.py"; then
                        restart_counts[$node]=$((attempts + 1))
                        last_restart_time[$node]=$current_time
                    fi
                    ;;
                "n8n_ros2_bridge")
                    if restart_node "$node" "my_robot_automation" "n8n_ros2_bridge.py"; then
                        restart_counts[$node]=$((attempts + 1))
                        last_restart_time[$node]=$current_time
                    fi
                    ;;
                *)
                    log "WARNING: Unknown node type for restart: $node"
                    ;;
            esac
        fi
    done

    # Check topic activity
    if ! check_topic_activity "/robot_status" 3; then
        log "WARNING: Robot status topic not active"
        ((issues_found++))
    fi

    if [ $issues_found -eq 0 ]; then
        log "Health check passed - all systems operational"
    else
        log "Health check found $issues_found issue(s)"
    fi

    return $issues_found
}

# Cleanup function
cleanup() {
    log "Watchdog shutting down..."
    rm -f "$WATCHDOG_PID_FILE"
    exit 0
}

# Signal handlers
trap cleanup SIGTERM SIGINT

# Main watchdog loop
main() {
    # Check if already running
    if [ -f "$WATCHDOG_PID_FILE" ]; then
        local existing_pid=$(cat "$WATCHDOG_PID_FILE")
        if kill -0 "$existing_pid" 2>/dev/null; then
            log "Watchdog already running (PID: $existing_pid)"
            exit 1
        else
            log "Removing stale PID file"
            rm -f "$WATCHDOG_PID_FILE"
        fi
    fi

    # Create PID file
    echo $$ > "$WATCHDOG_PID_FILE"

    # Create log directory
    mkdir -p "$(dirname "$WATCHDOG_LOG")"

    log "ROS2 Watchdog starting up..."
    log "Monitoring interval: ${MONITOR_INTERVAL}s"
    log "Max restart attempts: $MAX_RESTART_ATTEMPTS"
    log "Restart cooldown: ${RESTART_COOLDOWN}s"

    # Wait for ROS2 to be ready
    log "Waiting for ROS2 environment to be ready..."
    local wait_count=0
    while ! ros2 topic list >/dev/null 2>&1; do
        if [ $wait_count -ge 60 ]; then  # Wait up to 5 minutes
            log "ERROR: ROS2 environment not ready after 5 minutes"
            exit 1
        fi
        sleep 5
        ((wait_count++))
        log "Still waiting for ROS2... ($wait_count/60)"
    done

    log "ROS2 environment ready, starting monitoring..."

    # Main monitoring loop
    while true; do
        if ! perform_health_check; then
            log "Health check detected issues"
        fi

        sleep "$MONITOR_INTERVAL"
    done
}

# Run main function
main "$@"
