#!/bin/bash

#################################################################
# Autonomous Mobile Manipulator - Complete Management Script
# Handles Docker containers, ROS2 services, and system setup
#################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Configuration
CONTAINER_NAME="ros2_sim_container"
COMPOSE_FILE="docker-compose.yml"
PROJECT_NAME="autonomous_mobile_manipulator"

#################################################################
# Helper Functions
#################################################################

print_header() {
    echo -e "${BLUE}"
    echo "========================================================================="
    echo "  $1"
    echo "========================================================================="
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

#################################################################
# Prerequisites Check
#################################################################

check_prerequisites() {
    print_header "Checking Prerequisites"
    
    local all_ok=true
    
    # Check Docker
    if command -v docker &> /dev/null; then
        print_success "Docker is installed ($(docker --version | cut -d' ' -f3))"
    else
        print_error "Docker is not installed"
        all_ok=false
    fi
    
    # Check Docker Compose
    if docker compose version &> /dev/null; then
        print_success "Docker Compose is installed"
    elif command -v docker-compose &> /dev/null; then
        print_success "Docker Compose is installed (standalone)"
    else
        print_error "Docker Compose is not installed"
        all_ok=false
    fi
    
    # Check if user is in docker group
    if groups | grep -q docker; then
        print_success "User is in docker group"
    else
        print_warning "User is not in docker group (may need sudo)"
    fi
    
    # Check if Docker daemon is running
    if docker ps &> /dev/null; then
        print_success "Docker daemon is running"
    else
        print_error "Docker daemon is not running"
        all_ok=false
    fi
    
    echo ""
    if [ "$all_ok" = false ]; then
        print_error "Prerequisites check failed. Please install missing components."
        return 1
    fi
    
    return 0
}

#################################################################
# Container Management
#################################################################

check_container_status() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "running"
        else
            echo "stopped"
        fi
    else
        echo "not_found"
    fi
}

start_containers() {
    local mode="${1:-prod}"
    
    print_header "Starting Autonomous Mobile Manipulator"
    
    if [ "$mode" = "dev" ]; then
        print_info "Mode: Development (with sensor simulation)"
        COMPOSE_FILE="docker-compose.dev.yml"
    else
        print_info "Mode: Production (with real hardware/Gazebo)"
        COMPOSE_FILE="docker-compose.yml"
    fi
    
    # Check if container is already running
    local status=$(check_container_status)
    if [ "$status" = "running" ]; then
        print_warning "Container is already running"
        print_info "Use './run.sh restart' to restart or './run.sh stop' to stop"
        return 0
    fi
    
    # Start containers
    print_info "Starting Docker containers..."
    if [ -f "$COMPOSE_FILE" ]; then
        docker compose -f "$COMPOSE_FILE" -p "$PROJECT_NAME" up -d
        print_success "Containers started"
    else
        print_error "Compose file not found: $COMPOSE_FILE"
        return 1
    fi
    
    # Wait for services to be ready
    print_info "Waiting for services to initialize..."
    sleep 5
    
    # Check if ROS2 container is running
    if docker ps | grep -q "$CONTAINER_NAME"; then
        print_success "ROS2 container is running"
        
        # Check web interface
        print_info "Checking web interface..."
        sleep 2
        if curl -s http://localhost:8000/health &> /dev/null; then
            print_success "Web interface is accessible"
        else
            print_warning "Web interface may still be starting up"
        fi
    else
        print_error "ROS2 container failed to start"
        return 1
    fi
    
    echo ""
    display_access_info "$mode"
}

stop_containers() {
    print_header "Stopping Autonomous Mobile Manipulator"
    
    local status=$(check_container_status)
    if [ "$status" = "not_found" ] || [ "$status" = "stopped" ]; then
        print_warning "Containers are not running"
        return 0
    fi
    
    print_info "Stopping Docker containers..."
    docker compose -f "$COMPOSE_FILE" -p "$PROJECT_NAME" down
    print_success "Containers stopped"
}

restart_containers() {
    print_header "Restarting Autonomous Mobile Manipulator"
    stop_containers
    sleep 2
    start_containers "$1"
}

show_status() {
    print_header "System Status"
    
    local status=$(check_container_status)
    
    echo -e "${BLUE}Container Status:${NC}"
    if [ "$status" = "running" ]; then
        print_success "ROS2 Container: Running"
    elif [ "$status" = "stopped" ]; then
        print_warning "ROS2 Container: Stopped"
    else
        print_error "ROS2 Container: Not Found"
    fi
    
    echo ""
    echo -e "${BLUE}All Containers:${NC}"
    docker ps -a --filter "name=$PROJECT_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    
    echo ""
    echo -e "${BLUE}Resource Usage:${NC}"
    docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}" $(docker ps -q --filter "name=$PROJECT_NAME")
    
    echo ""
    echo -e "${BLUE}Services:${NC}"
    check_service_health
}

check_service_health() {
    # Check web interface
    if curl -s http://localhost:8000/health &> /dev/null; then
        print_success "Web Interface (port 8000): Accessible"
    else
        print_error "Web Interface (port 8000): Not accessible"
    fi
    
    # Check if n8n is running
    if curl -s http://localhost:5678 &> /dev/null; then
        print_success "n8n Automation (port 5678): Accessible"
    else
        print_warning "n8n Automation (port 5678): Not running"
    fi
}

show_logs() {
    local service="${1:-all}"
    
    print_header "Container Logs"
    
    if [ "$service" = "all" ]; then
        print_info "Showing logs for all containers (Ctrl+C to exit)"
        docker compose -p "$PROJECT_NAME" logs -f --tail=50
    else
        print_info "Showing logs for $service (Ctrl+C to exit)"
        docker logs -f "$service" 2>&1
    fi
}

exec_shell() {
    print_header "Entering ROS2 Container Shell"
    
    local status=$(check_container_status)
    if [ "$status" != "running" ]; then
        print_error "Container is not running. Start it first with: ./run.sh start"
        return 1
    fi
    
    print_info "Entering interactive shell..."
    docker exec -it "$CONTAINER_NAME" /bin/bash
}

display_access_info() {
    local mode="${1:-prod}"
    
    print_header "Access Information"
    
    # Get local IP
    LOCAL_IP=$(hostname -I | awk '{print $1}')
    
    echo -e "${GREEN}Robot System is Ready!${NC}"
    echo ""
    echo -e "${BLUE}Web Interfaces:${NC}"
    echo "  Primary Dashboard: http://localhost:8000"
    echo "  Remote Access:     http://${LOCAL_IP}:8000"
    echo ""
    echo -e "${BLUE}n8n Automation:${NC}"
    echo "  Local:  http://localhost:5678"
    echo "  Remote: http://${LOCAL_IP}:5678"
    echo ""
    
    if [ "$mode" = "dev" ]; then
        echo -e "${YELLOW}Development Mode Features:${NC}"
        echo "  • Simulated sensor data"
        echo "  • Fast startup (no Gazebo)"
        echo "  • Testing and development"
        echo "  • IMU, distance sensors, line sensors"
    else
        echo -e "${GREEN}Production Mode Features:${NC}"
        echo "  • Real hardware sensors"
        echo "  • MPU6050 IMU (I2C 0x68)"
        echo "  • Sharp IR distance sensors (SPI/MCP3008)"
        echo "  • Full robot control"
    fi
    
    echo ""
    echo -e "${BLUE}Useful Commands:${NC}"
    echo "  View logs:    ./run.sh logs"
    echo "  Check status: ./run.sh status"
    echo "  Enter shell:  ./run.sh shell"
    echo "  Restart:      ./run.sh restart"
    echo "  Stop:         ./run.sh stop"
}

clean_system() {
    print_header "Cleaning System"
    
    print_warning "This will remove all containers, volumes, and images"
    read -p "Are you sure? (yes/no): " confirm
    
    if [ "$confirm" != "yes" ]; then
        print_info "Cleanup cancelled"
        return 0
    fi
    
    print_info "Stopping containers..."
    docker compose -p "$PROJECT_NAME" down -v
    
    print_info "Removing unused Docker resources..."
    docker system prune -f
    
    print_success "System cleaned"
}

update_system() {
    print_header "Updating System"
    
    print_info "Pulling latest changes from Git..."
    if [ -d ".git" ]; then
        git pull
        print_success "Git repository updated"
    else
        print_warning "Not a Git repository"
    fi
    
    print_info "Rebuilding Docker images..."
    docker compose -p "$PROJECT_NAME" build --no-cache
    
    print_success "System updated. Restart with: ./run.sh restart"
}

show_help() {
    cat << EOF
Autonomous Mobile Manipulator - Management Script

USAGE:
    ./run.sh [COMMAND] [OPTIONS]

COMMANDS:
    start [--dev]   Start the robot system
                    --dev: Development mode with simulated sensors
    
    stop            Stop all containers
    
    restart [--dev] Restart the system
    
    status          Show system status and resource usage
    
    logs [service]  Show logs (tail -f)
                    service: Container name (optional, default: all)
    
    shell           Enter ROS2 container shell
    
    check           Check prerequisites
    
    clean           Remove all containers and volumes
    
    update          Update system and rebuild images
    
    help            Show this help message

EXAMPLES:
    ./run.sh start              # Start in production mode
    ./run.sh start --dev        # Start in development mode
    ./run.sh logs               # Show all logs
    ./run.sh logs $CONTAINER_NAME  # Show ROS2 container logs
    ./run.sh shell              # Enter container shell
    ./run.sh restart --dev      # Restart in dev mode

ACCESS:
    Web Interface:    http://localhost:8000
    n8n Automation:   http://localhost:5678

DOCUMENTATION:
    Setup Guide:      ros2_ws/src/my_robot_automation/SETUP_GUIDE.md
    API Docs:         ros2_ws/src/my_robot_automation/API_DOCUMENTATION.md
    Sensor Wiring:    ros2_ws/src/my_robot_automation/SENSOR_WIRING.md
    MPU6050 Setup:    ros2_ws/src/my_robot_automation/MPU6050_SETUP.md

EOF
}

#################################################################
# Main Script
#################################################################

main() {
    local command="${1:-help}"
    local option="${2}"
    
    case "$command" in
        start)
            check_prerequisites || exit 1
            if [ "$option" = "--dev" ] || [ "$option" = "-d" ]; then
                start_containers "dev"
            else
                start_containers "prod"
            fi
            ;;
        
        stop)
            stop_containers
            ;;
        
        restart)
            if [ "$option" = "--dev" ] || [ "$option" = "-d" ]; then
                restart_containers "dev"
            else
                restart_containers "prod"
            fi
            ;;
        
        status)
            show_status
            ;;
        
        logs)
            show_logs "$option"
            ;;
        
        shell)
            exec_shell
            ;;
        
        check)
            check_prerequisites
            ;;
        
        clean)
            clean_system
            ;;
        
        update)
            update_system
            ;;
        
        help|--help|-h)
            show_help
            ;;
        
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
