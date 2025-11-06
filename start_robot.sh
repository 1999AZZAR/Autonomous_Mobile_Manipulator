#!/bin/bash

# Autonomous Mobile Manipulator - Startup Script
# This script provides an easy way to start the ROS2 simulation and n8n workflow automation

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
PROJECT_NAME="Autonomous Mobile Manipulator"
BUILD_FLAG=""
START_MODE="normal"
DEV_MODE=false

# Function to print colored output
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  --dev               Start in development mode (sensor simulation)"
    echo "  -b, --build         Force rebuild of Docker images"
    echo "  -d, --detach        Start services in detached mode (default)"
    echo "  -f, --foreground    Start services in foreground mode"
    echo "  -s, --status        Show current status of services"
    echo "  -l, --logs          Show logs from running services"
    echo "  -r, --restart       Restart all services"
    echo "  -x, --stop          Stop all services"
    echo ""
    echo "Examples:"
    echo "  $0                    # Start services (build if needed)"
    echo "  $0 --dev              # Start in development mode (sensor simulation)"
    echo "  $0 -b                 # Force rebuild and start"
    echo "  $0 --dev -b           # Force rebuild in development mode"
    echo "  $0 -s                 # Show status"
    echo "  $0 -x                 # Stop services"
    echo "  $0 -l                 # Show logs"
}

# Function to check prerequisites
check_prerequisites() {
    print_info "Checking prerequisites..."

    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi

    if ! docker info &> /dev/null; then
        print_error "Docker daemon is not running. Please start Docker."
        exit 1
    fi

    # Check if Docker Compose is available
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        print_error "Docker Compose is not available. Please install Docker Compose."
        exit 1
    fi

    print_success "Prerequisites check passed"
}

# Function to show system info
show_system_info() {
    echo "========================================"
    echo "      $PROJECT_NAME"
    echo "========================================"
    echo ""
    print_info "System Information:"
    echo "  Docker: $(docker --version)"
    if command -v docker-compose &> /dev/null; then
        echo "  Docker Compose: $(docker-compose --version)"
    else
        echo "  Docker Compose: $(docker compose version)"
    fi
    echo "  Project Directory: $(pwd)"
    echo ""
}

# Function to show status
show_status() {
    # Set container prefix based on dev mode
    if [ "$DEV_MODE" = true ]; then
        CONTAINER_PREFIX="ros2_dev_container"
    else
        CONTAINER_PREFIX="ros2_sim_container"
    fi

    echo "========================================"
    echo "         Service Status"
    echo "========================================"
    echo ""

    local compose_file="docker-compose.yml"
    if [ "$DEV_MODE" = true ]; then
        compose_file="docker-compose.dev.yml"
    fi

    if command -v docker-compose &> /dev/null; then
        docker-compose -f $compose_file ps
    else
        docker compose -f $compose_file ps
    fi

    echo ""
    echo "========================================"
    echo "         Access Information"
    echo "========================================"
    echo ""
    print_info "Services will be available at:"
    echo "  🌐 n8n Workflow Interface: http://localhost:5678"
    echo "  🤖 Robot Web Interface:    http://localhost:5000"
    echo "  🔌 WebSocket Server:       ws://localhost:8765"
    echo ""
    print_info "Container Access:"
    echo "  ROS2 Container: docker exec -it $CONTAINER_PREFIX bash"
    echo "  n8n Container:  docker exec -it n8n_container bash"
    echo ""
}

# Function to show logs
show_logs() {
    local compose_file="docker-compose.yml"
    if [ "$DEV_MODE" = true ]; then
        compose_file="docker-compose.dev.yml"
    fi

    echo "========================================"
    echo "            Service Logs"
    echo "========================================"
    echo ""

    if command -v docker-compose &> /dev/null; then
        docker-compose -f $compose_file logs -f --tail=50
    else
        docker compose -f $compose_file logs -f --tail=50
    fi
}

# Function to start services
start_services() {
    if [ "$DEV_MODE" = true ]; then
        echo "========================================"
        echo "    Starting Services (Development Mode)"
        echo "========================================"
        echo ""
        print_info "Development mode: Using sensor simulation instead of Gazebo"
        COMPOSE_FILE="docker-compose.dev.yml"
        CONTAINER_PREFIX="ros2_dev_container"
    else
        echo "========================================"
        echo "    Starting Services (Production Mode)"
        echo "========================================"
        echo ""
        print_info "Production mode: Full Gazebo simulation with physics"
        COMPOSE_FILE="docker-compose.yml"
        CONTAINER_PREFIX="ros2_sim_container"
    fi

    local cmd_flags="$BUILD_FLAG"

    if [ "$START_MODE" = "foreground" ]; then
        print_info "Starting services in foreground mode..."
        cmd_flags="$cmd_flags --remove-orphans"
    else
        print_info "Starting services in detached mode..."
        cmd_flags="$cmd_flags -d --remove-orphans"
    fi

    if command -v docker-compose &> /dev/null; then
        print_info "Using docker-compose..."
        docker-compose -f $COMPOSE_FILE up $cmd_flags
    else
        print_info "Using docker compose..."
        docker compose -f $COMPOSE_FILE up $cmd_flags
    fi
}

# Function to stop services
stop_services() {
    local compose_file="docker-compose.yml"
    if [ "$DEV_MODE" = true ]; then
        compose_file="docker-compose.dev.yml"
    fi

    print_info "Stopping all services..."
    if command -v docker-compose &> /dev/null; then
        docker-compose -f $compose_file down
    else
        docker compose -f $compose_file down
    fi
    print_success "Services stopped"
}

# Function to restart services
restart_services() {
    print_info "Restarting all services..."
    stop_services
    sleep 2
    start_services
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        --dev)
            DEV_MODE=true
            shift
            ;;
        -b|--build)
            BUILD_FLAG="--build"
            shift
            ;;
        -d|--detach)
            START_MODE="detach"
            shift
            ;;
        -f|--foreground)
            START_MODE="foreground"
            shift
            ;;
        -s|--status)
            check_prerequisites
            show_system_info
            show_status
            exit 0
            ;;
        -l|--logs)
            check_prerequisites
            show_logs
            exit 0
            ;;
        -r|--restart)
            check_prerequisites
            restart_services
            exit 0
            ;;
        -x|--stop)
            check_prerequisites
            stop_services
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Main execution
main() {
    check_prerequisites
    show_system_info

    # Default action is to start services
    start_services

    # Show status after starting
    sleep 5
    show_status

    if [ "$DEV_MODE" = true ]; then
        print_success "Development system started successfully!"
        print_info "Sensor simulation active - no Gazebo required"
    else
        print_success "Production system started successfully!"
        print_info "Full Gazebo simulation active"
    fi
    print_info "Use '$0 --status' to check service status"
    print_info "Use '$0 --logs' to view service logs"
    print_info "Use '$0 --stop' to stop all services"
}

# Run main function if no specific command was given
if [ "$START_MODE" = "normal" ] && [ -z "$BUILD_FLAG" ]; then
    main
fi
