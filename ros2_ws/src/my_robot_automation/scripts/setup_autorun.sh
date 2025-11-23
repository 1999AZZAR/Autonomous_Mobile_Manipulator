#!/bin/bash

# Auto-run setup script for Autonomous Mobile Manipulator
# Sets up systemd service for automatic startup on boot

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="autonomous-mobile-manipulator"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
WORKING_DIR="$(dirname "$SCRIPT_DIR")"
MAIN_SCRIPT="$SCRIPT_DIR/main.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Autonomous Mobile Manipulator Auto-run${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        print_error "This script must be run as root (sudo)"
        echo "Usage: sudo $0 [options]"
        exit 1
    fi
}

create_service_file() {
    cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Autonomous Mobile Manipulator Control System
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=raspi
Group=raspi
WorkingDirectory=/tmp
ExecStart=/usr/bin/python3 $MAIN_SCRIPT
ExecReload=/bin/kill -HUP \$MAINPID
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=amm-control

# Security settings
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ReadWritePaths=$WORKING_DIR
ProtectHome=read-only

# Environment
Environment=PYTHONPATH=$WORKING_DIR
Environment=ROS_DOMAIN_ID=42

[Install]
WantedBy=multi-user.target
EOF
}

enable_autorun() {
    print_info "Enabling auto-run on boot..."

    # Check if main.py exists
    if [[ ! -f "$MAIN_SCRIPT" ]]; then
        print_error "Main script not found: $MAIN_SCRIPT"
        exit 1
    fi

    # Create service file
    print_info "Creating systemd service file..."
    create_service_file

    # Reload systemd
    print_info "Reloading systemd daemon..."
    systemctl daemon-reload

    # Enable service
    print_info "Enabling service..."
    systemctl enable "$SERVICE_NAME"

    # Start service
    print_info "Starting service..."
    systemctl start "$SERVICE_NAME"

    # Check status
    sleep 2
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        print_success "Auto-run enabled successfully!"
        echo ""
        print_info "Service Status:"
        systemctl status "$SERVICE_NAME" --no-pager -l | head -10
        echo ""
        print_info "To check logs: journalctl -u $SERVICE_NAME -f"
        print_info "To stop service: sudo systemctl stop $SERVICE_NAME"
    else
        print_error "Failed to start service"
        echo ""
        print_info "Check service status: sudo systemctl status $SERVICE_NAME"
        print_info "Check logs: sudo journalctl -u $SERVICE_NAME -n 50"
        exit 1
    fi
}

disable_autorun() {
    print_info "Disabling auto-run..."

    # Stop service if running
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        print_info "Stopping service..."
        systemctl stop "$SERVICE_NAME"
    fi

    # Disable service
    if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        print_info "Disabling service..."
        systemctl disable "$SERVICE_NAME"
    fi

    # Remove service file
    if [[ -f "$SERVICE_FILE" ]]; then
        print_info "Removing service file..."
        rm -f "$SERVICE_FILE"
        systemctl daemon-reload
    fi

    print_success "Auto-run disabled successfully!"
}

status_autorun() {
    echo ""
    print_info "Auto-run Status:"
    echo ""

    if [[ -f "$SERVICE_FILE" ]]; then
        echo -e "${GREEN}Service file exists: $SERVICE_FILE${NC}"

        if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
            echo -e "${GREEN}Service is enabled${NC}"
        else
            echo -e "${YELLOW}Service is disabled${NC}"
        fi

        if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
            echo -e "${GREEN}Service is running${NC}"
            echo ""
            print_info "Service Details:"
            systemctl status "$SERVICE_NAME" --no-pager | head -5
        else
            echo -e "${RED}Service is not running${NC}"
        fi
    else
        echo -e "${RED}Service file does not exist${NC}"
        echo -e "${YELLOW}Auto-run is not configured${NC}"
    fi

    echo ""
    print_info "Recent Logs:"
    journalctl -u "$SERVICE_NAME" -n 5 --no-pager 2>/dev/null || echo "No logs available"
}

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --enable          Enable auto-run on boot"
    echo "  --disable         Disable auto-run on boot"
    echo "  --status          Show current auto-run status"
    echo "  --restart         Restart the auto-run service"
    echo "  --logs            Show recent service logs"
    echo "  --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  sudo $0 --enable     # Enable auto-run"
    echo "  sudo $0 --disable    # Disable auto-run"
    echo "  sudo $0 --status     # Check status"
    echo "  sudo $0 --logs       # View logs"
}

# Main script logic
print_header

case "${1:-}" in
    --enable)
        check_root
        enable_autorun
        ;;
    --disable)
        check_root
        disable_autorun
        ;;
    --status)
        status_autorun
        ;;
    --restart)
        check_root
        print_info "Restarting service..."
        systemctl restart "$SERVICE_NAME"
        sleep 2
        if systemctl is-active --quiet "$SERVICE_NAME"; then
            print_success "Service restarted successfully"
        else
            print_error "Failed to restart service"
        fi
        ;;
    --logs)
        print_info "Recent service logs:"
        echo ""
        journalctl -u "$SERVICE_NAME" -n 20 --no-pager -f || echo "Service logs not available"
        ;;
    --help|-h)
        show_help
        ;;
    *)
        print_error "Invalid option: $1"
        echo ""
        show_help
        exit 1
        ;;
esac

echo ""
echo -e "${BLUE}========================================${NC}"
