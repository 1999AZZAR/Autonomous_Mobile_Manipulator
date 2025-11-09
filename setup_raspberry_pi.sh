#!/bin/bash
# Autonomous Mobile Manipulator - Raspberry Pi Setup Script
# This script automates the Raspberry Pi configuration for the Autonomous Mobile Manipulator

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Raspberry Pi
check_raspberry_pi() {
    if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
        log_error "This script is designed for Raspberry Pi. Detected: $(uname -m)"
        exit 1
    fi
    log_success "Raspberry Pi detected"
}

# Update system packages
update_system() {
    log_info "Updating system packages..."
    sudo apt update && sudo apt upgrade -y
    sudo apt autoremove -y && sudo apt clean
    log_success "System updated"
}

# Configure system settings
configure_system() {
    log_info "Configuring system settings..."

    # Set hostname
    sudo hostnamectl set-hostname lks-robot

    # Configure timezone
    sudo timedatectl set-timezone Asia/Jakarta

    # Enable required interfaces using direct config file modification for Ubuntu compatibility
    log_info "Configuring hardware interfaces..."

    # Check if we're on Ubuntu and modify config.txt directly
    if [[ -f /boot/firmware/config.txt ]]; then
        CONFIG_FILE="/boot/firmware/config.txt"

        # Enable I2C
        if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
            echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE"
        fi

        # Enable SPI
        if ! grep -q "^dtparam=spi=on" "$CONFIG_FILE"; then
            echo "dtparam=spi=on" | sudo tee -a "$CONFIG_FILE"
        fi

        # Enable UART
        if ! grep -q "^enable_uart=1" "$CONFIG_FILE"; then
            echo "enable_uart=1" | sudo tee -a "$CONFIG_FILE"
        fi

        # Enable camera
        if ! grep -q "^start_x=1" "$CONFIG_FILE"; then
            echo "start_x=1" | sudo tee -a "$CONFIG_FILE"
            echo "gpu_mem=128" | sudo tee -a "$CONFIG_FILE"
        fi

        log_info "Hardware interfaces configured via config.txt"
    else
        # Fallback to raspi-config for Raspberry Pi OS
        log_warning "config.txt not found, trying raspi-config..."
        sudo raspi-config nonint do_i2c 0
        sudo raspi-config nonint do_spi 0
        sudo raspi-config nonint do_serial 0  # Use single do_serial function for Ubuntu
        sudo raspi-config nonint do_camera 0
    fi

    log_success "System configured"
}

# Install system dependencies
install_dependencies() {
    log_info "Installing system dependencies..."

    sudo apt install -y \
        curl \
        wget \
        git \
        htop \
        vim \
        net-tools \
        i2c-tools \
        python3-pip \
        python3-dev \
        build-essential \
        libi2c-dev \
        libgpiod-dev \
        gpiod \
        python3-pigpio \
        python3-rpi.gpio \
        python3-lgpio \
        python3-gpiozero \
        ca-certificates \
        gnupg \
        lsb-release \
        uidmap \
        dbus-user-session \
        python3-numpy \
        python3-smbus

    # Install RTIMULib from pip since it's not in Ubuntu repos
    python3 -m pip install RTIMULib --break-system-packages || log_warning "RTIMULib installation failed, may need manual installation"

    # Install picamera2 from pip for Ubuntu 24.04
    python3 -m pip install picamera2 --break-system-packages || log_warning "picamera2 installation failed, camera functionality may be limited"

    log_success "Dependencies installed"
}

# Install and configure Docker
setup_docker() {
    log_info "Setting up Docker..."

    # Add Docker's official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

    # Set up the stable repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    # Install Docker Engine
    sudo apt update
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

    # Start and enable Docker
    sudo systemctl start docker
    sudo systemctl enable docker

    # Add user to docker group
    sudo usermod -aG docker $USER

    # Configure Docker for ARM64
    sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "experimental": true,
  "features": {
    "buildkit": true
  },
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2",
  "default-runtime": "runc"
}
EOF

    # Restart Docker
    sudo systemctl restart docker

    log_success "Docker configured"
}

# Configure GPIO and hardware interfaces
configure_hardware() {
    log_info "Configuring hardware interfaces..."

    # Add user to required groups
    sudo usermod -aG dialout,i2c,gpio,video $USER

    # Enable lingering for systemd user services
    sudo loginctl enable-linger $USER

    # Create GPIO permission script
    sudo tee /usr/local/bin/setup-gpio-permissions > /dev/null <<EOF
#!/bin/bash
# Set GPIO permissions for user access
sudo chown root:gpio /dev/gpiomem 2>/dev/null || true
sudo chmod g+rw /dev/gpiomem 2>/dev/null || true
sudo chown root:i2c /dev/i2c-1 2>/dev/null || true
sudo chmod g+rw /dev/i2c-1 2>/dev/null || true
EOF
    sudo chmod +x /usr/local/bin/setup-gpio-permissions

    # Add to startup
    sudo tee /etc/rc.local > /dev/null <<EOF
#!/bin/bash
/usr/local/bin/setup-gpio-permissions
exit 0
EOF
    sudo chmod +x /etc/rc.local

    log_success "Hardware interfaces configured"
}

# Configure network settings
configure_network() {
    log_info "Configuring network settings..."

    # Configure firewall
    sudo apt install -y ufw
    sudo ufw default deny incoming
    sudo ufw default allow outgoing
    sudo ufw allow ssh
    sudo ufw allow 5000  # Robot API
    sudo ufw allow 5678  # n8n interface
    sudo ufw allow 8765  # WebSocket
    echo "y" | sudo ufw enable

    log_success "Network configured"
}

# Configure performance optimizations
optimize_performance() {
    log_info "Applying performance optimizations..."

    # Configure swap
    sudo fallocate -l 2G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile

    # Make swap permanent
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi

    # Configure swappiness
    sudo tee /etc/sysctl.d/99-swap.conf > /dev/null <<EOF
vm.swappiness=10
EOF

    # Configure CPU governor
    sudo apt install -y cpufrequtils
    sudo tee /etc/default/cpufrequtils > /dev/null <<EOF
GOVERNOR="performance"
EOF

    log_success "Performance optimized"
}

# Test hardware functionality
test_hardware() {
    log_info "Testing hardware functionality..."

    # Test GPIO
    if python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO OK')" 2>/dev/null; then
        log_success "GPIO test passed"
    else
        log_warning "GPIO test failed - may need manual configuration"
    fi

    # Test I2C
    if sudo i2cdetect -y 1 &>/dev/null; then
        log_success "I2C test passed"
    else
        log_warning "I2C test failed - check connections"
    fi

    # Test Docker
    if docker run --rm hello-world &>/dev/null; then
        log_success "Docker test passed"
    else
        log_error "Docker test failed"
        return 1
    fi

    log_success "Hardware tests completed"
}

# Clone and setup project
setup_project() {
    log_info "Setting up Autonomous Mobile Manipulator project..."

    # Clone the project if it doesn't exist
    if [ ! -d "/home/$USER/Autonomous_Mobile_Manipulator" ]; then
        log_info "Cloning project repository..."
        git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git /home/$USER/Autonomous_Mobile_Manipulator
        log_success "Project cloned successfully"
    else
        log_info "Project directory already exists, pulling latest changes..."
        cd /home/$USER/Autonomous_Mobile_Manipulator
        git pull
        log_success "Project updated"
    fi

    # Change to project directory
    cd /home/$USER/Autonomous_Mobile_Manipulator

    # Make workflow management tools executable
    if [ -f "workflow_management_tools.sh" ]; then
        chmod +x workflow_management_tools.sh
        log_success "Workflow management tools ready"
    else
        log_warning "workflow_management_tools.sh not found in project directory"
    fi

    log_success "Project setup completed"
}

# Setup and import workflows
setup_workflows() {
    log_info "Setting up N8N workflows..."

    cd /home/$USER/Autonomous_Mobile_Manipulator

    # Wait for services to be fully ready
    log_info "Waiting for services to be ready..."
    local attempts=0
    local max_attempts=60

    while [ $attempts -lt $max_attempts ]; do
        if curl -s --max-time 5 http://localhost:5678 >/dev/null 2>&1 && \
           curl -s --max-time 5 http://localhost:5000/health >/dev/null 2>&1; then
            log_success "All services are responding"
            break
        fi

        if [ $((attempts % 10)) -eq 0 ]; then
            log_info "Still waiting for services... ($attempts/$max_attempts)"
        fi

        sleep 5
        attempts=$((attempts + 1))
    done

    if [ $attempts -eq $max_attempts ]; then
        log_error "Services failed to start properly after $max_attempts attempts"
        return 1
    fi

    # Run workflow management tools for safe import
    if [ -f "workflow_management_tools.sh" ]; then
        log_info "Running workflow management tools..."
        ./workflow_management_tools.sh safe-import

        if [ $? -eq 0 ]; then
            log_success "Workflows imported successfully"
        else
            log_warning "Workflow import completed with warnings - check logs above"
        fi
    else
        log_error "workflow_management_tools.sh not found"
        return 1
    fi

    log_success "Workflow setup completed"
}

# Complete post-reboot setup
complete_setup() {
    log_info "Completing post-reboot setup..."

    # Run project setup
    setup_project

    # Start the robot services
    log_info "Starting robot services..."
    cd /home/$USER/Autonomous_Mobile_Manipulator

    # Start services in production mode
    docker compose -f docker-compose.prod.yml up -d

    if [ $? -eq 0 ]; then
        log_success "Robot services started successfully"
    else
        log_error "Failed to start robot services"
        return 1
    fi

    # Setup workflows
    setup_workflows

    # Final verification
    log_info "Running final health checks..."
    if [ -f "/home/$USER/health_check.sh" ]; then
        /home/$USER/health_check.sh
        if [ $? -eq 0 ]; then
            log_success "All health checks passed!"
        else
            log_warning "Some health checks failed - manual verification recommended"
        fi
    fi

    log_success "🎉 Complete Autonomous Mobile Manipulator setup finished!"
    log_info ""
    log_info "System is ready and fully operational:"
    log_info "   - n8n: http://localhost:5678"
    log_info "   - Robot API: http://localhost:5000"
    log_info "   - WebSocket: ws://localhost:8765"
    log_info ""
    log_info "Available monitoring tools:"
    log_info "   ./system_monitor.sh        - System status"
    log_info "   ./health_check.sh          - Service health"
    log_info "   ./ros2_log_maintenance.sh  - Log management"
    log_info ""
    log_info "Workflow management:"
    log_info "   ./workflow_management_tools.sh help  - Available commands"
    log_info ""
    log_info "ROS2 Reliability Features:"
    log_info "   ✓ Automatic watchdog monitoring (30s intervals)"
    log_info "   ✓ Health checks with auto-recovery"
    log_info "   ✓ Resource limits and monitoring"
    log_info "   ✓ Structured logging and rotation"
    log_info "   ✓ DDS optimization for reliability"
}

# Create systemd service
create_service() {
    log_info "Creating systemd service..."

    # Create systemd service for robot deployment
    sudo tee /etc/systemd/system/lks-robot.service > /dev/null <<EOF
[Unit]
Description=Autonomous Mobile Manipulator
After=docker.service network.target
Requires=docker.service
ConditionPathExists=!/home/$USER/.lks-setup-complete

[Service]
Type=oneshot
RemainAfterExit=yes
User=$USER
WorkingDirectory=/home/$USER
ExecStart=/home/$USER/setup_raspberry_pi.sh complete-setup
ExecStop=/usr/bin/docker compose -f /home/$USER/Autonomous_Mobile_Manipulator/docker-compose.prod.yml down
TimeoutStartSec=900
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    # Create setup completion marker
    sudo tee /etc/systemd/system/lks-setup-marker.service > /dev/null <<EOF
[Unit]
Description=Mark LKS Setup Complete
After=lks-robot.service
Requires=lks-robot.service

[Service]
Type=oneshot
User=$USER
ExecStart=/bin/touch /home/$USER/.lks-setup-complete
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable lks-robot lks-setup-marker

    log_success "Services created"
}

# Create monitoring script
create_monitoring() {
    log_info "Creating monitoring scripts..."

    # Create ROS2 logs directory
    mkdir -p ~/ros2_logs

    # Copy log maintenance script
    cp ros2_log_maintenance.sh ~/ros2_log_maintenance.sh 2>/dev/null || log_warning "ros2_log_maintenance.sh not found in current directory"
    chmod +x ~/ros2_log_maintenance.sh 2>/dev/null || true

    # Create system monitoring script
    tee ~/system_monitor.sh > /dev/null <<EOF
#!/bin/bash
# System monitoring script with ROS2 focus

echo "=== System Status ==="
echo "Uptime: \$(uptime -p)"
echo "Load: \$(uptime | awk -F'load average:' '{print \$2}')"
echo "Memory: \$(free -h | awk 'NR==2{printf "%.1f%% used", \$3*100/\$2}')"
echo "Disk: \$(df -h / | awk 'NR==2{print \$5}') used"

echo ""
echo "=== Docker Status ==="
docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.Status}}" 2>/dev/null || echo "No containers running"

echo ""
echo "=== ROS2 Container Health ==="
if docker ps -q -f name=ros2_sim_prod_container | grep -q .; then
    ROS2_HEALTH=\$(docker inspect --format='{{.State.Health.Status}}' ros2_sim_prod_container 2>/dev/null || echo "unknown")
    echo "ROS2 Container Health: \$ROS2_HEALTH"

    # Check ROS2 nodes if container is healthy
    if [ "\$ROS2_HEALTH" = "healthy" ]; then
        echo "Active ROS2 Nodes:"
        docker exec ros2_sim_prod_container ros2 node list 2>/dev/null | head -10 || echo "Unable to query nodes"
    fi
else
    echo "ROS2 Container: NOT RUNNING"
fi

echo ""
echo "=== ROS2 Logs Status ==="
if [ -d ~/ros2_logs ]; then
    LOG_COUNT=\$(find ~/ros2_logs -name "*.log" 2>/dev/null | wc -l)
    LOG_SIZE=\$(du -sh ~/ros2_logs 2>/dev/null | cut -f1)
    echo "ROS2 Logs: \$LOG_COUNT files, \$LOG_SIZE total"

    # Show recent watchdog activity
    if [ -f ~/ros2_logs/watchdog.log ]; then
        echo "Recent Watchdog Activity:"
        tail -5 ~/ros2_logs/watchdog.log 2>/dev/null || echo "No recent activity"
    fi
else
    echo "ROS2 Logs: Directory not found"
fi

echo ""
echo "=== Temperature ==="
echo "CPU: \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
EOF

    chmod +x ~/system_monitor.sh

    # Create health check script
    tee ~/health_check.sh > /dev/null <<EOF
#!/bin/bash
# Health check script for LKS Robot with ROS2 focus

echo "Performing comprehensive health checks..."

ERRORS_FOUND=0
WARNINGS_FOUND=0

# Function to report errors and warnings
report_error() {
    echo "ERROR: \$1"
    ((ERRORS_FOUND++))
}

report_warning() {
    echo "WARNING: \$1"
    ((WARNINGS_FOUND++))
}

# Check Docker containers
echo "Checking Docker containers..."
if ! docker ps | grep -q ros2_sim_prod_container; then
    report_error "ROS2 container not running"
else
    echo "✓ ROS2 container is running"

    # Check ROS2 container health
    ROS2_HEALTH=\$(docker inspect --format='{{.State.Health.Status}}' ros2_sim_prod_container 2>/dev/null || echo "unknown")
    if [ "\$ROS2_HEALTH" != "healthy" ]; then
        report_warning "ROS2 container health: \$ROS2_HEALTH"
    else
        echo "✓ ROS2 container is healthy"
    fi
fi

if ! docker ps | grep -q n8n_prod_container; then
    report_error "n8n container not running"
else
    echo "✓ n8n container is running"
fi

# Check ROS2-specific health
echo "Checking ROS2 services..."
if docker ps | grep -q ros2_sim_prod_container; then
    # Check critical ROS2 topics
    if ! docker exec ros2_sim_prod_container timeout 5 ros2 topic list 2>/dev/null | grep -q robot_status; then
        report_error "ROS2 robot_status topic not available"
    else
        echo "✓ ROS2 robot_status topic active"
    fi

    # Check critical ROS2 nodes
    CRITICAL_NODES=("robot_automation_server" "rest_api_server" "websocket_server")
    for node in "\${CRITICAL_NODES[@]}"; do
        if ! docker exec ros2_sim_prod_container ros2 node list 2>/dev/null | grep -q "\$node"; then
            report_warning "ROS2 node not found: \$node"
        else
            echo "✓ ROS2 node active: \$node"
        fi
    done

    # Check watchdog process
    if ! docker exec ros2_sim_prod_container pgrep -f "ros2_watchdog.sh" >/dev/null 2>&1; then
        report_warning "ROS2 watchdog process not running"
    else
        echo "✓ ROS2 watchdog is active"
    fi
else
    report_error "Cannot check ROS2 services - container not running"
fi

# Check API endpoints
echo "Checking API endpoints..."
if ! curl -s --max-time 5 http://localhost:5000/health > /dev/null; then
    report_error "Robot API not responding"
else
    echo "✓ Robot API is responding"

    # Check API health details
    API_HEALTH=\$(curl -s --max-time 5 http://localhost:5000/health || echo "{}")
    if echo "\$API_HEALTH" | grep -q '"status":"healthy"'; then
        echo "✓ Robot API reports healthy status"
    else
        report_warning "Robot API health check failed"
    fi
fi

if ! curl -s --max-time 5 http://localhost:5678 > /dev/null; then
    report_error "n8n interface not responding"
else
    echo "✓ n8n interface is responding"
fi

# Check WebSocket endpoint
echo "Checking WebSocket endpoint..."
if ! timeout 5 bash -c 'echo > /dev/tcp/localhost/8765' 2>/dev/null; then
    report_warning "WebSocket port not accessible"
else
    echo "✓ WebSocket port is accessible"
fi

# Check system resources
echo "Checking system resources..."
MEMORY_USAGE=\$(free | awk 'NR==2{printf "%.0f", \$3*100/\$2}')
if [ "\$MEMORY_USAGE" -gt 90 ]; then
    report_error "High memory usage: \${MEMORY_USAGE}%"
elif [ "\$MEMORY_USAGE" -gt 80 ]; then
    report_warning "High memory usage: \${MEMORY_USAGE}%"
else
    echo "✓ Memory usage normal: \${MEMORY_USAGE}%"
fi

# Check disk space
DISK_USAGE=\$(df / | awk 'NR==2{print \$5}' | sed 's/%//')
if [ "\$DISK_USAGE" -gt 95 ]; then
    report_error "Critical disk usage: \${DISK_USAGE}%"
elif [ "\$DISK_USAGE" -gt 85 ]; then
    report_warning "High disk usage: \${DISK_USAGE}%"
else
    echo "✓ Disk usage normal: \${DISK_USAGE}%"
fi

# Check ROS2 logs for errors
echo "Checking ROS2 logs..."
if [ -d ~/ros2_logs ]; then
    RECENT_ERRORS=\$(find ~/ros2_logs -name "*.log" -mmin -10 -exec grep -l "ERROR\|CRITICAL" {} \; 2>/dev/null | wc -l)
    if [ "\$RECENT_ERRORS" -gt 0 ]; then
        report_warning "\$RECENT_ERRORS log files contain recent errors"
    else
        echo "✓ No recent errors in ROS2 logs"
    fi
fi

# Summary
echo ""
echo "=== Health Check Summary ==="
if [ "\$ERRORS_FOUND" -gt 0 ]; then
    echo "❌ FAILED: \$ERRORS_FOUND error(s) found"
    exit 1
elif [ "\$WARNINGS_FOUND" -gt 0 ]; then
    echo "⚠️  WARNING: \$WARNINGS_FOUND warning(s) found, but system operational"
    exit 0
else
    echo "✅ SUCCESS: All systems healthy"
    exit 0
fi
EOF

    chmod +x ~/health_check.sh

    # Add to cron for regular health checks
    (crontab -l 2>/dev/null; echo "*/5 * * * * /home/$USER/health_check.sh") | crontab -

    log_success "Monitoring configured"
}

# Main setup function
main() {
    local command="$1"

    case "$command" in
        "complete-setup")
            log_info "Starting LKS Robot Post-Reboot Setup"
            log_info "===================================="
            complete_setup
            ;;
        *)
            # Initial setup
            log_info "Starting LKS Robot Raspberry Pi Setup"
            log_info "======================================"

            # Run setup steps
            check_raspberry_pi
            update_system
            configure_system
            install_dependencies
            setup_docker
            configure_hardware
            configure_network
            optimize_performance

            if test_hardware; then
                create_service
                create_monitoring

                log_success "Initial setup completed successfully!"
                log_info ""
                log_info "Next steps:"
                log_info "1. Reboot: sudo reboot"
                log_info "2. After reboot, the setup will continue automatically..."
                log_info ""
                log_info "The setup will then:"
                log_info "   - Clone the Autonomous Mobile Manipulator project"
                log_info "   - Start all services (production mode)"
                log_info "   - Import and configure N8N workflows"
                log_info "   - Set up monitoring and health checks"
                log_info ""
                log_info "After complete setup, access interfaces:"
                log_info "   - n8n: http://localhost:5678"
                log_info "   - Robot API: http://localhost:5000"
                log_info ""
                log_info "Run './system_monitor.sh' to check system status"
                log_info "Run './health_check.sh' to verify all services are running"
            else
                log_error "Setup failed during hardware testing"
                exit 1
            fi
            ;;
    esac
}

# Run main function
main "$@"
