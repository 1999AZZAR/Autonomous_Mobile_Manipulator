#!/bin/bash
# LKS Robot Project - Raspberry Pi Setup Script
# This script automates the Raspberry Pi configuration for the LKS Robot Project

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

    # Enable required interfaces
    sudo raspi-config nonint do_i2c 0
    sudo raspi-config nonint do_spi 0
    sudo raspi-config nonint do_serial_hw 0
    sudo raspi-config nonint do_serial_cons 1
    sudo raspi-config nonint do_camera 0

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
        pigpio \
        python3-pigpio \
        python3-rpi.gpio \
        python3-lgpio \
        python3-gpiozero \
        python3-picamera2 \
        ca-certificates \
        gnupg \
        lsb-release \
        uidmap \
        dbus-user-session \
        python3-numpy \
        python3-smbus \
        python3-rtimulib

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

# Create systemd service
create_service() {
    log_info "Creating systemd service..."

    # Create systemd service for robot deployment
    sudo tee /etc/systemd/system/lks-robot.service > /dev/null <<EOF
[Unit]
Description=LKS Robot Project
After=docker.service network.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=$USER
WorkingDirectory=/home/$USER/LKS_Robot_Project
ExecStart=/usr/bin/docker compose -f docker-compose.prod.yml up -d
ExecStop=/usr/bin/docker compose -f docker-compose.prod.yml down
TimeoutStartSec=300
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable lks-robot

    log_success "Service created"
}

# Create monitoring script
create_monitoring() {
    log_info "Creating monitoring scripts..."

    # Create system monitoring script
    tee ~/system_monitor.sh > /dev/null <<EOF
#!/bin/bash
# System monitoring script

echo "=== System Status ==="
echo "Uptime: \$(uptime -p)"
echo "Load: \$(uptime | awk -F'load average:' '{print \$2}')"
echo "Memory: \$(free -h | awk 'NR==2{printf "%.1f%% used", \$3*100/\$2}')"
echo "Disk: \$(df -h / | awk 'NR==2{print \$5}') used"

echo ""
echo "=== Docker Status ==="
docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}" 2>/dev/null || echo "No containers running"

echo ""
echo "=== Temperature ==="
echo "CPU: \$(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
EOF

    chmod +x ~/system_monitor.sh

    # Create health check script
    tee ~/health_check.sh > /dev/null <<EOF
#!/bin/bash
# Health check script for LKS Robot

echo "Performing health checks..."

# Check Docker containers
if ! docker ps | grep -q lks_robot; then
    echo "ERROR: Robot container not running"
    exit 1
fi

if ! docker ps | grep -q lks_n8n; then
    echo "ERROR: n8n container not running"
    exit 1
fi

# Check API endpoints
if ! curl -s --max-time 5 http://localhost:5000/health > /dev/null; then
    echo "ERROR: Robot API not responding"
    exit 1
fi

if ! curl -s --max-time 5 http://localhost:5678 > /dev/null; then
    echo "ERROR: n8n interface not responding"
    exit 1
fi

echo "All services healthy"
EOF

    chmod +x ~/health_check.sh

    # Add to cron for regular health checks
    (crontab -l 2>/dev/null; echo "*/5 * * * * /home/$USER/health_check.sh") | crontab -

    log_success "Monitoring configured"
}

# Main setup function
main() {
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

        log_success "Setup completed successfully!"
        log_info ""
        log_info "Next steps:"
        log_info "1. Reboot: sudo reboot"
        log_info "2. After reboot, clone the project:"
        log_info "   git clone https://github.com/1999AZZAR/LKS_Robot_Project.git"
        log_info "3. Start the robot: cd LKS_Robot_Project && docker compose -f docker-compose.prod.yml up -d"
        log_info "4. Access interfaces:"
        log_info "   - n8n: http://localhost:5678"
        log_info "   - Robot API: http://localhost:5000"
        log_info ""
        log_info "Run './system_monitor.sh' to check system status"
        log_info "Run './health_check.sh' to verify all services are running"
    else
        log_error "Setup failed during hardware testing"
        exit 1
    fi
}

# Run main function
main "$@"
