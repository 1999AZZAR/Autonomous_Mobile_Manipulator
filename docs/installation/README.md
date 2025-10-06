# Installation Guide

This guide provides complete instructions for installing and setting up the Autonomous Mobile Manipulator software stack.

## Prerequisites

### System Requirements

#### Development Machine (Host PC)
- **Operating System**: Debian 11/12 or Ubuntu 20.04/22.04 (64-bit)
- **Memory**: 16GB RAM minimum, 32GB recommended
- **Processor**: Multi-core processor (Intel i5/i7 or equivalent)
- **Storage**: 50GB free space for Docker images and containers
- **Graphics**: For GUI applications (RViz, Gazebo)
- **Network**: Internet connection for downloading dependencies

#### Target Platform (Raspberry Pi 5)
- **Model**: Raspberry Pi 5 (8GB RAM recommended)
- **Operating System**: Ubuntu Server 22.04 (64-bit)
- **Storage**: 32GB microSD card minimum
- **Power Supply**: 5V/3A USB-C power supply
- **Network**: Ethernet or WiFi connectivity

### Required Software

#### Core Dependencies
- **Git**: Version control system
- **Docker Engine**: Containerization platform
- **Docker Compose**: Multi-container orchestration

#### Optional but Recommended
- **Visual Studio Code**: Development environment
  - Extensions: ROS, Docker, Python

## Installation Steps

### Step 1: Install Git

```bash
sudo apt update
sudo apt install git -y
```

Verify installation:
```bash
git --version
# Should output: git version 2.x.x
```

### Step 2: Install Docker Engine

#### Ubuntu/Debian (APT)
```bash
# Update package index
sudo apt update

# Install packages for APT over HTTPS
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release -y

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up stable repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io -y

# Start and enable Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group (logout/login required after this)
sudo usermod -aG docker $USER
```

#### Alternative: Docker Desktop
For graphical interface, install Docker Desktop from [docker.com](https://www.docker.com/products/docker-desktop/).

### Step 3: Verify Docker Installation

```bash
# Check Docker version
docker --version

# Check Docker Compose version
docker compose version

# Test Docker installation
docker run hello-world
```

Expected output:
```
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

### Step 4: Clone the Repository

```bash
# Navigate to desired directory
cd ~

# Clone the repository
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git

# Enter the project directory
cd Autonomous_Mobile_Manipulator

# Verify project structure
ls -la
```

### Step 5: Configure X11 (for GUI Applications)

If you plan to run GUI applications (RViz, Gazebo):

```bash
# Allow Docker to access display
xhost +local:docker

# Verify DISPLAY variable
echo $DISPLAY
# Should show something like :0
```

### Step 6: Build and Launch

```bash
# Build Docker images and start services
docker compose up --build -d

# Check running containers
docker compose ps
```

Expected output:
```
NAME                 IMAGE                        STATUS
n8n_container        docker.io/n8nio/n8n:latest   Up
ros2_sim_container   autonomous_mobile_manipulator-ros2-sim   Up
```

### Step 7: Verify Installation

#### Access n8n Web Interface
```bash
# Open web browser and navigate to:
http://localhost:5678
```

#### Access ROS 2 Environment
```bash
# Enter ROS 2 container
docker exec -it ros2_sim_container bash

# Source ROS 2 environment
source /opt/ros/iron/setup.bash

# Verify ROS 2 installation
ros2 --help
```

## Post-Installation Configuration

### Environment Variables

The system uses the following key environment variables:

```yaml
# docker-compose.yml
environment:
  - DISPLAY=${DISPLAY}           # X11 display for GUI apps
  - QT_X11_NO_MITSHM=1          # Disable MIT-SHM for containerized X11
  - GENERIC_TIMEZONE=Asia/Jakarta # Set timezone for n8n
```

### Network Configuration

The system uses host networking mode for:
- Simplified GUI forwarding
- Direct hardware access (serial ports, GPIO)
- Network service accessibility

## Troubleshooting Installation Issues

### Docker Permission Issues

```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Restart session or run:
newgrp docker

# Test Docker access
docker run hello-world
```

### X11 Display Issues

```bash
# Check DISPLAY variable
echo $DISPLAY

# Allow Docker X11 access
xhost +local:docker

# Test X11 forwarding
xeyes  # Should show animated eyes
```

### Container Startup Issues

```bash
# Check container logs
docker compose logs ros2_sim_container
docker compose logs n8n_container

# Restart specific service
docker compose restart ros2_sim_container

# Rebuild container
docker compose build --no-cache ros2_sim_container
```

## Next Steps

1. **Hardware Setup**: Proceed to [Hardware Setup Guide](../hardware/)
2. **Software Configuration**: Continue with [Software Configuration Guide](../software/)
3. **LabVIEW Integration**: See [LabVIEW Integration Guide](../labview-integration/)
4. **Testing**: Run the provided example workflows and launch files

## Support

For installation issues:
- Check the [Troubleshooting Guide](../troubleshooting/)
- Review container logs: `docker compose logs`
- Open an issue on [GitHub](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues)

---

*This installation guide ensures a smooth setup process for the Autonomous Mobile Manipulator project. Follow each step carefully and verify functionality before proceeding to advanced configuration.*
