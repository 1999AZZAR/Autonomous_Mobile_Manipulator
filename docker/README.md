# Docker Configuration Guide

This project supports two different Docker configurations for development and production environments:

## Development Environment (x86 PC/Laptop)

**Use for:** Local development, testing, and simulation on x86 machines

### Files:
- `docker-compose.dev.yml` - Development docker-compose configuration
- `ros2_ws/Dockerfile.dev` - Development Dockerfile with ROS2 Iron

### Features:
- ROS2 Iron with desktop packages (Gazebo, RViz support)
- GUI forwarding for visualization tools
- Full development environment
- Optimized for x86 architecture

### Usage:
```bash
# Start development environment
docker compose -f docker-compose.dev.yml up -d

# Stop development environment
docker compose -f docker-compose.dev.yml down
```

## Production Environment (ARM64 Raspberry Pi)

**Use for:** Deployment on Raspberry Pi 5 with Ubuntu 24.04

### Files:
- `docker-compose.prod.yml` - Production docker-compose configuration
- `ros2_ws/Dockerfile.prod` - Production Dockerfile with ROS2 Jazzy

### Features:
- ROS2 Jazzy (ARM64 compatible)
- Hardware device access (GPIO, I2C, SPI, UART)
- Production-optimized container names
- Host networking for better performance
- Privileged mode for hardware access

### Usage:
```bash
# Start production environment
docker compose -f docker-compose.prod.yml up -d

# Stop production environment
docker compose -f docker-compose.prod.yml down
```

## Key Differences

| Feature | Development (x86) | Production (ARM64) |
|---------|-------------------|-------------------|
| ROS Version | Iron | Jazzy |
| Architecture | x86_64 | ARM64 |
| GUI Support | Full (Gazebo, RViz) | Limited (no Gazebo desktop) |
| Hardware Access | None | GPIO, I2C, SPI, UART |
| Networking | Host mode | Host mode |
| Container Names | `*_dev_container` | `*_prod_container` |

## Environment Variables

Both configurations support the same environment variables:

- `DISPLAY` - X11 display forwarding
- `WAYLAND_DISPLAY` - Wayland display forwarding
- `XDG_RUNTIME_DIR` - XDG runtime directory
- `QT_QPA_PLATFORM` - Qt platform plugin
- `PYTHONPATH` - Python path for custom modules

## Ports

Both configurations expose the same ports:

- `5678` - n8n workflow interface
- `5000` - Robot REST API
- `8000` - Professional web interface (dev only)
- `8765` - WebSocket server

## Hardware Access (Production Only)

The production configuration includes device mappings for Raspberry Pi hardware:

- `/dev/gpiomem` - GPIO memory access
- `/dev/i2c-1` - I2C bus 1
- `/dev/spidev0.0` - SPI device
- `/dev/ttyS0`, `/dev/ttyAMA0` - Serial ports

## Building Images

### Development:
```bash
docker compose -f docker-compose.dev.yml build
```

### Production:
```bash
docker compose -f docker-compose.prod.yml build
```

## Troubleshooting

### Common Issues:

1. **Architecture Mismatch**: Ensure you're using the correct compose file for your platform
2. **Hardware Access**: Production containers need privileged mode and device mappings
3. **GUI Issues**: Development environment requires X11 forwarding setup
4. **Port Conflicts**: Stop other services using ports 5678, 5000, etc.

### Logs:

```bash
# Check ROS2 container logs
docker logs ros2_sim_dev_container    # Development
docker logs ros2_sim_prod_container   # Production

# Check n8n container logs
docker logs n8n_dev_container         # Development
docker logs n8n_prod_container        # Production
```
