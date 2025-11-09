# ROS2 Reliability and High Availability Setup

This document describes the comprehensive reliability features implemented for the ROS2-based Autonomous Mobile Manipulator to ensure maximum uptime and minimum downtime.

## 🛡️ Reliability Features Overview

### 1. **Automatic Watchdog System**
- **Location**: `ros2_ws/ros2_watchdog.sh`
- **Function**: Monitors critical ROS2 nodes and services every 30 seconds
- **Recovery**: Automatically restarts failed processes with cooldown periods
- **Limits**: Maximum 3 restart attempts per hour per service to prevent thrashing

**Monitored Components:**
- `robot_automation_server` - Core automation logic
- `rest_api_server` - REST API endpoint
- `websocket_server` - WebSocket communication
- `n8n_ros2_bridge` - Workflow integration
- `robot_status_server` - Status monitoring

### 2. **Health Checks and Monitoring**
- **Docker Health Checks**: Container-level health monitoring with 30s intervals
- **ROS2 Topic Monitoring**: Verifies critical topics are active
- **Resource Monitoring**: CPU, memory, and disk usage tracking
- **Comprehensive Health Script**: `health_check.sh` with detailed diagnostics

### 3. **Resource Management**
- **CPU Limits**: 1.5 cores maximum, 0.5 cores reserved
- **Memory Limits**: 1.5GB maximum, 512MB reserved
- **Shared Memory**: 256MB for ROS2 DDS communication
- **File Descriptors**: 1024 soft, 2048 hard limit
- **Process Limits**: 512 soft, 1024 hard limit

### 4. **ROS2-Specific Optimizations**
- **Domain ID**: Isolated network domain (ID: 42)
- **DDS Implementation**: CycloneDDS for reliable communication
- **QoS Settings**: Optimized for real-time performance
- **Network Isolation**: Prevents interference with other ROS2 instances

### 5. **Logging and Debugging**
- **Structured Logging**: Timestamped, categorized log entries
- **Log Rotation**: Automatic compression and archiving
- **Core Dumps**: Enabled for crash analysis
- **Centralized Logs**: All logs in `/root/ros2_ws/logs/`

### 6. **Container Security and Isolation**
- **No New Privileges**: Security hardening
- **Read-Only Filesystem**: Except necessary directories
- **Resource Limits**: Prevents resource exhaustion
- **Device Access**: Controlled hardware access

### 7. **Automatic Recovery Mechanisms**
- **Graceful Shutdown**: Proper cleanup on container stop
- **Dependency Management**: Services start in correct order
- **Restart Policies**: `unless-stopped` for persistent operation
- **Cooldown Periods**: Prevents restart loops

## 🔧 Configuration Files

### Docker Compose (`docker-compose.prod.yml`)
```yaml
# Resource limits and health checks
deploy:
  resources:
    limits:
      cpus: '1.5'
      memory: 1.5G
    reservations:
      cpus: '0.5'
      memory: 512M

healthcheck:
  test: ["CMD-SHELL", "timeout 10 ros2 topic list | grep -q robot_status"]
  interval: 30s
  timeout: 10s
  retries: 3
```

### CycloneDDS Configuration (`ros2_ws/cyclonedds_config.xml`)
```xml
<Domain id="42">
  <Discovery>
    <LeaseDuration>10s</LeaseDuration>
  </Discovery>
  <Internal>
    <HeartbeatInterval>100ms</HeartbeatInterval>
  </Internal>
</Domain>
```

### Watchdog Script (`ros2_ws/ros2_watchdog.sh`)
- Monitors critical processes
- Implements restart logic with limits
- Logs all activities
- Memory and CPU monitoring

## 📊 Monitoring and Maintenance

### Health Check Script
```bash
./health_check.sh
```
- Comprehensive system health assessment
- ROS2-specific checks
- Resource usage monitoring
- Error and warning reporting

### System Monitoring
```bash
./system_monitor.sh
```
- Real-time system status
- Docker container health
- ROS2 node status
- Log activity summary

### Log Maintenance
```bash
./ros2_log_maintenance.sh
```
- Automatic log compression
- Archive old logs
- Emergency cleanup
- Size management

## 🚨 Alerting and Notifications

The system includes comprehensive error detection:
- **Critical Errors**: Container failures, service unavailability
- **Warnings**: High resource usage, missing nodes
- **Monitoring**: 5-minute health check intervals via cron

## 🔄 Recovery Procedures

### Automatic Recovery
1. **Process Failure**: Watchdog detects and restarts (up to 3 attempts)
2. **Container Failure**: Docker restart policy handles automatic restart
3. **Resource Exhaustion**: Automatic cleanup and resource management

### Manual Recovery
```bash
# Restart ROS2 services
docker compose -f docker-compose.prod.yml restart ros2-sim

# Check logs
docker logs ros2_sim_prod_container

# Manual health check
./health_check.sh

# Clean logs if needed
./ros2_log_maintenance.sh
```

## 📈 Performance Optimizations

### DDS Optimizations
- CycloneDDS with optimized network settings
- Domain isolation prevents conflicts
- Reduced discovery overhead

### Memory Management
- Shared memory allocation for DDS
- Memory monitoring and limits
- Automatic cleanup of old logs

### CPU Management
- CPU affinity and limits
- Process priority management
- Efficient monitoring intervals

## 🔒 Security Considerations

- Container privilege minimization
- Network isolation with domain IDs
- Resource limits prevent DoS
- Secure logging practices

## 📝 Maintenance Schedule

- **Daily**: Log compression and cleanup
- **Weekly**: Archive rotation
- **Monthly**: Full log maintenance review
- **Continuous**: Health monitoring every 5 minutes

## 🐛 Troubleshooting

### Common Issues and Solutions

1. **ROS2 Node Not Starting**
   - Check watchdog logs: `tail -f ros2_logs/watchdog.log`
   - Verify dependencies: `docker exec ros2_sim_prod_container ros2 doctor`

2. **High Resource Usage**
   - Check monitoring: `./system_monitor.sh`
   - Review logs for patterns
   - Consider resource limit adjustments

3. **Communication Failures**
   - Verify DDS configuration
   - Check network isolation
   - Review domain ID conflicts

4. **Log Disk Full**
   - Run maintenance: `./ros2_log_maintenance.sh`
   - Check archive settings
   - Adjust retention policies

This comprehensive reliability setup ensures the ROS2-based robot maintains maximum uptime while providing detailed monitoring and automatic recovery capabilities.
