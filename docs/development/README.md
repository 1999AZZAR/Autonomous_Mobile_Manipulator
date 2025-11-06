# Development Workflow Guide

This guide provides comprehensive instructions for developing and extending the Autonomous Mobile Manipulator robot software stack.

## Table of Contents

- [Development Environment Setup](#development-environment-setup)
  - [Required Tools](#required-tools)
  - [IDE Configuration](#ide-configuration)
- [Code Development Workflow](#code-development-workflow)
  - [Development Environment Considerations](#development-environment-considerations)
  - [1. Feature Branch Development](#1-feature-branch-development)
  - [2. Code Review Process](#2-code-review-process)
  - [3. Continuous Integration](#3-continuous-integration)
- [ROS 2 Package Development](#ros-2-package-development)
  - [Creating New ROS 2 Packages](#creating-new-ros-2-packages)
  - [Package Structure Guidelines](#package-structure-guidelines)
  - [Code Style Guidelines](#code-style-guidelines)
- [Testing and Validation](#testing-and-validation)
  - [Unit Testing](#unit-testing)
  - [Integration Testing](#integration-testing)
- [Debugging and Profiling](#debugging-and-profiling)
  - [ROS 2 Debugging Tools](#ros-2-debugging-tools)
  - [Performance Profiling](#performance-profiling)
- [Documentation Development](#documentation-development)
  - [Documentation Standards](#documentation-standards)
  - [Documentation Build Process](#documentation-build-process)
- [Version Control Best Practices](#version-control-best-practices)
  - [Commit Message Guidelines](#commit-message-guidelines)
  - [Branch Management](#branch-management)
- [Quality Assurance](#quality-assurance)
  - [Code Review Checklist](#code-review-checklist)
  - [Testing Requirements](#testing-requirements)
- [Deployment and Release](#deployment-and-release)
  - [Release Process](#release-process)
- [Support and Maintenance](#support-and-maintenance)
  - [Issue Tracking](#issue-tracking)
  - [Maintenance Schedule](#maintenance-schedule)
- [Advanced Development Topics](#advanced-development-topics)
  - [Custom Message Types](#custom-message-types)
  - [Plugin Development](#plugin-development)
  - [Real-time Systems Development](#real-time-systems-development)
- [Community Contribution](#community-contribution)
  - [Contributing Guidelines](#contributing-guidelines)
  - [Community Standards](#community-standards)
- [Resources and References](#resources-and-references)
  - [Essential Documentation](#essential-documentation)
  - [Development Tools](#development-tools)
  - [Community Resources](#community-resources)

## Development Environment Setup

### Required Tools

#### Core Development Tools
- **Git**: Version control system
- **Docker & Docker Compose**: Containerized development environment
- **Visual Studio Code**: Primary development IDE
- **ROS 2 Iron**: Robotics framework

#### Optional but Recommended
- **GitHub Desktop**: Graphical Git interface
- **Postman**: API testing and documentation
- **Wireshark**: Network protocol analysis
- **htop**: System resource monitoring

### IDE Configuration

#### Visual Studio Code Setup

```bash
# Install VS Code extensions
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension redhat.vscode-yaml
code --install-extension ms-vscode.powershell
code --install-extension eamodio.gitlens
```

#### VS Code Settings

```json
// .vscode/settings.json
{
  "python.defaultInterpreterPath": "/usr/bin/python3",
  "ros.distro": "iron",
  "files.associations": {
    "*.launch.py": "python",
    "*.xacro": "xml",
    "*.urdf": "xml",
    "*.sdf": "xml"
  },
  "editor.formatOnSave": true,
  "editor.rulers": [80, 100, 120],
  "python.linting.enabled": true,
  "python.linting.pylintEnabled": true,
  "python.formatting.provider": "black"
}
```

## Code Development Workflow

### Development Environment Considerations

#### Host vs Container Development
This project uses Docker containers for ROS2 development, but you edit code on your host system. The launch files include conditional imports to handle this workflow:

- **Host System (Code Editing)**: ROS2 packages are not available, but launch files include error handling that provides clear messages directing you to use the Docker environment
- **Docker Container (Execution)**: All ROS2 packages are available, and your code changes are automatically available via volume mounts
- **Best Practice**: Edit code on host, execute and test in containers

#### Conditional Import Handling
Launch files use conditional imports to gracefully handle missing ROS2 packages:

```python
# Conditional import for ROS2 packages - allows development on systems without ROS2 installed
try:
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    # ... other ROS2 imports
except ImportError as e:
    print(f"Warning: ROS2 packages not available. This launch file requires ROS2 to be installed.")
    print(f"Install ROS2 or run this in the Docker container environment.")
    print(f"Error: {e}")
    raise
```

This allows:
- Syntax checking on host systems
- Clear error messages when ROS2 is not available
- Proper execution within Docker containers

### 1. Feature Branch Development

```bash
# Create feature branch
git checkout -b feature/new-navigation-algorithm

# Make changes to ROS 2 packages
# Edit launch files, configurations, or add new nodes

# Test changes in simulation
ros2 launch my_robot_bringup gazebo_world.launch.py

# Test changes on hardware (if available)
ros2 launch my_robot_bringup robot.launch.py

# Commit changes
git add .
git commit -m "feat: Add new navigation algorithm

- Implement A* path planning
- Add obstacle avoidance behavior
- Update navigation parameters
- Add unit tests"

# Push feature branch
git push origin feature/new-navigation-algorithm
```

### 2. Code Review Process

#### Pull Request Guidelines

**PR Template**:
```markdown
## Description
Brief description of changes and their purpose.

## Changes Made
- List of specific changes
- Files modified
- New features added

## Testing Performed
- Simulation testing results
- Hardware testing results
- Unit test coverage

## Related Issues
- Links to related GitHub issues

## Screenshots/Demos
- Visual evidence of functionality
```

**Review Checklist**:
- [ ] Code follows project style guidelines
- [ ] New features have tests
- [ ] Documentation updated
- [ ] No breaking changes
- [ ] Performance impact assessed

### 3. Continuous Integration

#### Pre-commit Hooks

```bash
# Install pre-commit hooks
pip install pre-commit
pre-commit install

# Run all checks manually
pre-commit run --all-files
```

#### GitHub Actions Workflow

```yaml
# .github/workflows/ci.yml
name: CI/CD Pipeline
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    - name: Build Docker image
      run: docker compose build
    - name: Run tests
      run: docker compose exec ros2_sim_container colcon test
    - name: Check code style
      run: docker compose exec ros2_sim_container colcon test-result --verbose
```

## ROS 2 Package Development

### Creating New ROS 2 Packages

#### 1. Python Package

```bash
# In ROS 2 container
docker exec -it ros2_sim_container bash

# Navigate to source directory
cd /root/ros2_ws/src

# Create new Python package
ros2 pkg create my_new_package --build-type ament_python --dependencies rclpy std_msgs geometry_msgs

# Edit package.xml
vim my_new_package/package.xml

# Create node file
vim my_new_package/my_new_package/my_node.py

# Build package
colcon build --packages-select my_new_package

# Source and test
source install/setup.bash
ros2 run my_new_package my_node
```

#### 2. C++ Package

```bash
# In ROS 2 container
docker exec -it ros2_sim_container bash

# Navigate to source directory
cd /root/ros2_ws/src

# Create new C++ package
ros2 pkg create my_cpp_package --build-type ament_cmake --dependencies rclcpp std_msgs

# Edit CMakeLists.txt
vim my_cpp_package/CMakeLists.txt

# Create C++ node
vim my_cpp_package/src/my_cpp_node.cpp

# Build package
colcon build --packages-select my_cpp_package

# Source and test
source install/setup.bash
ros2 run my_cpp_package my_cpp_node
```

### Package Structure Guidelines

#### Standard ROS 2 Package Layout

```
my_robot_package/
├── package.xml                 # Package manifest and dependencies
├── CMakeLists.txt             # CMake build configuration (C++)
├── setup.py                   # Python package setup (Python)
├── setup.cfg                  # Python package configuration (Python)
├── resource/                  # Resource files
│   └── package_name           # Package name marker
├── include/                   # C++ header files
│   └── package_name/          # Header file directory
├── src/                       # Source code
│   ├── package_name/          # Python module directory
│   └── CMakeLists.txt         # C++ source CMakeLists
├── launch/                    # Launch files
├── config/                    # Configuration files
├── worlds/                    # Gazebo world files
├── rviz/                      # RViz configuration files
├── urdf/                      # Robot description files
├── meshes/                    # 3D mesh files
├── test/                      # Unit tests
└── README.md                  # Package documentation
```

### Code Style Guidelines

#### Python Code Style

```python
#!/usr/bin/env python3
"""Module docstring describing the package purpose."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import Optional


class MovementController(Node):
    """Class docstring describing the node functionality."""

    def __init__(self) -> None:
        """Initialize the movement controller node."""
        super().__init__('movement_controller')

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialize velocity command
        self.twist = Twist()

        self.get_logger().info('Movement controller initialized')

    def control_loop(self) -> None:
        """Main control loop for robot movement."""
        # Set forward velocity
        self.twist.linear.x = 0.5
        self.twist.angular.z = 0.0

        # Publish velocity command
        self.velocity_publisher.publish(self.twist)


def main(args: Optional[list] = None) -> None:
    """Main function to run the movement controller."""
    rclpy.init(args=args)

    controller = MovementController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### C++ Code Style

```cpp
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MovementController : public rclcpp::Node
{
public:
    MovementController() : Node("movement_controller")
    {
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Timer for control loop
        timer_ = this->create_timer(
            std::chrono::milliseconds(100),
            std::bind(&MovementController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Movement controller initialized");
    }

private:
    void control_loop()
    {
        // Create velocity command message
        auto twist_msg = geometry_msgs::msg::Twist();

        // Set forward velocity
        twist_msg.linear.x = 0.5;
        twist_msg.angular.z = 0.0;

        // Publish velocity command
        velocity_publisher_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MovementController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

## Testing and Validation

### Unit Testing

#### Python Unit Tests

```python
#!/usr/bin/env python3
"""Unit tests for movement controller."""

import unittest
from unittest.mock import Mock, patch
import rclpy
from geometry_msgs.msg import Twist

from my_robot_package.my_robot_package.movement_controller import MovementController


class TestMovementController(unittest.TestCase):
    """Test cases for MovementController class."""

    def setUp(self):
        """Set up test fixtures."""
        self.node = MovementController()

    def tearDown(self):
        """Clean up after tests."""
        self.node.destroy_node()

    def test_velocity_command_creation(self):
        """Test that velocity commands are created correctly."""
        # Test forward movement
        expected_twist = Twist()
        expected_twist.linear.x = 0.5
        expected_twist.angular.z = 0.0

        # Verify command structure
        self.assertEqual(self.node.twist.linear.x, expected_twist.linear.x)
        self.assertEqual(self.node.twist.angular.z, expected_twist.angular.z)

    def test_publisher_creation(self):
        """Test that publisher is created with correct topic."""
        self.assertIsNotNone(self.node.velocity_publisher)
        # Additional publisher verification tests


if __name__ == '__main__':
    rclpy.init()
    unittest.main()
    rclpy.shutdown()
```

#### Running Tests

```bash
# Run all tests in workspace
colcon test

# Run specific package tests
colcon test --packages-select my_robot_package

# Run with verbose output
colcon test --packages-select my_robot_package --event-handlers console_direct+

# Generate test coverage report
colcon test --packages-select my_robot_package --event-handlers console_direct+
colcon test-result --verbose
```

### Integration Testing

#### Hardware-in-the-Loop Testing

```bash
# Test with actual robot hardware
ros2 launch my_robot_bringup robot.launch.py

# In separate terminal, run integration tests
ros2 run my_robot_package integration_test

# Monitor system behavior
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /joint_states
```

#### Simulation Testing

```bash
# Launch robot in simulation
ros2 launch my_robot_bringup gazebo_world.launch.py

# Run simulation-based tests
ros2 run my_robot_package simulation_test

# Verify physics and sensor behavior
ros2 topic echo /scan
ros2 topic echo /imu/data
```

## Debugging and Profiling

### ROS 2 Debugging Tools

#### Topic Monitoring

```bash
# Monitor topic publication rates
ros2 topic hz /cmd_vel
ros2 topic hz /scan
ros2 topic hz /imu/data

# Monitor topic bandwidth usage
ros2 topic bw /cmd_vel
ros2 topic bw /scan

# Echo topic data with timestamps
ros2 topic echo /cmd_vel --csv > velocity_data.csv
```

#### Node Introspection

```bash
# List all running nodes
ros2 node list

# Get detailed node information
ros2 node info /node_name

# Check node parameters
ros2 param list /node_name
ros2 param get /node_name parameter_name

# Monitor node CPU and memory usage
ros2 doctor --report
```

#### Message Analysis

```bash
# Analyze message structure
ros2 interface show geometry_msgs/msg/Twist

# Check message compatibility
ros2 interface proto geometry_msgs/msg/Twist

# Validate message serialization
ros2 topic echo /cmd_vel --raw | xxd
```

### Performance Profiling

#### System Resource Monitoring

```bash
# Monitor Docker container resources
docker stats

# Check system resource usage
htop

# Monitor network traffic
iftop -i lo

# Check disk usage
df -h
du -sh ros2_ws/
```

#### ROS 2 Performance Tools

```bash
# Enable ROS 2 logging
export ROS_LOG_LEVEL=debug

# Profile node execution time
ros2 run my_robot_package performance_test

# Monitor message latency
ros2 topic delay /cmd_vel

# Check memory usage
ros2 doctor --report | grep memory
```

## Documentation Development

### Documentation Standards

#### README Files
- **Package README**: Include installation, usage, and API reference
- **Main Project README**: Comprehensive project overview and quick start
- **API Documentation**: Detailed interface specifications

#### Code Documentation

```python
def move_to_position(self, x: float, y: float, theta: float) -> bool:
    """
    Move robot to specified position and orientation.

    Args:
        x: Target X coordinate in meters
        y: Target Y coordinate in meters
        theta: Target orientation in radians

    Returns:
        bool: True if movement completed successfully

    Raises:
        NavigationError: If path planning fails
        TimeoutError: If movement times out

    Example:
        >>> controller.move_to_position(2.0, 1.5, 0.0)
        True
    """
```

### Documentation Build Process

```bash
# Generate ROS documentation
rosdoc2 build --package-path ros2_ws/src

# Generate API documentation
doxygen Doxyfile

# Build Sphinx documentation
cd docs/
sphinx-build -b html source build
```

## Version Control Best Practices

### Commit Message Guidelines

```bash
# Format: <type>(<scope>): <description>

feat(ros2-sim): Add SLAM integration for Navigation2
fix(dockerfile): Resolve Gazebo GPU acceleration issue
docs(README): Update installation instructions
test(navigation): Add path planning unit tests
refactor(controllers): Optimize PID control parameters
```

**Commit Types**:
- `feat`: New feature implementation
- `fix`: Bug fix
- `docs`: Documentation updates
- `test`: Test additions or modifications
- `refactor`: Code refactoring without functional changes
- `style`: Code style improvements
- `chore`: Maintenance tasks

### Branch Management

```bash
# Feature branch workflow
git checkout main
git pull origin main
git checkout -b feature/feature-name

# Development work...
git add .
git commit -m "feat: Implement new feature"

# Merge back to main
git checkout main
git merge feature/feature-name --no-ff
git push origin main

# Clean up feature branch
git branch -d feature/feature-name
```

## Quality Assurance

### Code Review Checklist

- [ ] Code follows project style guidelines
- [ ] New features have corresponding tests
- [ ] Documentation is updated for API changes
- [ ] No breaking changes to existing interfaces
- [ ] Performance impact is assessed and acceptable
- [ ] Error handling is comprehensive
- [ ] Security considerations are addressed

### Testing Requirements

#### Unit Tests
- Test all public methods and functions
- Cover edge cases and error conditions
- Mock external dependencies appropriately
- Achieve minimum 80% code coverage

#### Integration Tests
- Test component interactions
- Verify hardware interface compatibility
- Validate communication protocols
- Test error recovery mechanisms

#### System Tests
- End-to-end workflow validation
- Performance benchmarking
- Stress testing under load
- Long-duration stability testing

## Deployment and Release

### Release Process

#### Pre-release Checklist
- [ ] All tests pass
- [ ] Documentation is complete and accurate
- [ ] No breaking changes
- [ ] Performance benchmarks meet requirements
- [ ] Security audit completed

#### Release Steps
```bash
# Update version numbers
# Bump version in package.xml files
# Update changelog

# Create release branch
git checkout -b release/v1.0.0

# Tag release
git tag -a v1.0.0 -m "Release version 1.0.0"

# Push release
git push origin main
git push origin v1.0.0

# Create GitHub release
# Upload release notes and binaries
```

## Support and Maintenance

### Issue Tracking

#### GitHub Issues Categories
- **Bug Reports**: Unexpected behavior or crashes
- **Feature Requests**: New functionality requests
- **Documentation Issues**: Outdated or incorrect documentation
- **Enhancement Requests**: Performance or usability improvements

#### Issue Response Process
1. **Acknowledge**: Confirm issue receipt within 24 hours
2. **Reproduce**: Verify issue can be reproduced
3. **Investigate**: Identify root cause
4. **Fix**: Implement solution
5. **Test**: Verify fix resolves issue
6. **Document**: Update documentation if needed

### Maintenance Schedule

#### Daily Maintenance
- Monitor system health and performance
- Check log files for errors
- Verify backup integrity
- Update security patches

#### Weekly Maintenance
- Review and merge pull requests
- Update dependencies
- Run full test suite
- Performance benchmarking

#### Monthly Maintenance
- Security audits
- Documentation review
- Hardware calibration checks
- Long-term stability testing

## Advanced Development Topics

### Custom Message Types

#### Defining Custom Messages

```bash
# Create custom message package
ros2 pkg create my_robot_msgs --build-type ament_cmake --dependencies std_msgs

# Define custom message
# my_robot_msgs/msg/RobotStatus.msg
string status
float32 battery_level
int32 error_code
geometry_msgs/Pose pose

# Build and generate interfaces
colcon build --packages-select my_robot_msgs
source install/setup.bash
```

### Plugin Development

#### Gazebo Plugin Development

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class MyRobotPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      // Plugin initialization code
      this->model = model;

      // Connect to world update event
      this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MyRobotPlugin::OnUpdate, this));
    }

  private:
    void OnUpdate()
    {
      // Plugin update logic
    }

    physics::ModelPtr model;
    event::ConnectionPtr update_connection;
  };

  GZ_REGISTER_MODEL_PLUGIN(MyRobotPlugin)
}
```

### Real-time Systems Development

#### RTOS Integration

```cpp
// Real-time task scheduling
#include <chrono>
#include <thread>

void real_time_control_loop()
{
    auto next_time = std::chrono::steady_clock::now();

    while (running)
    {
        // Control algorithm execution
        compute_control_outputs();

        // Precise timing for real-time operation
        next_time += std::chrono::milliseconds(10); // 100 Hz
        std::this_thread::sleep_until(next_time);
    }
}
```

## Community Contribution

### Contributing Guidelines

1. **Fork and Clone**: Fork repository and create feature branch
2. **Development**: Implement changes following code standards
3. **Testing**: Add comprehensive tests for new functionality
4. **Documentation**: Update documentation for changes
5. **Pull Request**: Submit well-documented pull request
6. **Review**: Participate in code review process

### Community Standards

- **Respectful Communication**: Maintain professional and inclusive environment
- **Constructive Feedback**: Provide helpful suggestions and improvements
- **Knowledge Sharing**: Share insights and help other developers
- **Continuous Learning**: Stay updated with latest ROS 2 developments

## Resources and References

### Essential Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/iron/)
- [Gazebo Documentation](https://gazebosim.org/docs/)
- [Docker Documentation](https://docs.docker.com/)
- [n8n Documentation](https://docs.n8n.io/)

### Development Tools
- [ROS 2 Tutorials](https://docs.ros.org/en/iron/Tutorials.html)
- [Colcon Build System](https://colcon.readthedocs.io/)
- [ament Build System](https://github.com/ament/ament_cmake)
- [ROS 2 Testing Framework](https://github.com/ros2/launch_testing)

### Community Resources
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [GitHub ROS 2 Community](https://github.com/ros2)
- [ROS 2 YouTube Channel](https://www.youtube.com/c/ROS2)

---

*This development workflow guide provides comprehensive guidance for extending and maintaining the Autonomous Mobile Manipulator robot software stack. Follow the established patterns and contribute to the project's continued evolution.*
