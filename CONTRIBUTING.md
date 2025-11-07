# Contributing to Autonomous Mobile Manipulator

This document provides guidelines for contributing to the Autonomous Mobile Manipulator project.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Environment Setup](#development-environment-setup)
- [Code Contribution Guidelines](#code-contribution-guidelines)
- [Testing Requirements](#testing-requirements)
- [Documentation Standards](#documentation-standards)
- [Pull Request Process](#pull-request-process)
- [Issue Reporting](#issue-reporting)
- [Code of Conduct](#code-of-conduct)

## Getting Started

### Prerequisites

Before contributing, ensure you have:

- Git for version control
- Docker and Docker Compose for containerized development
- Basic understanding of ROS 2, Python, and robotics concepts
- Familiarity with Linux development environment

### Repository Setup

```bash
# Clone the repository
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git
cd Autonomous_Mobile_Manipulator

# Create a feature branch for your work
git checkout -b feature/your-feature-name
```

## Development Environment Setup

### Local Development

1. **Install Dependencies**
   ```bash
   # Install Docker and Docker Compose
   # Follow the installation guide in docs/installation/README.md
   ```

2. **Start Development Environment**
   ```bash
   # Start with sensor simulation for development
   ./run.sh --dev

   # Or start full production environment
   ./run.sh
   ```

3. **Access Development Interfaces**
   - n8n Workflow Editor: http://localhost:5678
   - Robot REST API: http://localhost:5000
   - ROS 2 Development Container: `docker exec -it ros2_sim_container bash`

### ROS 2 Development

```bash
# Access ROS 2 development environment
docker exec -it ros2_sim_container bash

# Source ROS 2 workspace
cd /root/ros2_ws
source install/setup.bash

# Build workspace
colcon build

# Run tests
colcon test
```

### Hardware Development

For hardware-related contributions:

1. Review hardware specifications in `docs/hardware/`
2. Test on supported platforms (Ubuntu/Debian, Raspberry Pi 5)
3. Document any hardware requirements or modifications
4. Include hardware testing procedures

## Code Contribution Guidelines

### Python Code Standards

- Follow PEP 8 style guidelines
- Use type hints for function parameters and return values
- Write docstrings for all public functions and classes
- Use meaningful variable and function names
- Keep functions focused on single responsibilities

```python
def calculate_motor_speed(distance: float, time: float) -> float:
    """
    Calculate motor speed based on distance and time.

    Args:
        distance: Distance to travel in meters
        time: Time to complete travel in seconds

    Returns:
        Motor speed in m/s
    """
    if time <= 0:
        raise ValueError("Time must be positive")
    return distance / time
```

### ROS 2 Package Structure

- Follow ROS 2 package naming conventions
- Include package.xml and CMakeLists.txt for each package
- Use launch files for system startup
- Include parameter files for configuration
- Document all services, topics, and actions
- Follow ROS 2 message and service naming conventions
- Include proper error handling in ROS 2 nodes

### n8n Workflow Standards

- Use descriptive node names and descriptions
- Include error handling in workflow nodes
- Document workflow inputs, outputs, and logic
- Test workflows with various scenarios
- Include workflow metadata and tags

### Commit Message Guidelines

Use clear, descriptive commit messages:

```
feat: add obstacle avoidance algorithm
fix: resolve motor control timing issue
docs: update API documentation
test: add unit tests for sensor interface
```

### Branch Naming

- `feature/feature-name`: New features
- `fix/issue-description`: Bug fixes
- `docs/update-description`: Documentation updates
- `test/test-description`: Testing improvements

## Testing Requirements

### Unit Testing

- Write unit tests for all Python modules
- Use pytest framework
- Achieve minimum 80% code coverage
- Test both success and failure scenarios

```python
import pytest
from your_module import calculate_motor_speed

def test_calculate_motor_speed():
    assert calculate_motor_speed(10.0, 2.0) == 5.0

def test_calculate_motor_speed_zero_time():
    with pytest.raises(ValueError):
        calculate_motor_speed(10.0, 0.0)
```

### Integration Testing

- Test ROS 2 node interactions
- Verify n8n workflow functionality
- Test hardware interfaces when possible
- Include system-level integration tests

### ROS 2 Testing

```bash
# Run ROS 2 tests
colcon test --packages-select your_package
colcon test-result --verbose
```

### Hardware Testing

For hardware contributions:

- Test on multiple platforms when possible (Ubuntu/Debian, Raspberry Pi 5)
- Include sensor validation procedures
- Document hardware testing results
- Verify safety systems functionality
- Test emergency stop mechanisms
- Validate motor control algorithms
- Confirm sensor calibration procedures

## Documentation Standards

### Code Documentation

- Include docstrings for all public functions
- Document class attributes and methods
- Explain complex algorithms and logic
- Include usage examples where helpful

### Project Documentation

- Update relevant documentation files
- Include installation instructions for new features
- Document configuration changes
- Update API documentation for new endpoints

### Commit Documentation

- Reference issue numbers in commits
- Include rationale for significant changes
- Document breaking changes clearly

## Pull Request Process

### Before Submitting

1. **Test Your Changes**
   ```bash
   # Run all tests
   colcon test

   # Verify no linting errors
   # Run code quality checks
   ```

2. **Update Documentation**
   - Update relevant documentation
   - Add tests for new functionality
   - Update changelog if applicable

3. **Code Review Checklist**
   - [ ] Code follows style guidelines
   - [ ] All tests pass
   - [ ] Documentation updated
   - [ ] No breaking changes without discussion
   - [ ] Hardware changes tested on target platforms

### Submitting a Pull Request

1. **Create Pull Request**
   - Use descriptive title and description
   - Reference related issues
   - Include screenshots for UI changes
   - Describe testing performed

2. **Pull Request Template**
   ```
   ## Description
   Brief description of changes

   ## Type of Change
   - [ ] Bug fix
   - [ ] New feature
   - [ ] Breaking change
   - [ ] Documentation update

   ## Testing
   Describe testing performed

   ## Checklist
   - [ ] Tests pass
   - [ ] Documentation updated
   - [ ] Code follows style guidelines
   ```

3. **Review Process**
   - At least one maintainer review required
   - Address review comments
   - CI checks must pass
   - Squash commits before merge

## Issue Reporting

### Bug Reports

When reporting bugs, include:

- **Description**: Clear description of the issue
- **Steps to Reproduce**: Step-by-step reproduction instructions
- **Expected Behavior**: What should happen
- **Actual Behavior**: What actually happens
- **Environment**: OS, hardware, software versions
- **Logs**: Relevant log output
- **Screenshots**: If applicable

### Feature Requests

For new features, include:

- **Problem Statement**: What problem needs solving
- **Proposed Solution**: How to solve it
- **Alternatives Considered**: Other approaches
- **Use Cases**: Who benefits and how

### Issue Labels

- `bug`: Software defects
- `enhancement`: New features
- `documentation`: Documentation issues
- `hardware`: Hardware-related issues
- `question`: Questions and discussions
- `help wanted`: Good first issues

## Code of Conduct

### Our Standards

- Be respectful and inclusive
- Focus on constructive feedback
- Accept responsibility for mistakes
- Show empathy towards other contributors
- Use welcoming and inclusive language
- Prioritize safety in robotics development
- Consider hardware and real-world implications

### Unacceptable Behavior

- Harassment or discriminatory language
- Personal attacks or trolling
- Publishing private information
- Spam or off-topic content
- Any conduct that violates legal requirements
- Sharing untested or unsafe robotics code
- Disregarding hardware safety protocols

### Safety First

Given the robotics nature of this project:

- Always test code in simulation before hardware deployment
- Include safety checks in all hardware-related code
- Document potential safety risks
- Report safety concerns immediately
- Follow ROS 2 and robotics safety best practices

### Enforcement

Violations of the code of conduct will be addressed by project maintainers. Serious violations, especially those compromising safety, may result in immediate removal from the project.

## Getting Help

- **Documentation**: Check `docs/` directory first
- **Issues**: Search existing issues before creating new ones
- **Discussions**: Use GitHub Discussions for questions
- **Community**: Join ROS 2 and robotics communities for additional support

## Recognition

Contributors are recognized through:
- GitHub contributor statistics
- Mention in release notes
- Attribution in documentation
- Community recognition

Thank you for contributing to the Autonomous Mobile Manipulator project!
