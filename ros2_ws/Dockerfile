# Default Dockerfile for docker-compose (ROS 2 Jazzy)
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-robot-localization \
    ros-jazzy-rplidar-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    procps \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --break-system-packages flask pyserial smbus2 mpu6050-raspberrypi spidev

# ROS 2 workspace
WORKDIR /root/ros2_ws
COPY . .
RUN . /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select my_robot_description my_robot_bringup my_robot_automation --continue-on-error

CMD ["/bin/bash"]
