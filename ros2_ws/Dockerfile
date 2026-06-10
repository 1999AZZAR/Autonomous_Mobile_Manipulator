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
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --break-system-packages --ignore-installed numpy \
    flask flask-cors pyserial smbus2 mpu6050-raspberrypi spidev requests \
    prisma psycopg2-binary redis httpx opencv-python-headless

# Install Node.js + Prisma CLI
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g prisma && \
    rm -rf /var/lib/apt/lists/*
ENV PATH="/usr/local/lib/node_modules/.bin:${PATH}"

# ROS 2 workspace
WORKDIR /root/ros2_ws
COPY . .

CMD ["/bin/bash"]
