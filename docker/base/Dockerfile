FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Basic development/debugging tools only
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    wget \
    nano \
    vim \
    iputils-ping \
    net-tools \
    python3-pip \
    python3-venv \
    python3-dev \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Make ROS available in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace

CMD ["/bin/bash"]
