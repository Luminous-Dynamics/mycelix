# Mycelix Docker Container
# Multi-stage build for minimal final image

# Stage 1: Build environment
FROM ubuntu:22.04 AS builder

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Rust for Holochain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install Node.js for Holochain conductor
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    libwebsocketpp-dev \
    libjsoncpp-dev \
    libssl-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
WORKDIR /mycelix
COPY . .

# Build the project
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)"

# Stage 2: Runtime environment
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    nodejs \
    && rm -rf /var/lib/apt/lists/*

# Copy built artifacts from builder
WORKDIR /mycelix
COPY --from=builder /mycelix/build ./build
COPY --from=builder /mycelix/launch ./launch
COPY --from=builder /mycelix/config ./config
COPY --from=builder /mycelix/scripts ./scripts

# Copy demo script
COPY --from=builder /mycelix/demo.sh ./demo.sh
RUN chmod +x demo.sh

# Expose ports
EXPOSE 8888 9000 11311

# Set up entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["./demo.sh"]