# Multi-stage Dockerfile: build stage + runtime (install) stage

# ----------------------
# Build stage: compile SDKs and build workspace
# ----------------------
FROM arm64v8/ros:jazzy AS builder

# Install build-time tooling and ROS packages required to build packages
RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-pip \
    libboost-all-dev \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy-linux

# Workspace layout
RUN mkdir -p /home/ws/src
WORKDIR /home/ws

# Copy only the required packages into the builder
COPY ./src/YDLidar-SDK /home/ws/src/YDLidar-SDK
COPY ./src/ydlidar_ros2_driver /home/ws/src/ydlidar_ros2_driver
COPY ./src/ubiquity_motor_ros2 /home/ws/src/ubiquity_motor_ros2
COPY ./src/magni_description /home/ws/src/magni_description
COPY ./src/magni_bringup /home/ws/src/magni_bringup
COPY ./src/magni_teleop /home/ws/src/magni_teleop

# Build YDLidar SDK
WORKDIR /home/ws/src/YDLidar-SDK
RUN mkdir -p build
WORKDIR build
RUN cmake ..
RUN make
RUN make install

# Build the ROS2 workspace
WORKDIR /home/ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --base-paths \
    src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs \
    src/ubiquity_motor_ros2 \
    --cmake-args --event-handlers console_direct+

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --base-paths \
    src/magni_description \
    src/ydlidar_ros2_driver \
    src/magni_bringup \
    src/magni_teleop

# ----------------------
# Runtime stage: smaller image based on ros-core variant
# ----------------------
FROM arm64v8/ros:jazzy-ros-core AS runtime

# Install only runtime packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    libboost-all-dev \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy-linux

# Create workspace layout and copy artifacts from the builder stage
WORKDIR /home/ws
COPY --from=builder /home/ws/install /home/ws/install
# Copy any system-installed files (e.g. YDLidar SDK installed into /usr/local)
COPY --from=builder /usr/local /usr/local
#Copy scripts
COPY ./scripts/run-magni-container.sh /run-magni-container.sh
RUN chmod +x /run-magni-container.sh

# Ensure the container entrypoint sources the workspace install
RUN sed --in-place --expression \
    '$isource "/home/ws/install/setup.bash"' \
    /ros_entrypoint.sh || true

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "magni_bringup", "magni_bringup.launch.py"]
