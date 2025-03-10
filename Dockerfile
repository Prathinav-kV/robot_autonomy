# Use ROS 2 Humble as base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle

# Install necessary dependencies
RUN apt update && apt install -y \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/robot_autonomy

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select robot_autonomy"

# Copy the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
