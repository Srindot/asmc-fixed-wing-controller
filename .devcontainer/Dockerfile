# Pull the rosgzpx4 image
FROM rosgzpx4:latest

# Add the following labels
LABEL org.opencontainers.image.description="uav assignment"
LABEL org.opencontainers.image.title="UAVdevcon"
LABEL org.opencontainers.image.vendor="B Srinath Dhatre"
LABEL org.opencontainers.image.source="https://github.com/Srindot/assignment2_Introduction_To_UAV_Design.git"
LABEL maintainer="bsdhatre@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# Grant ownership of the home directory to rosusr
USER root
RUN chown -R rosusr:rosusr /home/rosusr

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Switch to bash shell
SHELL ["/bin/bash", "-c"]

WORKDIR /home/rosusr

# Update packages and install required dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
    python3-colcon-common-extensions && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create and build the ws_sensor_combined workspace
RUN mkdir -p /home/rosusr/ws_sensor_combined/src && \
    cd /home/rosusr/ws_sensor_combined/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd .. && \
    . /opt/ros/humble/setup.bash && \
    colcon build && \
    . install/local_setup.bash

# Create and build the ws_offboard_control workspace
RUN mkdir -p /home/rosusr/ws_offboard_control/src && \
    cd /home/rosusr/ws_offboard_control/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd .. && \
    . /opt/ros/humble/setup.bash && \
    colcon build && \
    . install/local_setup.bash

RUN pip install git+https://github.com/fishros/ros2bag_convert.git

# Set the default command to open a bash shell
CMD ["bash"]
