# Pull the rosgzpx4 image
FROM ghcr.io/srindot/rosgzpx4

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/rosusr/ws_control_fixed_wing/src/

# Update packages and install required dependencies
RUN apt-get update && apt-get install -y ros-humble-tf2 ros-humble-tf2-ros

# Create and build the ws_offboard_control workspace
RUN git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git 

# Install PlotJuggler
RUN apt-get install -y ros-humble-plotjuggler-ros

# Give ownership of the user's home directory
RUN chown -R rosusr:rosusr /home/rosusr

# Set the default command to open a bash shell
CMD ["bash"]
