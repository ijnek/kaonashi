FROM osrf/ros:rolling-desktop-full

# Update all packages
RUN apt update && apt upgrade -y

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
