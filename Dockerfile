FROM osrf/ros:melodic-desktop-full
MAINTAINER Matthew Barulic matthew.barulic@gmail.com

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install apt-utils git software-properties-common && apt-get clean

# Initialize catkin workspace
RUN mkdir -p ~/catkin_ws
WORKDIR ~/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace"
RUN mkdir -p src

COPY . ./src/roboracing-software

# Install all ROS dependencies that can automatically be installed
# We're ignoring rosdep keys that are only required for interacting with hardware
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && rosdep install -iy --from-paths ./src  --skip-keys='realsense_camera pointgrey_camera_driver libuvc_camera'"
