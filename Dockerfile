#FROM nvcr.io/nvidia/isaac-sim:5.0.0
#
#ENV DEBIAN_FRONTEND=noninteractive
#COPY ros2-apt-source.deb /tmp/
#RUN apt-get update && apt-get install -y apt-transport-https
#
## https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#RUN apt-get install locales -y
#RUN locale-gen en_US en_US.UTF-8
#RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
#RUN export LANG=en_US.UTF-8
#
#RUN apt-get install software-properties-common -y
#RUN add-apt-repository universe -y
#
#RUN dpkg -i /tmp/ros2-apt-source.deb
#
#RUN apt-get update -y
#RUN apt-get upgrade -y
#RUN apt-get install ros-humble-ros-base -y
#RUN apt-get install ros-dev-tools -y
#RUN apt-get install ros-humble-vision-msgs
#RUN apt-get install ros-humble-ackermann-msgs

#Stage 1: Start with the Isaac Sim image
FROM nvcr.io/nvidia/isaac-sim:5.0.0 as isaac_sim

# Stage 2: Start with ROS 2 Humble image
FROM isaac_sim_ros:ubuntu_22_humble as ros2_base

# Copy Isaac Sim installation from Stage 1
COPY --from=isaac_sim /isaac-sim /isaac-sim
#COPY --from=isaac_sim /workspace/isaac_sim_ws /workspace/isaac_sim_ws

# Set up the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/isaac_sim_ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /workspace/isaac_sim_ws

# Start with bash shell
CMD ["/bin/bash"]

