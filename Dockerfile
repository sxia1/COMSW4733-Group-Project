FROM nvcr.io/nvidia/isaac-sim:5.0.0

ENV DEBIAN_FRONTEND=noninteractive
ENV isaac_sim_package_path=${HOME}/isaacsim
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

# For X-11 Forwarding
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all 
ENV XAUTHORITY=/root/.Xauthority

COPY ros2-apt-source.deb /tmp/

RUN apt-get update && apt-get install -y apt-transport-https

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
RUN apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt-get install software-properties-common -y
RUN add-apt-repository universe -y

RUN dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install libbrotli1=1.0.9-2build6 --allow-downgrades -y
RUN apt-get install libfreetype6-dev -y
RUN apt-get install ros-humble-desktop -f -y
RUN apt-get install ros-humble-ros-base -y
RUN apt-get install ros-dev-tools -y
RUN apt-get install ros-humble-vision-msgs
RUN apt-get install ros-humble-ackermann-msgs
RUN apt-get install ros-humble-ros2-control -y
RUN apt-get install ros-humble-ros2-controllers -y
RUN apt-get install ros-$ROS_DISTRO-topic-based-ros2-control -y
RUN apt-get install ros-humble-xacro -y
RUN apt-get install ros-humble-eigen-stl-containers
RUN apt-get install xorg -y
RUN apt-get update

# https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon
RUN apt install python3-rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
RUN apt update
RUN apt dist-upgrade

RUN apt install python3-colcon-common-extensions
RUN apt install python3-colcon-mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default
RUN apt install python3-vcstool

#RUN mkdir -p ~/ws_moveit/src
#RUN cd ~/ws_moveit/src
#RUN git clone -b humble https://github.com/moveit/moveit2_tutorials
#RUN apt update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
#RUN cd ~/ws_moveit
#RUN colcon build --mixin release

##Stage 1: Start with the Isaac Sim image
#FROM nvcr.io/nvidia/isaac-sim:5.0.0 as isaac_sim
#
## Stage 2: Start with ROS 2 Humble image
#FROM isaac_sim_ros:ubuntu_22_humble as ros2_base
#
## Copy Isaac Sim installation from Stage 1
#COPY --from=isaac_sim /isaac-sim /isaac-sim
##COPY --from=isaac_sim /workspace/isaac_sim_ws /workspace/isaac_sim_ws
#
## Set up the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/isaac_sim_ws/install/setup.bash" >> ~/.bashrc
#
## Set working directory
#WORKDIR /workspace/isaac_sim_ws
#
## Start with bash shell
#CMD ["/bin/bash"]
#
