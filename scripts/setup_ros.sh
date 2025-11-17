cd /${ROS_DISTRO}_ws
export FASTRTPS_DEFAULT_PROFILES_FILE=/${ROS_DISTRO}_ws/fastdds.xml
apt-get update
#If using docker, perform this step outside the container and relaunch the container
#git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
source /opt/ros/$ROS_DISTRO/setup.sh
colcon build
source install/local_setup.bash

apt-get install ros-humble-ur-robot-driver
apt-get install ros-humble-ur-moveit-config
