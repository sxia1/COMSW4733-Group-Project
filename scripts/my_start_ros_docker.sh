xhost +
docker run -it --rm --net=host --env="DISPLAY" --env="ROS_DOMAIN_ID" \
    -v ~/IsaacSim-ros_workspaces/humble_ws:/humble_ws \
    -v ~/COMSW4733-Group-Project:/root:rw \
    --name ros_ws_docker osrf/ros:humble-desktop /bin/bash
