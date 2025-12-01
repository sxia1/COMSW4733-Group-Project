# COMSW4733-Group-Project


# Installing Issac Sim on gcloud
Installing Nvidia Driver
```
curl -L https://storage.googleapis.com/compute-gpu-installation-us/installer/latest/cuda_installer.pyz --output cuda_installer.pyz
sudo python3 cuda_installer.pyz install_driver
```

Installing Docker
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

Creating Docker Group
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Installing the NVIDIA Container Toolkit packages
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update

sudo apt-get install -y nvidia-container-toolkit
```

Setup Nvidia Container Toolkit
```
sudo systemctl restart docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify NVIDIA Container Toolkit
```
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

Running the Container and IssacSim
```
docker pull nvcr.io/nvidia/isaac-sim:5.0.0
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:5.0.0

PUBLIC_IP=$(curl -s ifconfig.me) && ./runheadless.sh --/app/livestream/publicEndpointAddress=$PUBLIC_IP --/app/livestream/port=49100
```

Install Isaac Sim WebRTC Streaming Client on your Local Desktop  
Run the client using the public ip address of the gcloud Spot Instance  


# Runing Moveit2 Docker Container Locally
Note that the command on the official website for Humble Moveit2, doesn't actually work. Please refer to the General Guide Instead

Running Moveit2 Docker container
```
wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/.docker/docker-compose.yml
# Running with cpu not gpu on local system
DOCKER_IMAGE=humble-humble-tutorial-source docker compose run --rm --name moveit2_container cpu
# Rolling Ridley Distro Option, because Setup Assistant was giving me issues in Humble Distro
DOCKER_IMAGE=main-rolling-tutorial-source docker compose run --rm --name moveit2_container cpu
```

Running Moveit2 Setup Assistant inside the Docker Container
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py --debug
```

Building and Running ur5e\_cutter Packages
```
cd ~/ws_moveit/
colcon build --packages-select ur5e_cutter_description
colcon build --packages-select ur5e_cutter_moveit_config
source install/local_setup.bash
ros2 launch ur5e_cutter_moveit_config demo.launch.py
```

# Configuring X11 forwarding

Copy gcloud ssh command and add -X ssh flag
```
gcloud compute ssh --ssh-flag="-X" --zone "<ZONE>" "<INSTANCE>" --project "<PROJECT>"
```

Everything else is handled in the Dockerfiles, but here are some commands
```
xclock
xauth list
echo $DISPLAY
echo $XAUTHORITY
```

# Running Moveit2 and IsaacSim Integration
```
cd COMS4733-Group-Project
# builds the Docker Image and starts the interactive container
. scripts/my_start_isaac_sim.sh
. /root/scripts/start_isaac_sim.sh

# Open a new Terminal session
docker exec -it $user-isaac-sim bash
cd /root/ws_moveit2/
. /root/scripts/setup_ros.sh

colcon build # first build will take approximately an hour
colcon build --packages-select ur5e_cutter_moveit_config # for subsequent builds where you are only changing this package
source install/setup.bash
ros2 launch ur5e_cutter_moveit_config isaac_demo.launch.py
```

In Isaac Sim, open and run the create\_scene.py file in the scripts editor window. Follow the instructions below to create the Action Graph and press the play button.

In the Moveit2 rviz GUI, select the planning algorithm on the context tab (testing with BiRRT). In the planning tab, set Goal State to ready and click "Plan & Execute"

It is possible that the algorithm does not successfully find a path given the constraints first try. This is expected.


# Creating the Action Graph in IsaacSim

Creating the Action Graph in Python hasn't been completed yet. This is how to do it manually from the GUI.

1. Tools > Robotics > ROS 2 OmniGraphs > Joint States
2. Select /World/ur5e\_cutter in Articulation Root Add
3. Change Publisher Topic to /isaac\_joint\_states
4. Change Subscriber Topic to /isaac\_joint\_commands
5. Check off the Publisher and Subcriber Boxes
6. Click OK

Fixing the joint state publisher: Add an articulation root to shoulder\_link and select that as the targetPrim instead


# Official Documentation
[How to Install Isaac Sim on Google Cloud](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_advanced_cloud_setup_gcp.html)  

[Install GPU Driver](https://cloud.google.com/compute/docs/gpus/install-drivers-gpu#linux)  

[Issac Sim Latest Releases](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html#isaac-sim-latest-release)  

[General Moveit2 Docker Guide](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html)

[Humble Moveit2 Docker Guide](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html)

[Moveit2 Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

[X11 Forwarding](https://www.simplified.guide/ssh/x11-forwarding-as-root)
