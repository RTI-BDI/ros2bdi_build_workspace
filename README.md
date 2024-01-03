
# A docker-based build environment for ROS2BDI, including specific modified ros2_planning_system and all additional needed dependencies.

This repository can be used to setup a building and runnning environment based on docker.

1. Build docker image for developing environment:

    > $ sudo docker build --platform=linux/amd64 --rm  --tag ros2bdi-build-env -f DockerfileBuildEnv .

2. Run developing environment:

    > $ docker run --platform=linux/amd64 -v ./ros2bdi_ws:/root/ros2bdi_ws -v ./plansys2_ws:/root/plansys2_ws -v ./tmp:/root/tmp --rm -it ros2bdi-build-env bash

3. Build ROS2BDI

    > root@ros2bdi-build-env:~/ros2bdi_ws ./src/ROS2-BDI/build.sh

4. Build docker image for running environment

    > $ sudo docker build --platform=linux/amd64 --rm  --tag ros2bdi-run-env --file DockerfileRunEnv .

5. Configure Webots to run locally:
    
    We will run webots on host machine (while ros2bdi and plansys2 are running on docker).
    To do so, Webots 2023_b needs to be installed and webots local simulation python server whould be listening on port 2000:

    On LINUX

    > $ export WEBOTS_HOME=/usr/local/webots

    > $ python3 local_simulation_server.py
    
    On MAC, because docker is executed as a VM, network is isolated. So we need to forward request from docker VM to host machine. Use script local_simulation_server.sh for simplicity:

    > $ ./local_simulation_server.sh

6. Manually run nodes:

    world

    > $ docker run --platform=linux/amd64 -v .:/root/ros2bdi_ws -v /Users/Shared/shared:/root/shared -e "WEBOTS_SHARED_FOLDER=/Users/Shared/shared:/root/shared" --rm -it --name ros2bdi ros2bdi-run-env ros2 launch webots_ros2_simulations blocks_world.launch.py

    agents

    > $ docker run --platform=linux/amd64 -v ./tmp:/tmp --rm -it ros2bdi-run-env ros2 launch ros2_bdi_on_webots carrier_a.launch.py
    
    > $ docker run --platform=linux/amd64 -v ./tmp:/tmp --rm -it ros2bdi-run-env ros2 launch ros2_bdi_on_webots carrier_b.launch.py
    
    > $ docker run --platform=linux/amd64 -v ./tmp:/tmp --rm -it ros2bdi-run-env ros2 launch ros2_bdi_on_webots carrier_c.launch.py
    
    > $ docker run --platform=linux/amd64 -v ./tmp:/tmp --rm -it ros2bdi-run-env ros2 launch ros2_bdi_on_webots gripper_a.launch.py

6. Alternatively, with docker compose

    > $ sudo docker compose up