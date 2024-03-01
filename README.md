
# A docker-based build environment for ROS2-BDI

This repository includes a build environment (based on docker) for ROS2-BDI.

External repos from RTI-BDI are here included in `./ros2bdi_ws/src` as git submodules. Changes needs to be committed directly to the original repos. Then, commit this repo (to commit the updated hash-referenced-commit of the external-repo, on which each submodule refers to).

#### Submodule:

 - https://github.com/RTI-BDI/ROS2-BDI/tree/humble_online
 - https://github.com/RTI-BDI/ros2_planning_system/tree/humble
 - https://github.com/RTI-BDI/JavaFF/tree/squashed



## Building
Code building should be done within docker. Here are the basics steps to getting started:

0. Initialize submodule repositories:
    ```console
    git submodule init
    git submodule update
    ```

1. Build docker image for developing environment:
    ```console
    sudo make docker-buid
    ```

2. Run developing environment:
    ```console
    make docker-login
    ```

3. Clone additional repositories from ros2 and ros2-java (repos taken from [https://github.com/ros2-java/ros2_java](https://github.com/ros2-java/ros2_java)):
    ```console
    cd ./src
    vcs import < ros2_java_desktop.repos
    ```

3. Build within docker:
    ```console
    root@ros2bdi-build-env: ~/ros2bdi_ws ./build_all.sh
    ```



## Running

1. Configure Webots to run locally:
    
    We will run webots on host machine (while ros2bdi and plansys2 are running on docker).
    To do so, Webots 2023_b needs to be installed and webots local simulation python server should be listening on port 2000:

    On LINUX
    ```console
    make webots-linux
    ```

    On macOS, because docker is executed as a VM, network is isolated. So we need to forward request from docker VM to host machine. The forward proxy services 2000 and 1234 allow the local simulation server to connect to the Webots simulator. On Linux and Windows, these two proxy services are not needed.
    ```console
    make webots-mac
    ```

2. Manually run nodes:

    Type `make` to get list of commands:

    ```console
    $ make
    Webots executes natively on host machine, be sure to run webots server.
    On mac, docker runs on VM, be sure to run forwarding services from VM to host network. Forwarding services NOT running

    docker-login                   Login to the container. If the container is already running, login into existing one.
    docker-build                   Build the docker ros2bdi-build-env image.
    run-world                      Login and run world.
    run-carrier_a                  Login and run carrier_a.
    run-carrier_b                  Login and run carrier_b.
    run-carrier_c                  Login and run carrier_c.
    run-gripper_a                  Login and run gripper_a.
    run-all                        Login and run all.
    webots-mac                     Run local simulation server for Webots on mac.
    webots-linux                   Run local simulation server for Webots on linux.
    ```

    For example, to run world:
    ```console
    make run-world
    ```
    
    Or specify the command to run in the docker after `make doker-login`:
    ```console
    make doker-login ros2 launch webots_ros2_simulations blocks_world.launch.py
    ```

3. Alternatively, with ***docker compose***:

    ```console
    sudo docker compose up
    ```
    
    Or run selected services one-by-one manually:
    ```console
    sudo docker compose blocks_world
    sudo docker compose carrier_a
    sudo docker compose carrier_b
    sudo docker compose carrier_c
    sudo docker compose gripper_a
    ```
