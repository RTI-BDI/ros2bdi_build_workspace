#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash



NUM_CORES=$(nproc) && \
export MAKEFLAGS="-j$NUM_CORES" && \
export AMENT_BUILD_GRADLE_ARGS="-j$NUM_CORES" && \
export AMENT_BUILD_MAKE_ARGS="-j$NUM_CORES"


apt-get update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
colcon build --symlink-install --base-paths src 2>&1 | tee ./build.log