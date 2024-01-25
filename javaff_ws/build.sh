#!/bin/bash

NUM_CORES=$(nproc) && \
export MAKEFLAGS="-j$NUM_CORES" && \
export AMENT_BUILD_GRADLE_ARGS="-j$NUM_CORES" && \
export AMENT_BUILD_MAKE_ARGS="-j$NUM_CORES"

colcon build --symlink-install
