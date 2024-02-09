#!/bin/bash

NUM_CORES=$(nproc) && \
export MAKEFLAGS="-j$NUM_CORES" && \
export AMENT_BUILD_GRADLE_ARGS="-j$NUM_CORES" && \
export AMENT_BUILD_MAKE_ARGS="-j$NUM_CORES"

# lanciare prima di ros2bdi

colcon build --symlink-install --packages-select ament_java_resources ament_build_type_gradle ament_cmake_export_jars ament_cmake_export_jni_libraries rcljava_common rosidl_generator_java
colcon build --symlink-install --packages-select javaff_interfaces --packages-ignore rosidl_default_generators rosidl_default_runtime

# da qui lanciare dopo ros2bdi

colcon build --symlink-install
