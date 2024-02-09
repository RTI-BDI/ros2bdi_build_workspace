#!/bin/bash

NUM_CORES=$(nproc) && \
export MAKEFLAGS="-j$NUM_CORES" && \
export AMENT_BUILD_GRADLE_ARGS="-j$NUM_CORES" && \
export AMENT_BUILD_MAKE_ARGS="-j$NUM_CORES"



apt-get update

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}



# ---- plansys2 ------------
colcon build --symlink-install --base-paths src/ros2_planning_system --packages-ignore plansys2_bt_actions 2>&1 | tee ./plansys.build.log

# ---- javaff ------------
colcon build --symlink-install --packages-select ament_java_resources ament_build_type_gradle ament_cmake_export_jars ament_cmake_export_jni_libraries rcljava_common rosidl_generator_java

# ---- javaff ------------
colcon build --symlink-install --packages-select rosidl_default_generators rosidl_default_runtime builtin_interfaces std_msgs

# ---- plansys2 ------------
colcon build --symlink-install --packages-select plansys2_msgs unique_identifier_msgs action_msgs

# ---- ros2bdi ------------
colcon build --symlink-install --packages-select ros2_bdi_interfaces
# rm -rf /root/ros2bdi_ws/build/ros2_bdi_interfaces

# ---- javaff ------------
colcon build --symlink-install --packages-select javaff_interfaces --packages-ignore unique_identifier_msgs action_msgs

# ---- ros2bdi ------------
colcon build --packages-select ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core --packages-skip ros2_bdi_interfaces

# ---- ros2bdi ------------
colcon build --symlink-install --packages-ignore ros2_bdi_tests --packages-skip ros2_bdi_interfaces ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core

# ---- ros2bdi ------------
source /root/ros2bdi_ws/install/setup.bash
colcon build --symlink-install --packages-select ros2_bdi_tests

# ---- ros2bdi ------------
# colcon build --symlink-install --packages-select webots_ros2_simulations_interfaces webots_ros2_simulations ros2_bdi_on_webots

# ---- javaff ------------
colcon build --symlink-install --base-paths src/JavaFF --packages-skip javaff_interfaces ament_java_resources ament_build_type_gradle ament_cmake_export_jars ament_cmake_export_jni_libraries rcljava_common rosidl_generator_java


