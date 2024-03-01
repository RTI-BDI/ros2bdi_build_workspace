#!/bin/bash


# ---- clone additional repos from ros2 and ros-java ------------
cd ./src
vcs import < ros2_java_desktop.repos
cd ..


# ---- use all cores to build ---------------
NUM_CORES=$(nproc) && \
export MAKEFLAGS="-j$NUM_CORES" && \
export AMENT_BUILD_GRADLE_ARGS="-j$NUM_CORES" && \
export AMENT_BUILD_MAKE_ARGS="-j$NUM_CORES"


# ---- rosdep install -----------------------
apt-get update
# rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
rosdep install --from-paths src -y -i --skip-keys "ament_tools" # as specified at https://github.com/ros2-java/ros2_java

# ---- from ros2_planning_system ------------
colcon build --symlink-install --base-paths src/ros2_planning_system --packages-ignore plansys2_bt_actions 2>&1 | tee ./plansys.build.log

# ---- from ros2 --------------------------
colcon build --symlink-install --base-paths src/ros2
colcon build --symlink-install --base-paths src/ros2-java #--packages-select rcljava
# colcon build --symlink-install --packages-select ament_java_resources ament_build_type_gradle ament_cmake_export_jars ament_cmake_export_jni_libraries rcljava_common rosidl_generator_java
# colcon build --symlink-install --packages-select rosidl_default_generators rosidl_default_runtime builtin_interfaces std_msgs
# colcon build --symlink-install --base-paths /opt/ros/humble --packages-select builtin_interfaces std_msgs

# ---- from JavaFF --------------------------
colcon build --symlink-install --base-paths src/JavaFF --event-handlers console_direct+

# ---- from ros2_planning_system ------------
colcon build --symlink-install --packages-select plansys2_msgs

# ---- from ROS2-BDI ------------------------
colcon build --symlink-install --packages-select ros2_bdi_interfaces
# rm -rf /root/ros2bdi_ws/build/ros2_bdi_interfaces

# ---- from JavaFF --------------------------
colcon build --symlink-install --packages-select javaff_interfaces --packages-ignore unique_identifier_msgs action_msgs

# ---- from ROS2-BDI ------------------------
colcon build --packages-select ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core --packages-skip ros2_bdi_interfaces

# ---- from ROS2-BDI ------------------------
colcon build --symlink-install --packages-ignore ros2_bdi_tests --packages-skip ros2_bdi_interfaces ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core

# ---- from ROS2-BDI ------------------------
source /root/ros2bdi_ws/install/setup.bash
colcon build --symlink-install --packages-select ros2_bdi_tests

# ---- from ROS2-BDI ------------------------
# colcon build --symlink-install --packages-select webots_ros2_simulations_interfaces webots_ros2_simulations ros2_bdi_on_webots

# ---- from JavaFF --------------------------
colcon build --symlink-install --base-paths src/JavaFF --packages-skip javaff_interfaces ament_java_resources ament_build_type_gradle ament_cmake_export_jars ament_cmake_export_jni_libraries rcljava_common rosidl_generator_java


