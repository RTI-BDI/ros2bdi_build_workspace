#!/bin/bash


# ---- clone additional repos from ros2 and ros-java ------------
cd ./src-java
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
rosdep install --from-paths src src-java -y -i --skip-keys "ament_tools" # as specified at https://github.com/ros2-java/ros2_java

# ---- from src/ros2_planning_system ----------
source ./install/setup.bash
colcon build --symlink-install --base-paths src/ros2_planning_system --event-handlers console_direct+ --packages-ignore plansys2_bt_actions 2>&1 | tee ./plansys.build.log

# ---- from src-java --------------------------
source ./install/setup.bash
# apt-get install ros-humble-rcl* # non serve
colcon build --symlink-install --base-paths src-java # to compile ros2-java up to rcljava; DO NOT WORK specifying the rcljava package or the folder
# colcon build --symlink-install --packages-up-to rcljava # non funziona, non trova rcl_interfaces

# ---- from src/JavaFF ------------------------
source ./install/setup.bash
colcon build --symlink-install --base-paths src/JavaFF --event-handlers console_direct+

# ---- from src/ROS2-BDI ----------------------
source ./install/setup.bash
colcon build --symlink-install --packages-select ros2_bdi_interfaces
# rm -rf ./build/ros2_bdi_interfaces

# ---- from src/javaff_interfaces -------------
source ./install/setup.bash
colcon build --symlink-install --base-paths src/javaff_interfaces
# colcon build --symlink-install --packages-select javaff_interfaces --packages-ignore unique_identifier_msgs action_msgs

# ---- from src/ROS2-BDI ----------------------
source ./install/setup.bash
colcon build --packages-select ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core --packages-skip ros2_bdi_interfaces
# colcon build --symlink-install --packages-select litter_world ros2_bdi_on_litter_world litter_world_interfaces webots_ros2_simulations_interfaces webots_ros2_simulations ros2_bdi_on_webots
colcon build --symlink-install --base-paths src/ROS2-BDI --packages-ignore ros2_bdi_tests --packages-skip ros2_bdi_interfaces ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core
colcon build --symlink-install --packages-select ros2_bdi_tests
