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

# colcon graph --legend --dot --base-path src src-java | dot -Tpng -o graph.png


# ---- auto-resolve build dependencies --------
# https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html
# colcon build --symlink-install --base-paths src src-java --packages-up-to litter_world ros2_bdi_on_litter_world webots_ros2_simulations ros2_bdi_on_webots


# ---- from src-java/ -------------------------
source ./install/setup.bash
colcon build --symlink-install --base-paths src-java 2>&1 | tee ./rcljava.build.log # to compile ros2-java up to rcljava; DO NOT WORK specifying the rcljava package or the folder
# colcon build --symlink-install --packages-up-to rcljava # non funziona, non trova rcl_interfaces

# ---- from src/ros2_planning_system ----------
source ./install/setup.bash
colcon build --symlink-install --base-paths src/ros2_planning_system --event-handlers console_direct+ --packages-ignore plansys2_bt_actions 2>&1 | tee ./plansys.build.log # contains plansys2_msgs

# ---- from src/JavaFF ------------------------
source ./install/setup.bash
colcon build --symlink-install --base-paths src/JavaFF --event-handlers console_direct+ 2>&1 | tee ./javaff.build.log # depends on: rcljava plansys2_msgs lifecycle_msgs example_interfaces javaff_interfaces

# ---- from src/ROS2-BDI ----------------------
source ./install/setup.bash
colcon build --symlink-install --packages-select ros2_bdi_interfaces 2>&1 | tee ./ros2_bdi_interfaces.build.log
# rm -rf ./build/ros2_bdi_interfaces

# ---- from src/javaff_interfaces -------------
source ./install/setup.bash
colcon build --symlink-install --packages-select javaff_interfaces 2>&1 | tee ./javaff_interfaces.build.log # depends on plansys2_msgs ros2_bdi_interfaces
# ros2_bdi_core and ros2_bdi_on_litter_world depend on javaff_interfaces

# ---- from src/ROS2-BDI ----------------------
source ./install/setup.bash
colcon build --packages-select ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core --packages-skip ros2_bdi_interfaces # ros2_bdi_core depends on javaff_interfaces
source ./install/setup.bash
# colcon build --symlink-install --packages-select litter_world ros2_bdi_on_litter_world litter_world_interfaces webots_ros2_simulations_interfaces webots_ros2_simulations ros2_bdi_on_webots
colcon build --symlink-install --base-paths src/ROS2-BDI --packages-ignore ros2_bdi_tests --packages-skip ros2_bdi_interfaces ros2_bdi_utils ros2_bdi_skills ros2_bdi_bringup ros2_bdi_core
source ./install/setup.bash
colcon build --symlink-install --packages-select ros2_bdi_tests
