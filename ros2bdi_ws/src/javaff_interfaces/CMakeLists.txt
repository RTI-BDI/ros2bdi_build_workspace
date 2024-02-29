cmake_minimum_required(VERSION 3.5)
project(javaff_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcljava_common REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

rosidl_generator_java_get_typesupports(_java_type_supports)

rosidl_generate_interfaces( ${PROJECT_NAME}
  
  "msg/PartialPlan.msg"
  "msg/SearchResult.msg"
  "msg/ActionExecutionStatus.msg"
  "msg/ActionCommittedStatus.msg"
  "msg/ExecutionStatus.msg"
  "msg/CommittedStatus.msg"

  "srv/UnexpectedState.srv"
  "srv/JavaFFPlan.srv"
  DEPENDENCIES
  builtin_interfaces
  rcl_interfaces
  ${_java_type_supports}
  plansys2_msgs
  ros2_bdi_interfaces
  #SKIP_INSTALL
)


ament_package()
