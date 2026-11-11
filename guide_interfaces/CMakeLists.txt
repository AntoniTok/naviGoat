cmake_minimum_required(VERSION 3.8)
project(guide_interfaces)

# Tools we need: the base build system, standard messages, and the message code generator
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate Python and C++ code from our message definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RouteProgress.msg"
  "msg/NavGoalStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Make sure anything using our messages also gets the runtime support
ament_export_dependencies(rosidl_default_runtime)

ament_package()