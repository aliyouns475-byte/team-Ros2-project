cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "action/GoToPose.action"
  "action/DrawSquare.action"
  "srv/SetSpeedLimits.srv"
  DEPENDENCIES action_msgs
)
ament_package()
