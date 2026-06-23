cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/TaskItem.msg"
  "srv/GetTaskList.srv"
  "srv/GetTrayDetections.srv"
  "srv/MoveBaseLateral.srv"
  "action/MoveToScanPose.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
