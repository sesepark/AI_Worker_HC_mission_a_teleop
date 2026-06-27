cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PartDetection.msg"
  "msg/PartDetectionArray.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(perception_nodes)

install(PROGRAMS
  perception_nodes/part_detector/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detector_node
)
install(PROGRAMS
  perception_nodes/monitor_ocr/monitor_ocr_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME monitor_ocr_node
)
install(PROGRAMS
  perception_nodes/wrist_projection/wrist_task_grasp_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wrist_task_grasp_planner_node
)
install(PROGRAMS
  perception_nodes/management/tray_manage_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tray_manage_node
)
install(PROGRAMS
  perception_nodes/monitor_ocr/viewer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME monitor_ocr_viewer
)
install(PROGRAMS
  perception_nodes/head_pipe/head_pipe_top_centers_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME head_pipe_top_centers_node
)
install(PROGRAMS
  perception_nodes/head_pipe/pipe_centers_preset_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pipe_centers_preset_pub
)
install(PROGRAMS
  perception_nodes/part_detector/peg_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME peg_detector_node
)
install(PROGRAMS
  perception_nodes/part_detector/nut_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nut_detector_node
)
install(PROGRAMS
  perception_nodes/place_validity/place_pose_valid_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME place_pose_valid_node
)

install(DIRECTORY
  launch
  config
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
