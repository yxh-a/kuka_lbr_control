cmake_minimum_required(VERSION 3.8)
project(kuka_control_tutorial)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generates kuka_control_tutorial.action.MoveToPose for both C++ and Python.
# This already installs a Python package named `kuka_control_tutorial`, so a
# second ament_python_install_package() for the same name would clash on its
# CMake targets.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/FollowPath.action"
  DEPENDENCIES action_msgs geometry_msgs
)

# Drop the hand-written modules into that same generated Python package, so a
# node imports the action and the helpers from one namespace.
install(FILES
  ${PROJECT_NAME}/cartesian_move_server.py
  ${PROJECT_NAME}/draw_trajectory_gui.py
  ${PROJECT_NAME}/motion_client.py
  ${PROJECT_NAME}/move_to_pose_client.py
  ${PROJECT_NAME}/path_planning.py
  ${PROJECT_NAME}/stiffness_client.py
  ${PROJECT_NAME}/trajectory.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cartesian_move_server
  scripts/draw_trajectory_gui
  scripts/move_to_pose_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_trajectory test/test_trajectory.py)
  ament_add_pytest_test(test_path_planning test/test_path_planning.py)
  ament_add_pytest_test(test_doc_links test/test_doc_links.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
