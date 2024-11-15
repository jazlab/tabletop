cmake_minimum_required(VERSION 3.8)
project(tabletop_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit REQUIRED)

include_directories(include)

add_library(srvcli SHARED src/moveit_pose_server.cpp src/moveit_pose_client.cpp)
ament_target_dependencies(srvcli rclcpp moveit tabletop_msgs)

add_executable(moveit_interface src/moveit_interface.cpp)
ament_target_dependencies(moveit_interface rclcpp moveit)

add_executable(moveit_cli_node src/moveit_client_node.cpp)
target_link_libraries(moveit_cli_node srvcli)
ament_target_dependencies(moveit_interface rclcpp moveit)

add_executable(moveit_srv_node src/moveit_server_node.cpp)
target_link_libraries(moveit_srv_node srvcli)
ament_target_dependencies(moveit_interface rclcpp moveit)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS srvcli LIBRARY DESTINATION lib)

install(TARGETS moveit_interface moveit_cli_node moveit_srv_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(srvcli)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
