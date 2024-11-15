cmake_minimum_required(VERSION 3.8)
project(tabletop_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit
  moveit_ros_planning_interface
  geometry_msgs
  tabletop_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(_tabletop_moveit_interface SHARED src/server.cpp)
ament_target_dependencies(_tabletop_moveit_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(server src/server_node.cpp)
target_link_libraries(server _tabletop_moveit_interface)
ament_target_dependencies(server ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS _tabletop_moveit_interface LIBRARY DESTINATION lib)

install(TARGETS server DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(_tabletop_moveit_interface)

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
