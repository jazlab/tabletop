cmake_minimum_required(VERSION 3.8)
project(tabletop_unbag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fastcdr REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(unbag src/main.cpp src/unbagger.cpp src/handlers/csv_handler.cpp
                     src/handlers/image_handler.cpp)

target_include_directories(
  unbag PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(
  unbag
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
  rcpputils
  rcutils
  sensor_msgs
  cv_bridge)

target_link_libraries(unbag fastcdr ${OpenCV_LIBS} Threads::Threads)

install(TARGETS unbag DESTINATION lib/${PROJECT_NAME})

# Diagnostic tool: isolates the storage-read cost of unbagging (rosbag2 indexed
# log-time reader vs. an mmap'd file-order mcap scan). See docs/performance.md.
add_executable(bag_read_bench tools/bag_read_bench.cpp)
target_include_directories(
  bag_read_bench PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(bag_read_bench rosbag2_cpp rosbag2_storage
                          mcap_vendor)
target_link_libraries(bag_read_bench Threads::Threads)
install(TARGETS bag_read_bench DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # This repo formats C++ with clang-format and CMake with cmake-format /
  # cmake-lint (see .pre-commit-config.yaml). Those conflict with the ament
  # style linters (uncrustify/cpplint use K&R + 80 cols), so skip the style
  # linters here and keep the non-style checks (copyright, cppcheck, xmllint).
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
