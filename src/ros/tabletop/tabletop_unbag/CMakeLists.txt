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
find_package(pluginlib REQUIRED)
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

# Use the serial HDF5: both the serial and OpenMPI dev packages may be present,
# and the converter is single-process -- it must not pull in MPI.
set(HDF5_PREFER_PARALLEL OFF)
find_package(HDF5 REQUIRED COMPONENTS C)

# Shared core: the format-agnostic flattening + image decode, the HDF5 writer,
# and the HDF5 handlers. Built once as a PIC static library so it can be linked
# both into the `unbag` CLI and into the rosbag2 storage plugin (.so) without
# compiling the sources twice. The CSV/JPEG file handlers stay in the CLI only.
set(unbag_core_deps rosidl_typesupport_introspection_cpp rosidl_runtime_cpp
                    rcpputils rcutils sensor_msgs cv_bridge)

add_library(
  unbag_core STATIC src/flatten.cpp src/image_decode.cpp src/hdf5_writer.cpp
                    src/handlers/hdf5_handlers.cpp)
set_target_properties(unbag_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  unbag_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    ${OpenCV_INCLUDE_DIRS} ${HDF5_C_INCLUDE_DIRS})
ament_target_dependencies(unbag_core ${unbag_core_deps})
target_link_libraries(unbag_core fastcdr ${OpenCV_LIBS} ${HDF5_C_LIBRARIES}
                      Threads::Threads)

# The CLI converter.
add_executable(unbag src/main.cpp src/unbagger.cpp src/handlers/csv_handler.cpp
                     src/handlers/image_handler.cpp)
target_include_directories(
  unbag PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                ${OpenCV_INCLUDE_DIRS} ${HDF5_C_INCLUDE_DIRS})
ament_target_dependencies(unbag rclcpp rosbag2_cpp rosbag2_storage
                          ${unbag_core_deps})
target_link_libraries(unbag unbag_core fastcdr ${OpenCV_LIBS}
                      ${HDF5_C_LIBRARIES} Threads::Threads)

install(TARGETS unbag DESTINATION lib/${PROJECT_NAME})

# rosbag2 storage plugin: `ros2 bag record -s hdf5` (or `ros2 bag convert`)
# writes the same analysis-ready HDF5 layout as `unbag --format hdf5`, by
# flattening/decoding each message at write time. It is write-only (the
# analysis-ready layout is not round-trippable, so playback is unsupported).
add_library(rosbag2_storage_hdf5 SHARED src/storage/hdf5_storage.cpp)
target_include_directories(
  rosbag2_storage_hdf5
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          ${OpenCV_INCLUDE_DIRS} ${HDF5_C_INCLUDE_DIRS})
ament_target_dependencies(rosbag2_storage_hdf5 rclcpp rosbag2_storage pluginlib
                          ${unbag_core_deps})
target_link_libraries(rosbag2_storage_hdf5 unbag_core fastcdr ${OpenCV_LIBS}
                      ${HDF5_C_LIBRARIES} Threads::Threads)
pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  TARGETS rosbag2_storage_hdf5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
