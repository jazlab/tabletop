cmake_minimum_required(VERSION 3.22)
project(tabletop_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/Eyelink.msg" "msg/TeensySensor.msg")

set(srv_files "srv/Ping.srv" "srv/SetArmLock.srv" "srv/SetReward.srv"
              "srv/SetSmartglass.srv")

set(action_files "action/EyelinkSmoothPursuit.action"
                 "action/FlicResponseTime.action")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  ADD_LINTER_TESTS)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
