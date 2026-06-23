cmake_minimum_required(VERSION 3.22)
project(tabletop_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf meshes config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
