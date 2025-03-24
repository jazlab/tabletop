cmake_minimum_required(VERSION 3.5)
project(tabletop_description)

find_package(ament_cmake REQUIRED)

# TODO: maybe add meshes/objects if necessary
install(
  DIRECTORY launch urdf rviz meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
