cmake_minimum_required(VERSION 3.8)
project(ps11_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in"
)

ament_package()
