cmake_minimum_required(VERSION 3.8)
project(custom_gripper_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
