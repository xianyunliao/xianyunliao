cmake_minimum_required(VERSION 3.8)

project(bot2_rgbd_camera)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
