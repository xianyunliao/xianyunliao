cmake_minimum_required(VERSION 3.8)

project(bot2_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    bot2_nav2/multi_goal_nav.py
  RENAME multi_goal_nav
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
