cmake_minimum_required(VERSION 3.8)
project(turtlebot4_gz_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gui launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
