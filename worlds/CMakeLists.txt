cmake_minimum_required(VERSION 3.8)
project(mundos_workshop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz config worlds models DESTINATION share/${PROJECT_NAME})

ament_package()
