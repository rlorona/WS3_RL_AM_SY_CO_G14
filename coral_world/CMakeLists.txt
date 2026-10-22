cmake_minimum_required(VERSION 3.8)
project(coral_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds scripts launch rviz config textures DESTINATION share/${PROJECT_NAME})
# clearpath_gz busca "<world>.sdf" directamente en cada <prefijo>/share
install(FILES worlds/coral.sdf DESTINATION share)

ament_package()
