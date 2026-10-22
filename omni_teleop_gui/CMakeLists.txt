cmake_minimum_required(VERSION 3.16)
project(omni_teleop_gui)

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTORCC ON)
# moc explicito: AUTOMOC no detectaba el header .hh
qt5_wrap_cpp(OMNI_TELEOP_MOC src/OmniTeleop.hh)

# El nombre de la libreria (libOmniTeleop.so) es el "filename" del plugin en gui.config
add_library(OmniTeleop SHARED
  src/OmniTeleop.cc
  src/OmniTeleop.qrc
  ${OMNI_TELEOP_MOC}
)
target_link_libraries(OmniTeleop
  gz-gui8::gz-gui8
  gz-msgs10::gz-msgs10
  gz-plugin2::register
  gz-transport13::gz-transport13
  Qt5::Core
  Qt5::Quick
)

install(TARGETS OmniTeleop LIBRARY DESTINATION lib/${PROJECT_NAME})

# Al hacer source del workspace, Gazebo encuentra el plugin
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
