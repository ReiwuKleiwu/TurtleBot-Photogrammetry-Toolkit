cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0100 NEW)
project(particlefilter_gz_gui_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick Widgets)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_GUI REQUIRED gz-gui8)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
find_library(GZ_MINIMAL_SCENE_LIB
  NAMES MinimalScene libMinimalScene.so
  PATHS /opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins
  NO_DEFAULT_PATH
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(WasdCameraController SHARED
  include/particlefilter_gz_gui_plugins/WasdCameraController.hh
  src/WasdCameraController.cc
  gui/WasdCameraController.qrc
)

target_include_directories(WasdCameraController PRIVATE
  include
  ${GZ_GUI_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_PLUGIN_INCLUDE_DIRS}
)

target_compile_options(WasdCameraController PRIVATE
  ${GZ_GUI_CFLAGS_OTHER}
  ${GZ_TRANSPORT_CFLAGS_OTHER}
  ${GZ_MSGS_CFLAGS_OTHER}
  ${GZ_PLUGIN_CFLAGS_OTHER}
)

target_link_directories(WasdCameraController PRIVATE
  ${GZ_GUI_LIBRARY_DIRS}
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
  ${GZ_PLUGIN_LIBRARY_DIRS}
)

target_link_libraries(WasdCameraController
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
  ${GZ_MINIMAL_SCENE_LIB}
  ${GZ_GUI_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
)

set_target_properties(WasdCameraController PROPERTIES
  BUILD_RPATH "/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins"
  INSTALL_RPATH "/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins"
)

install(TARGETS WasdCameraController
  LIBRARY DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY gui
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/environment/gz_gui_plugin_path.sh")

ament_package()
