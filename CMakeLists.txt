cmake_minimum_required(VERSION 2.8.3)
project(rqt_marine_sonar)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    rqt_gui
    rqt_gui_cpp
    marine_acoustic_msgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)

roslint_cpp()

set(rqt_marine_sonar_SRCS
src/echogram_widget.cpp
src/marine_echogram_plugin.cpp
src/ping.cpp
)

set(rqt_marine_sonar_HDRS
  include/rqt_marine_sonar/marine_echogram_plugin.h
  include/rqt_marine_sonar/echogram_widget.h
  include/rqt_marine_sonar/ping.h
)

set(rqt_marine_sonar_UIS
  src/marine_echogram_plugin.ui
)

set(rqt_marine_sonar_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rqt_marine_sonar_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp rqt_gui marine_acoustic_msgs
#  DEPENDS system_lib
)

qt5_wrap_cpp(rqt_marine_sonar_MOCS ${rqt_marine_sonar_HDRS})

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(rqt_marine_sonar_UIS_H ${rqt_marine_sonar_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  ${rqt_marine_sonar_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${rqt_marine_sonar_SRCS}
  ${rqt_marine_sonar_MOCS}
  ${rqt_marine_sonar_UIS_H}
)

qt5_use_modules(${PROJECT_NAME} Widgets)

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES} Qt5::Charts
    ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})


install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
