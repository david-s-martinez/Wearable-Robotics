cmake_minimum_required(VERSION 3.0.2)
project(exo_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(TUM_ICS_USE_QT5 TRUE)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tum_ics_params
  tum_ics_tfs
  tum_ics_skin_msgs
  tum_ics_skin_bridge
  tum_ics_skin_descr
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(DEPENDS_VAR system_lib libqt5-dev)
if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    set(DEPENDS_VAR )
endif()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        tum_ics_params
        std_msgs
        message_runtime
        tf2
        tf2_ros
        tum_ics_tfs
        tum_ics_skin_msgs
        tum_ics_skin_bridge
        tum_ics_skin_descr
    DEPENDS ${DEPENDS_VAR}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
message(STATUS "EIGEN3_INCLUDE_DIR " ${EIGEN3_INCLUDE_DIR})

find_package(TCommon REQUIRED)
add_definitions(${TCOMMON_DEFINITIONS})
include_directories(${TCOMMON_INCLUDE_DIRS})
message(STATUS "TCOMMON_INCLUDE_DIRS   " ${TCOMMON_INCLUDE_DIRS})
#message(STATUS "TCOMMON_LIBRARIES      " ${TCOMMON_LIBRARIES})

find_package(SkinCore REQUIRED)
include_directories(${SKINCORE_INCLUDE_DIRS})
add_definitions(${SKINCORE_DEFINITIONS})
message(STATUS "SKINCORE_INCLUDE_DIRS   " ${SKINCORE_INCLUDE_DIRS})
#message(STATUS "SKINCORE_LIBRARIES      " ${SKINCORE_LIBRARIES})

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

# feed moc

QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

# feed user interface compiler

QT5_WRAP_UI(FORMS_UI ${FORMS})

# add Qt Modules
#set(QT_USE_QTNETWORK TRUE)

#include(${QT_USE_FILE})

# DON'T USE (enforces Q_SIGNAL etc. macros)
# use here because of boost problems in tf etc.
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# needed for Qt generated files
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

file(GLOB_RECURSE MY_CTRL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
        include/*.h)

add_library(ExoControllers src/exo_pos_control.cpp
    ${MY_CTRL_INC}
)

target_link_libraries(ExoControllers
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${TCOMMON_LIBRARIES}
    ${QT_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
)

add_dependencies(ExoControllers
  ${sensor_msgs_EXPORTED_TARGETS}
  ${std_msgs_EXPORTED_TARGETS}
  ${visualization_msgs_EXPORTED_TARGETS}
)

set(APP_TARGETS
  exo_control
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/main_${target}.cpp)

  target_link_libraries(${target}
      ${catkin_LIBRARIES}
      ${roscpp_LIBRARIES}
      ${TCOMMON_LIBRARIES}
      ${QT_LIBRARIES}
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets
      Qt5::OpenGL
      ExoControllers
  )
endforeach()
     

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
