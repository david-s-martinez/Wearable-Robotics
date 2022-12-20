cmake_minimum_required(VERSION 3.0.2)
project(skin_control)
SET(TUM_ICS_USE_QT5 TRUE)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tum_ics_params
  tum_ics_tfs
  tum_ics_skin_msgs
  tum_ics_skin_bridge
  tum_ics_skin_descr
  std_msgs
  message_generation
)

set(DEPENDS_VAR system_lib libqt5-dev)
if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    set(DEPENDS_VAR )
endif()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  tum_ics_params
  tum_ics_tfs
  tum_ics_skin_msgs
  tum_ics_skin_bridge
  tum_ics_skin_descr
  std_msgs
  message_runtime
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

if(NOT TUM_ICS_USE_QT5)

message("${BGREEN}+++${NOCOLOR} Compile with ${BGREEN}Qt4${NOCOLOR}")

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)

 

# resolve qt dependencies
# add Qt Modules
set(QT_USE_QTNETWORK TRUE)

include(${QT_USE_FILE})

# add_definitions(${QT_DEFINITIONS})

endif()

if(TUM_ICS_USE_QT5)

message("${BGREEN}+++${NOCOLOR} Compile with ${BYELLOW}Qt5${NOCOLOR}")

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

endif()



if(NOT TUM_ICS_USE_QT5)

# feed moc

QT4_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

# feed user interface compiler

QT4_WRAP_UI(FORMS_UI ${FORMS})

# feed qt resource compiler

QT4_ADD_RESOURCES(RESOURCES_RC ${RESOURCES})

endif()

if(TUM_ICS_USE_QT5)

# feed moc

QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

# feed user interface compiler

QT5_WRAP_UI(FORMS_UI ${FORMS})

endif()




# DON'T USE (enforces Q_SIGNAL etc. macros)
# use here because of boost problems in tf etc.
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)


# needed for Qt generated files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

set(APP_TARGETS
    skin_control
)

foreach(target ${APP_TARGETS})
    add_executable(${target} src/Applications/main_${target}.cpp)
    add_dependencies(${target} ${tum_ics_skin_msgs_EXPORTED_TARGETS})

    target_link_libraries(${target}
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${SKINCORE_LIBRARIES}
        ${TCOMMON_LIBRARIES}
        ${QT_LIBRARIES}
        Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	Qt5::OpenGL
    )
endforeach()

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the
#   debtools. This function is controlled with the global variable -DDEB_BUILD
#   (true/false)
include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
ics_ros_debgen()

install(TARGETS ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
