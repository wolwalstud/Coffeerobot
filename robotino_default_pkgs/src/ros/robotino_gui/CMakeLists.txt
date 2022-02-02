##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robotino_gui)

set(CMAKE_AUTOUIC ON)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp
             rviz
) 

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

##############################################################################
# Sections
##############################################################################


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)  
set(QRC_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/robotino_default_pkgs/src/robotino_gui/resource.qrc)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport)
find_package(Qt5 COMPONENTS Core Gui Widgets)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})##############################################################################
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource.qrc)
#qt5_add_resources(RESOURCES_RCC ${RESOURCE})

# Sources
##############################################################################
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

set(QT_SOURCES
#Main Window
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
#Calibration Window
    src/Manual_Window/manual_window.cpp
    src/Manual_Window/manual_window.h
    src/Manual_Window/manual_window.ui
#Position Window
    #src/Position_Window/position_window.cpp
    #src/Position_Window/position_window.h
    #src/Position_Window/position_window.ui
#rviz Window
    #src/rviz_window/rviz_window.cpp
    #src/rviz_window/rviz_window.h
    #src/rviz_window/rviz_window.ui
#Teleop Window
    src/Teleop_Window/teleop_window.cpp
    src/Teleop_Window/teleop_window.h
    src/Teleop_Window/teleop_window.ui
#Help Window
    src/Help_Window/help_window.cpp
    src/Help_Window/help_window.h
    src/Help_Window/help_window.ui
#sidebar Window
    src/sidebar.cpp
    src/sidebar.h
 #ROS SRC
    src/ROS_src/_ros.h
    src/ROS_src/_ros.cpp
#Resources
    resource.qrc
    )

##############################################################################
# Binaries
##############################################################################
add_executable(robotino_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${SRCS} ${RESOURCE})
target_link_libraries(robotino_gui ${QT_LIBRARIES} ${catkin_LIBRARIES}
        Qt5::Core     Qt5::Widgets)
install(TARGETS robotino_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

