##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(picodex_rviz_plugins)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
# find_package(catkin REQUIRED COMPONENTS rviz  qt_build roscpp)

find_package(catkin REQUIRED COMPONENTS rviz qt_gui_cpp qt_build roscpp)

include_directories(${catkin_INCLUDE_DIRS} /usr/include/OGRE)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
#rosbuild_prepare_qt5(Core Widgets) # Add the appropriate components to the component list here
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/picodex/*.hpp  src/*.h)

if(rviz_QT_VERSION VERSION_LESS "5")
    QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
    QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
    QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
else()
    QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
    QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
    QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
endif()

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

#add_executable(picodex ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(picodex_rviz_plugins ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

target_link_libraries(picodex_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} )

#install(TARGETS picodex RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS picodex_rviz_plugins LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

