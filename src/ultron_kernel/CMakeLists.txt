cmake_minimum_required(VERSION 2.8.3)
project(ultron_kernel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface 
  diagnostic_updater roslaunch roslint roscpp sensor_msgs std_msgs message_generation tf serial)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_python_setup()

# aggiunti

add_message_files(
  FILES
  RobotInfo.msg
  RobotStatus.msg
  RobotOdom.msg
  RobotIMU.msg
  RobotSpeed.msg	
)

add_service_files(
  FILES
  GetRobotInfo.srv
  RobotCommand.srv
  GetDataEncoders.srv
)
# aggiunti

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater hardware_interface  roscpp sensor_msgs serial
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_executable(ultron_node
  src/ultron_kernel.cpp
  src/ultron_hardware.cpp
  src/ultron_diagnostics.cpp)
add_dependencies( ultron_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultron_node  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies( ultron_node  ultron_node_msgs_generate_messages_cpp)

roslint_cpp(
  src/ultron_kernel.cpp
  src/ultron_hardware.cpp
  src/ultron_diagnostics.cpp
  include/ultron_kernel/ultron_diagnostics.h
  include/ultron_kernel/ultron_hardware.h
)

add_executable(mpu6050_serial_to_imu_node 
  src/mpu6050_serial_to_imu_node.cpp
  src/RemoteController.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mpu6050_serial_to_imu_node
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)



install(PROGRAMS scripts/serial_node.py scripts/MySerialClient.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ultron_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
