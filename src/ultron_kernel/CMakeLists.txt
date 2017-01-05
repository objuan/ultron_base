cmake_minimum_required(VERSION 2.8.3)
project(ultron_kernel)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface 
  diagnostic_updater roslaunch roslint roscpp sensor_msgs std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS chrono)


# aggiunti

add_message_files(
  FILES
  RobotInfo.msg
  RobotStatus.msg
  RobotOdom.msg
)

add_service_files(
  FILES
  GetRobotInfo.srv
  RobotCommand.srv
)
# aggiunti

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES horizon_legacy
  CATKIN_DEPENDS diagnostic_updater hardware_interface  roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(horizon_legacy
  src/horizon_legacy/crc.cpp
  src/horizon_legacy/Logger.cpp
  src/horizon_legacy/Message.cpp
  src/horizon_legacy/Message_data.cpp
  src/horizon_legacy/Message_request.cpp
  src/horizon_legacy/Message_cmd.cpp
  src/horizon_legacy/Transport.cpp
  src/horizon_legacy/Number.cpp
  src/horizon_legacy/linux_serial.cpp
)

add_executable(ultron_node
  src/ultron_kernel.cpp
  src/ultron_hardware.cpp
  src/ultron_diagnostics.cpp
  src/horizon_legacy_wrapper.cpp)
target_link_libraries(ultron_node horizon_legacy ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies( ultron_node  ultron_node_msgs_generate_messages_cpp)

roslint_cpp(
  src/ultron_kernel.cpp
  src/ultron_hardware.cpp
  src/ultron_diagnostics.cpp
  include/ultron_kernel/ultron_diagnostics.h
  include/ultron_kernel/ultron_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS horizon_legacy ultron_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
