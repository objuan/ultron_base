#!/bin/bash

echo "remove"
rm -r ~/petrorov/ultron/arduino/libraries/ros_lib/
rm -r ~/Arduino/libraries/ros_lib/

echo "build"
rosrun rosserial_client make_libraries ~/petrorov/ultron/arduino/libraries
rosrun rosserial_client make_libraries ~/Arduino/libraries

echo "copy"
cp /home/marco/catkin_ws/src/rosserial/rosserial_arduino/src/ros_lib/ros.h  ~/petrorov/ultron/arduino/libraries/ros_lib

cp /home/marco/catkin_ws/src/rosserial/rosserial_arduino/src/ros_lib/ArduinoHardware.h  ~/petrorov/ultron/arduino/libraries/ros_lib

cp /home/marco/catkin_ws/src/rosserial/rosserial_arduino/src/ros_lib/ros.h  ~/Arduino/libraries/ros_lib

cp /home/marco/catkin_ws/src/rosserial/rosserial_arduino/src/ros_lib/ArduinoHardware.h  ~/Arduino/libraries/ros_lib
