#!/bin/bash

echo "Petrorov Arduino Starter .. V1.0 "

#if [[ $? -ne 1 ]]; then
#    echo " Usage "
#    exit 1
#fi

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
