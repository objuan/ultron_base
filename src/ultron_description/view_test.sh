#!/bin/bash

echo "View RVIZ Test .. V1.0 "

roslaunch ultron_description display.launch model:='$(find ultron_description)/urdf/test.urdf'
