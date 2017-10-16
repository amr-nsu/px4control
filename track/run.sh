#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

v4l2-ctl --set-ctrl=power_line_frequency=1

roslaunch run.launch
