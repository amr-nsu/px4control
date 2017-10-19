#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash

catkin init
catkin build
