#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

python vision_pose.py