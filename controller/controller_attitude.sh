#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

python controller_attitude.py #__ns:=hil
