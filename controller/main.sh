#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

python main.py #__ns:=hil
