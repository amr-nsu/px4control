#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

roslaunch run.launch & P1=$!
python main.py & P2=$!
wait $P1 $P2
