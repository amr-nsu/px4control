#!/bin/bash

cd ${0%/*}

source ./devel/setup.bash || exit 1

roslaunch simulator.launch
