#!/bin/bash

cd ${0%/*}

source /opt/ros/kinetic/setup.bash || exit 1

#rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.055 image:=/camera/image_raw camera:=/camera
rosrun camera_calibration_parsers convert calibrate.ini calibrate.yaml
