#!/bin/bash 

roslaunch android_cam-imu.launch &
sleep 1
echo "launch Android_Camera-IMU successfully"

roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64" frame_id:="Pandar64" &
sleep 1
echo "hesai_lidar successfully"

roslaunch darknet_ros yolo_v3.launch &
echo "launch darknet successfully"

wait
exit 0

