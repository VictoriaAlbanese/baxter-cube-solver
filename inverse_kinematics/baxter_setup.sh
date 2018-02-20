#!/bin/bash
rosrun baxter_tools enable_robot.py -e
echo "Enabled robot."

rosrun baxter_tools camera_control.py -c left_hand_camera
echo "Closed left cam."

rosrun baxter_tools camera_control.py -c right_hand_camera
echo "Closed right cam."

rosrun baxter_tools camera_control.py -c head_camera
echo "Closed head cam."

rosrun baxter_tools camera_control.py -o right_hand_camera -r 640x400 
echo "Opened right cam."

rosrun baxter_tools camera_control.py -c left_hand_camera
echo "Closed left cam again."

rosrun baxter_tools camera_control.py -o head_camera -r 960x600
echo "Opened head cam."
