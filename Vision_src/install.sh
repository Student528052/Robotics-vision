#!/bin/sh
# This is with the asumption that the working directory is /home/ros/ros2_ws/

# git clone https://Student528052/Robotics-vision.git
ros2 pkg create camera_rv_node --build-type ament_python --dependencies rclpy

#moving  all nessesary files
cp Robotics-vision/Vision_src/test_script.py ./camera_rv_node/camera_rv_node/camera.py
chmod +x ./camera_rv_node/camera_rv_node/camera.py
cp Robotics-vision/Vision_src/test_image.png ./camera_rv_node/camera_rv_node/test_image.png
rm camera_rv_node/setup.py
cp Robotics-vision/Vision_src/setup.py ./camera_rv_node/setup.py

#building the node
echo "source /home/ros/ros2_ws/install/setup.bash" >> ./.bashrc
source .bashrc
colcon build 