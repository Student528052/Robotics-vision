# Robotics-vision
Project Robotics (Sorting metal) repository. 
## Downloading
## Building/Installing. 

# adding a node to this code: 
This is assuming you work inside a ROS2 devcontainer. <br>
#### First, you need to clone the repo

```sh
$ git clone https://github.com/Student528052/Robotics-vision.git
```
After whic, you need to run the `install.sh` script, which creates the node inside ROS2. 

' # NOTE: if the script has failed ( or is not there likely), you can just follow these commands:<br>
```sh
colcon pkg create camera_node --build-type ament_python --dependencies rclpy

#creating all nessesary files
cp Robotics-vision/Vision_src/test_script.py ./camera_rv_node/camera_rv_node/camera.py
chmod +x ./camera_rv_node/camera_rv_node/camera.py
cp Robotics-vision/Vision_src/test_image.png ./camera_rv_node/camera_rv_node/test_image.png
rm camera_rv_node/setup.py
cp Robotics-vision/Vision_src/setup.py ./camera_rv_node/setup.py
```
For building the node
```sh
echo "source /home/ros/ros2_ws/install/setup.bash" >> ./.bashrc
source .bashrc
colcon build 
```

#### Running the node in ROS
Once you have built it, ROS should be able to detect and excecute the node by running: 
```sh
$ ros2 run camera_rv_node camera
```