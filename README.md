# Final Year Project 

Link to full documentation : https://drive.google.com/file/d/1m5zop82CfQDfNUxipqevMrXIc9uzrJJz/view?usp=sharing 

![image](https://github.com/user-attachments/assets/5361df34-0e91-4a38-ad99-cf2e0dbd2648)

Installation : 
This was done using Ubuntu 20.04 noetic, on Jetson Nano Orin in HUMAC Lab Sunway Uni.

1. Follow https://github.com/orbbec/ros_astra_camera, but mkdir the catkin_make first, then only install the other dependencies, within this directory.
2. Can copy paste this :

```
sudo apt install libgflags-dev  ros-noetic-image-geometry ros-noetic-camera-info-manager ros-noetic-image-transport ros-noetic-image-publisher libusb-1.0-0-dev libeigen3-dev ros-noetic-backward-ros libdw-dev
```

4. Plug in astra depth camera, then run the start camera
*Follow installation steps for astra camera if havent yet : https://github.com/orbbec/ros_astra_camera

```
source ./devel/setup.bash 
roslaunch astra_camera astra.launch
rviz
```

Manual Control :
To open the launch files for controlling the quadruped using the teleop node, the steps are as
follows:
```
roslaunch champ_config bringup.launch rviz:=true
roslaunch champ_teleop teleop.launch
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
rostopic echo and joint_trajectories
```
For slam / navigation:
```
roslaunch sundog_config bringup.launch
roslaunch rplidar_ros rplidar_a2m8.launch
```
a) Steps for running lidar mapping :
```
roslaunch hector_mapping mapping_default.launch
roslaunch sundog_config slam.launch
roscd champ_navigation/rviz
rviz -d navigate.rviz
```
b) Steps for running autonomous navigation :
```
roslaunch sundog_config navigate.launch
roscd champ_navigation/rviz
rviz -d navigate.rviz
```

Then to save the map, run [16] :
```
roscd sundog_config/maps
rosrun map_server map_saver
```
