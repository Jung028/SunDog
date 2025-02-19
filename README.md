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


