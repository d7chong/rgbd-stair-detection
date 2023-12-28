[![C++](https://img.shields.io/badge/C%2B%2B-blue.svg)](https://devdocs.io/cpp/) [![CMake](https://img.shields.io/badge/CMake-green.svg)](https://cmake.org/) [![ROS](https://img.shields.io/badge/ROS-ff6f61.svg)](http://www.ros.org/) [![PCD](https://img.shields.io/badge/Point%20Cloud%20Data-blue.svg)](https://pointclouds.org/) [![Ubuntu 18.04](https://img.shields.io/badge/Ubuntu%2018.04-orange.svg)](https://releases.ubuntu.com/18.04/)

# stair-detection

<p align="center">
    <img src="imgs/stairs-ascending-final-demo.gif" alt="Ascending stairs demo" width="600">
    <img src="imgs/stairs-descending-final-demo.gif" alt="Descending stairs demo" width="600">
</p>

This codebase uses a RGBD camera's ROS pointcloud topic to detect stairs. 

The algorithm can: 
- classify stairs as either ascending or descending
- identify the number of steps
- identify the dimensions of the stairs (length, width, height)
- predict the distance from the camera to the staircase
- create a segmentation mask over the detected stairs


## Improvements

This implementation was made for RGBD cameras with ROS pointcloud topics. Any RGBD camera with a ROS pointcloud topic should be compatible with this implementation.

Various changes were made to improve this codebase:

- Implemented better segmentation masks for descending stairs
- Tweaked parameters for better stair detection
- Implemented more memory-efficient code
- Removed unnecessary functions or code

## Installation

#### Requirements

This implementation was successfully run on Ubuntu 18.04.6 LTS. You may have trouble running this on higher versions, so Ubuntu 18.04 Bionic is recommended.

Before you make a workspace and clone this repository, you need to have the following packages installed:
- [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) (ROS1)
- Your SDK for your sensor. I used a Intel Realsense camera, so I installed the following:
    - [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) for ROS1
    - [librealsense](https://github.com/IntelRealSense/librealsense)

#### Making your workspace

```bash
mkdir -p ~/stairs_ws/src
cd ~/stairs_ws/src
git clone --depth 1 --single-branch https://github.com/d7chong/stair-detection.git .
cd ~/stairs_ws/
MAKEFLAGS='-j4 -l4' catkin_make

echo "source ~/stairs_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Tips

#### 1. Rebuilding with ```catkin_make``` after changes are made

Whenever changes are made to the codebase, make sure to rebuild the program:

```bash
MAKEFLAGS='-j4 -l4' catkin_make
```

#### 2. Changing the ROS pointcloud topic

Even if you are using a RGBD camera with a ROS pointcloud topic, the topic name will probably be different from that in this implementation. 

In the ```main_stairs.cpp``` file, there is a function ```void startMainLoop``` in ```class mainLoop``` In this function, you can see the following lines:

```C++
ros::init(argc, argv, "d435i_pcd_node");
ros::NodeHandle nh;

ros::Subscriber cloud_sub = nh.subscribe("/camera/depth/color/points", 1, &mainLoop::cloudCallback, this);
```

The name in ros::init in the first line isn't significant - you can change it to whatever you like. However, it is **essential** that you change the string in the third line to **your own ROS pointcloud topic**! After launching your sensor's ROS node, look for your pointcloud topic by running

```bash
rostopic list
```

To double check if your the topic you found is the pointcloud topic, try tunning it in [rviz](https://wiki.ros.org/rviz).

#### 3. Tweaking parameters

There are various parameters to tweak in this implementation. If you are not interested in improving upon this code, and you simply want to run the code, you should focus on the following parameters:

1. **Voxel size**

    The algorithm for this codebase divides the RGBD image into voxels, and you can set the voxel size in advance. Smaller voxels result in increased quality and accuracy, but decreased efficiency. I usually used voxel sizes of 0.5cm~2cm (0.005f ~ 0.02f), but you should experiment with various voxel sizes. 

2. **Stair size parameters**

    Unfortunately, this repo is **not** robust for **all** stair sizes. Therefore, there are constraints for stair dimensions, to filter out noisy results. You can set minimum and maximum values for the stair length, width, and height.

## Running the program

#### [Terminal 1] 

After connecting your RGBD camera to your computer, launch the ROS pointcloud node (this is for a Intel Realsense D435i depth camera):

```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```

#### [Terminal 2]
In a new terminal window, launch the executable to detect stairs:

```bash
rosrun stairs_detection stairs
``` 

## Reference

This is a tweaked implementation of [Stairs Detection with Odometry-Aided Traversal from a Wearable RGB-D Camera (Perez-Yus et al., 2017)](https://www.sciencedirect.com/science/article/pii/S1077314216300315), and has been adopted from the [original implementation](https://github.com/aperezyus/stairs_detection).