# PointCloudApps
This project hosts code that implements local shape descriptor of point cloud.  
>standalone: Basic implementation of descriptor. It visualizes descriptors for each pixel.
>experiment_pcl: Add on for comparison with descriptors from PCL. It records my descriptor with PCL descriptors.
>matlab: matlab codes to evaluate performances. It reads data recorded by 'experiment_pcl'

## Prequistes
This project has been developed in the QtCreator environment. To run this code, you need
'''
- Qt5.x and QtCreator
- OpenCV 3.1.
- OpenCL 1.2
- Eigen 3
- PCL 1.8 (only for experiment_pcl)
'''
and all the prequisites for them. If you are on Ubuntu 14.04 64bit, you do NOT have to build the dependencies by yourself.  
You may download the pre-built binaries from [Here](https://drive.google.com/open?id=0B3n44jck6ZLuaUhDSkh1V1pUUjg) It includes boost-1.61, eigen-3.2.8, flann-1.8.4, opencv-3.1.0, openni2, and pcl-1.8.  
I tried to build the prequisites for PCL by myself but the other prequisites (mainly for opencv) are just installed by 'apt-get install' as follows.
'''
sudo add-apt-repository ppa:george-edison55/cmake-3.x
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install cmake
sudo apt-get install build-essential cmake-gui git
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libgtk-3-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libv4l-dev
sudo apt-get install libgl1-mesa-dev qtcreator
sudo apt-get install g++ python libusb-1.0-0-dev libudev-dev openjdk-6-jdk freeglut3-dev doxygen graphviz
'''
Once you download the pre-built binary and unzip it, you need to edit the library path in standalone/PCApps-share.pri
'''
LIBRARY_ROOT=/path/to/unzipped/library
'''
Then you can build the project.

