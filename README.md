# ORB_SLAM2_CUDA
ORB-SLAM2 with GPU enhancement modified and added ROS topic publisher for **NVIDIA Jetson TX1**. 
Tested with Monocular camera in real time.

Based on ORB-SLAM2 with GPU enhancements by yunchih's [ORB-SLAM2-GPU2016-final](https://github.com/yunchih/ORB-SLAM2-GPU2016-final), which is based on Raul Mur-Artal's [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

- [Performance comparison between ORB-SLAM2 and ORB-SLAM2 with GPU enhancement running on TX1](https://www.youtube.com/watch?v=3597nnW2JCg&list=PLde9NsDtSVwZNb_pPyKm5eOPk86x_9Yk1&index=1)
- [Full playlist videos](https://www.youtube.com/playlist?list=PLde9NsDtSVwZNb_pPyKm5eOPk86x_9Yk1)
- [ORB-SLAM2 with GPU enhancements running on NVIDIA Jetson TX2 by Dan Pollock](https://www.youtube.com/watch?v=yBH9man45z0&feature=youtu.be)

I struggled with a number of issues to get the work of yunchih up and running on TX1, so hopefully this can help anyone doing the same thing. The original works offers ROS node to process live data, but it doesn't broadcast any message. So I added a ROS publisher for a few topics.

This is still a work in progress so expects things to change.

## Implementation
- [x] Monocular
- [ ] Stereo
- [ ] RGB-D

## Published topics
* tf
* pose
* pointcloud
* current frame

## Prerequisite
* I started with a fresh flash for TX1 with [JetPack 3.0](http://www.jetsonhacks.com/2017/03/21/jetpack-3-0-nvidia-jetson-tx2-development-kit/) and OpenCV4Tegra **not installed**.
* I recommend to run TX1 from a SD card, at least 64GB, because the build (especially OpenCV) consumes a lot of memory. You can follow JetsonHacks' post [here](http://www.jetsonhacks.com/2017/01/26/run-jetson-tx1-sd-card/).

## Installation
### Build GPU enabled OpenCV3 ROS Kinetic
I followed this [page](https://qiita.com/kendemu/items/a805b0b9828b6f6031db) to make the ROS replacement part. If you follow the commands on that page remember to do the patches afterwards. But if we use opencv 3.2 we don't need to do the patches, which is what I do below. 
First, check to get the CUDA compiler version:
```
nvcc --version 
```
If you get error ```nvcc: command not found ```, check [this page](https://devtalk.nvidia.com/default/topic/995277/cuda-8-0-toolkit-install-nvcc-not-found-ubuntu-16-04/) to solve it first before moving on.
Clone the OpenCV repo locally and checkout to version v3.2.0:
```
sudo apt-get install git
cd
git clone https://github.com/opencv/opencv.git opencv
cd opencv
git checkout -b v3.2.0 3.2.0
```
Then we install neccessary packages:
```
cd
sudo echo "deb-src http://packages.ros.org/ros/ubuntu xenial main" >> /etc/apt/sources.list.d/ros-latest.list
sudo apt-get update
sudo apt-get source ros-kinetic-opencv3
sudo apt-get install devscripts build-essential
cd ros-kinetic-opencv3-3.2.0
sudo apt-get build-dep ros-kinetic-opencv3-3.2.0
sudo dpkg-buildpackage -b -uc
cd ../
sudo mkdir /usr/src/deb
sudo cp ros-kinetic-opencv3_3.2.0-4xenial_arm64.deb /usr/src/deb/
cd /usr/src/deb/
sudo chmod a+wr /usr/src/deb
sudo apt-ftparchive packages . | gzip -c9 > Packages.gz
sudo apt-ftparchive sources . | gzip -c9 > Sources.gz
sudo chmod a+wr /etc/apt/sources.list.d/ros-latest.list
sudo echo "deb file:/usr/src/deb ./" >> /etc/apt/sources.list.d/ros-latest.list
sudo sed -i -e "1,2s/^/#/g" /etc/apt/sources.list.d/ros-latest.list
sudo apt-get update
sudo apt-get remove ros-kinetci-opencv3
sudo apt-get install ros-kinetic-opencv3
```
You can change ```ros-kinetic-desktop-full``` according to your need:
```
sudo apt-get install ros-kinetic-desktop-full
sudo sed -i -e "s/#//g" /etc/apt/sources.list.d/ros-latest.list
```
### Build OpenCV with CUDA for Tegra
This one is pretty straight forward, just follow the instructions in [this link](https://docs.opencv.org/3.2.0/d6/d15/tutorial_building_tegra_cuda.html) but change the version to 3.2.0. Below are all the commands I used, refer to the above link if you need clarification:
```
# The following command can be pasted into a shell in order to install the required packages:
sudo apt-get install libglew-dev libtiff5-dev zlib1g-dev libjpeg-dev libpng12-dev libjasper-dev libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libeigen3-dev libtbb-dev libgtk2.0-dev pkg-config
# Appropriate packages for Python2 and Python3
sudo apt-get install python-dev python-numpy python-py python-pytest
# Optionally:
sudo apt-get install python3-dev python3-numpy python3-py python3-pytest
# If you want to use OpenCV Extra:
cd
git clone https://github.com/opencv/opencv_extra.git
cd opencv_extra
git checkout -b v3.2.0 3.2.0
# Preparing the build area
cd
mkdir build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DBUILD_PNG=OFF \
    -DBUILD_TIFF=OFF \
    -DBUILD_TBB=OFF \
    -DBUILD_JPEG=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_ZLIB=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_python2=ON \
    -DBUILD_opencv_python3=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_OPENMP=OFF \
    -DWITH_FFMPEG=ON \
    -DWITH_GSTREAMER=OFF \
    -DWITH_GSTREAMER_0_10=OFF \
    -DWITH_CUDA=ON \
    -DWITH_GTK=ON \
    -DWITH_VTK=OFF \
    -DWITH_TBB=ON \
    -DWITH_1394=OFF \
    -DWITH_OPENEXR=OFF \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 \
    -DCUDA_ARCH_BIN=5.3 \
    -DCUDA_ARCH_PTX="" \
    -DINSTALL_C_EXAMPLES=ON \
    -DINSTALL_TESTS=OFF \
    -DOPENCV_TEST_DATA_PATH=../opencv_extra/testdata \
    ../opencv
```
Check the [link](https://docs.opencv.org/3.2.0/d6/d15/tutorial_building_tegra_cuda.html) to find the correct ```cmake``` command for your platform (there are 3 sets of them). In the case of **NVIDIA Jetson TX1**:
```
make -j4
sudo make instal
```
### Install dependancies for ORB-SLAM2
#### Pangolin
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.
#### BLAS and LAPACK
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```
#### Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. Required at least 3.1.0.
#### PCL for ROS
```
sudo apt-get install libopenni2-dev
sudo apt-get install python-vtk
```
### Building ORB_SLAM2_CUDA
Clone the repo and execute the build script for normal ORB-SLAM2:
```
git clone https://github.com/hoangthien94/ORB_SLAM2_CUDA.git ORB_SLAM2_CUDA
cd ORB_SLAM2_CUDA
chmod +x build.sh
./build.sh
```
Remember to run ```build.sh``` before building ROS because the **lib/libORB_SLAM2_CUDA.so** needs to be created first.
To build ROS node:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/path/to/ORB_SLAM2_CUDA/Examples/ROS
chmod +x build_ros.sh
./build_ros.sh
```
Done. If you have problem with the build, sometimes it helps to remove previous build folders:
```
# in ORB_SLAM2_CUDA folder
sudo rm -rf build
sudo rm -rf Thirdparty/DBoW2/build
sudo rm -rf Thirdparty/g2o/build
./build.sh
sudo rm -rf Examples/ROS/ORB_SLAM2_CUDA/build
./build_ros.sh
```
When the build is completed, you can try the examples as in the ORB-SLAM2 repo's instructions.

## Run non-ROS examples in Monocular node
Please refer to [ORB-SLAM2 repo](https://github.com/raulmur/ORB_SLAM2#4-monocular-examples) for a detailed step-by-step instruction, with two modifications:
- The executable is located in the `build` folder instead of `Examples/Monocular`.
- For `TUM` and `KITTI` examples, add a fourth argument at the end of the command, which corresponds to `bUseViewer` that enables / disables Viewer to pop up.

Example run:
```
$ cd /path/to/ORB_SLAM2_CUDA
$ ./build/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml Data/rgbd_dataset_freiburg1_desk true
```

## Run ROS launch file for Monocular node
This one is created by me. **Requires PCL library to run**. 

First you need to have the camera's image published on topic ```camera/image_raw```. 

Change the vocabulary and camera settings file accordingly. The directory is set in the launch file, located at ```ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch```

Then launch:

```roslaunch /path/to/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch```

This will run the ROS publisher node. The ROS topics will now be published in the ROS network. Run ```RVIZ``` for visualization:

```rosrun rviz rviz```

Note that Viewer is disable by default.

## Full usage:

```roslaunch /path/to/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch [bUseViewer:=(false by default)] [bEnablePublishROSTopic:=(true by default)]```

Example: 
* To launch this node with Viewer enabled:
```roslaunch /path/to/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch bUseViewer:=true```
* To launch this node without publishing any ROS topics:
```roslaunch /path/to/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch bEnablePublishROSTopic:=false```

### This is a work in progress. So expects new things and bugs fixes in future version. Happy coding.

## References and Useful links for troubleshooting
https://devtalk.nvidia.com/default/topic/1001801/orb_slam2-cuda-enhanced-running-on-a-tx2/#
https://github.com/raulmur/ORB_SLAM2/issues/202
https://github.com/raulmur/ORB_SLAM2/issues/205
https://github.com/raulmur/ORB_SLAM2/issues/209
https://github.com/raulmur/ORB_SLAM2/pull/144
https://github.com/raulmur/ORB_SLAM2/issues/317


