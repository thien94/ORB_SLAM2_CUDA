echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2_CUDA
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j1
