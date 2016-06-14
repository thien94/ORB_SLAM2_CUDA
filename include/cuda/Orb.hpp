#pragma once
#ifndef __ORB_HPP__
#define __ORB_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <cuda/Allocator.h>
#include <cuda_runtime.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace Orb {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  class GpuOrb {
    unsigned int maxKeypoints;
    KeyPoint * keypoints;
    GpuMat descriptors;
    cudaStream_t stream;
    Stream cvStream;
  public:
    GpuOrb(int maxKeypoints = 10000);
    ~GpuOrb();

    void launch_async(InputArray _image, const KeyPoint * _keypoints, const int npoints, Mat &_descriptors);
    void join();

    static void loadPattern(const Point * _pattern);
  };
}
#endif
