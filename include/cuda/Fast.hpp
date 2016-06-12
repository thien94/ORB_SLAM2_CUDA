#pragma once
#ifndef __FAST_HPP__
#define __FAST_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <cuda/Allocator.h>
#include <cuda_runtime.h>

namespace Fast {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  const float FEATURE_SIZE = 7.0;

  void tileDetect_gpu(InputArray _image, std::vector<KeyPoint> &keypoints, int highThreshold, int lowThreshold);

  class GpuFast {
    short2 * kpLoc;
    float * kpScore;
    unsigned int * counter_ptr;
    const int highThreshold;
    const int lowThreshold;
    const unsigned int maxKeypoints;
    cv::cuda::GpuMat scoreMat;
    cudaStream_t stream;
    Stream cvStream;
  public:
    GpuFast(int highThreshold, int lowThreshold, int maxKeypoints = 10000);
    ~GpuFast();

    void detect(InputArray, std::vector<KeyPoint>&);

    void detectAsync(InputArray);
    void joinDetectAsync(std::vector<KeyPoint>&);
  };
}
#endif
