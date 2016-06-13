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
    int highThreshold;
    int lowThreshold;
    unsigned int maxKeypoints;
    unsigned int count;
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

  class IC_Angle {
    KeyPoint * keypoints;
    cudaStream_t stream;
  public:
    IC_Angle();
    ~IC_Angle();
    void launch_async(InputArray _image, KeyPoint *keypoints, int npoints, int half_k);
    void join();

    static void loadUMax(const int* u_max, int count);
  };

  void deviceSynchronize();
}
#endif
