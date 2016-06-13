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

  void computeOrbDescriptors(InputArray _image, const KeyPoint * _keypoints, const int npoints, Mat &descriptors, const Point * pattern);
}
#endif
