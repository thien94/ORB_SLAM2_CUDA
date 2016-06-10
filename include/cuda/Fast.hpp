#pragma once
#ifndef __FAST_HPP__
#define __FAST_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <cuda/Allocator.h>

namespace Fast {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  enum {
    LOCATION_ROW = 0,
    RESPONSE_ROW,
    ROWS_COUNT,
    FEATURE_SIZE = 7
  };

  /*
  int calcKeypoints_gpu(PtrStepSzb img, PtrStepSzb mask, short2* kpLoc, int maxKeypoints, PtrStepSzi score, int threshold, cudaStream_t stream);

  void detect(InputArray _image, std::vector<KeyPoint> &keypoints, int threshold, int max_npoints_);
  void detectAsync(InputArray _image, OutputArray _keypoints, int threshold, int max_npoints_, Stream& stream);
  void convert(InputArray _gpu_keypoints, std::vector<KeyPoint>& keypoints);
  */

  void tileDetect_gpu(InputArray _image, int minBorderX, int maxBorderX, int minBorderY, int maxBorderY, std::vector<KeyPoint> &keypoints, int highThreshold, int lowThreshold);
}
#endif
