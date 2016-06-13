#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/utility.hpp"
#include "opencv2/core/cuda/reduce.hpp"
#include "opencv2/core/cuda/functional.hpp"
#include <helper_cuda.h>
#include <cuda/Orb.hpp>
#include <Utils.hpp>

using namespace cv;
using namespace cv::cuda;
using namespace cv::cuda::device;

namespace Orb {

#define GET_VALUE(idx) \
    image(loc.y + __float2int_rn(pattern[idx].x * b + pattern[idx].y * a), \
          loc.x + __float2int_rn(pattern[idx].x * a - pattern[idx].y * b))

  __global__ void calcOrb_kernel(const PtrStepb image, KeyPoint * keypoints, const int npoints, const Point * pattern, PtrStepb descriptors) {
    int id = blockIdx.x;
    int tid = threadIdx.x;
    if (id >= npoints) return;

    const KeyPoint &kpt = keypoints[id];
    short2 loc = make_short2(kpt.pt.x, kpt.pt.y);
    pattern += 16 * tid;

    uchar * desc = descriptors.ptr(id);
    const float factorPI = (float)(CV_PI/180.f);
    float angle = (float)kpt.angle * factorPI;
    float a = (float)cosf(angle), b = (float)sinf(angle);

    int t0, t1, val;
    t0 = GET_VALUE(0); t1 = GET_VALUE(1);
    val = t0 < t1;
    t0 = GET_VALUE(2); t1 = GET_VALUE(3);
    val |= (t0 < t1) << 1;
    t0 = GET_VALUE(4); t1 = GET_VALUE(5);
    val |= (t0 < t1) << 2;
    t0 = GET_VALUE(6); t1 = GET_VALUE(7);
    val |= (t0 < t1) << 3;
    t0 = GET_VALUE(8); t1 = GET_VALUE(9);
    val |= (t0 < t1) << 4;
    t0 = GET_VALUE(10); t1 = GET_VALUE(11);
    val |= (t0 < t1) << 5;
    t0 = GET_VALUE(12); t1 = GET_VALUE(13);
    val |= (t0 < t1) << 6;
    t0 = GET_VALUE(14); t1 = GET_VALUE(15);
    val |= (t0 < t1) << 7;

    desc[tid] = (uchar)val;
  }

  //void computeOrbDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, const vector<Point>& pattern)
  void computeOrbDescriptors(InputArray _image, const KeyPoint * _keypoints, const int npoints, Mat & _descriptors, const Point * _pattern) {
    PUSH_RANGE("computeDescriptors", 1);
    if (npoints == 0) {
      POP_RANGE;
      return ;
    }
    cudaStream_t stream = 0;
    const GpuMat image = _image.getGpuMat();
    KeyPoint * keypoints;
    checkCudaErrors( cudaMalloc(&keypoints, sizeof(KeyPoint) * npoints) );
    checkCudaErrors( cudaMemcpyAsync(keypoints, _keypoints, sizeof(KeyPoint) * npoints, cudaMemcpyHostToDevice, stream) );
    Point * pattern;
    checkCudaErrors( cudaMalloc(&pattern, sizeof(Point) * 512) );
    checkCudaErrors( cudaMemcpyAsync(pattern, _pattern, sizeof(Point) * 512, cudaMemcpyHostToDevice, stream) );

    GpuMat descriptors(npoints, 32, CV_8UC1, Scalar::all(0));

    dim3 dimBlock(32);
    dim3 dimGrid(npoints);
    calcOrb_kernel<<<dimGrid, dimBlock, 0, stream>>>(image, keypoints, npoints, pattern, descriptors);
    checkCudaErrors( cudaGetLastError() );

    checkCudaErrors( cudaStreamSynchronize(stream) );

    descriptors.download(_descriptors);
    checkCudaErrors( cudaFree(keypoints) );
    POP_RANGE;
  }

}
