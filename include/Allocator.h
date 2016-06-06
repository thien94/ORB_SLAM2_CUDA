#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include <opencv2/core/cuda.hpp>

namespace ORB_SLAM2
{

class Allocator : public cv::cuda::GpuMat::Allocator
{
    const int allocatorPitchBase = 128;
    size_t getPitch(size_t widthSize);

public:

    bool allocate(cv::cuda::GpuMat* mat, int rows, int cols, size_t elemSize);
    void free(cv::cuda::GpuMat* mat);
};

}

#endif
