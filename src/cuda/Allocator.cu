#include "Allocator.h"
#include <cassert>

#define cudaCheck(stat) assert(stat == cudaSuccess)

namespace ORB_SLAM2
{

size_t Allocator::getPitch(size_t widthSize){
    return 128 + widthSize - widthSize%128;
}

bool Allocator::allocate(cv::cuda::GpuMat* mat, int rows, int cols, size_t elemSize)
{
    if (rows > 1 && cols > 1)
    {
        mat->step = getPitch(elemSize * cols);
        cudaCheck(cudaMallocManaged(&mat->data, mat->step * row));
    }
    else
    {
        // Single row or single column must be continuous
        cudaCheck(cudaMallocManaged(&mat->data, elemSize * cols * rows));
        mat->step = elemSize * cols;
    }

    mat->refcount = (int*) new int();

    return true;
}

void Allocator::free(cv::cuda::GpuMat* mat)
{
    cudaFree(mat->datastart);
    delete mat->refcount;
}

}
