#include <cassert>
#include <helper_cuda.h>

#include <cuda/Allocator.h>

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
        checkCudaErrors(cudaMallocManaged(&mat->data, mat->step * rows));
    }
    else
    {
        // Single row or single column must be continuous
        checkCudaErrors(cudaMallocManaged(&mat->data, elemSize * cols * rows));
        mat->step = elemSize * cols;
    }

    mat->refcount = (int*) new int();

    return true;
}

void Allocator::free(cv::cuda::GpuMat* mat)
{
    checkCudaErrors(cudaFree(mat->datastart));
    delete mat->refcount;
}

cv::cuda::GpuMat::Allocator * gpu_mat_allocator;

}


namespace {
  void __attribute__((constructor)) init() {
    // Setup GPU Memory Management
    ORB_SLAM2::gpu_mat_allocator = new ORB_SLAM2::Allocator();
    // cv::cuda::GpuMat::setDefaultAllocator(ORB_SLAM2::gpu_mat_allocator);
  }
}
