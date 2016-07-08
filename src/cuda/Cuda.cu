#include <helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace ORB_SLAM2 { namespace cuda {
  void deviceSynchronize() {
    checkCudaErrors( cudaDeviceSynchronize() );
  }
} }
