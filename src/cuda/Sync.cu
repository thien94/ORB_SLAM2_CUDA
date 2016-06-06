
namespace cuda {
    inline void sync(){
        cudaDeviceSynchronize();
    }
}
