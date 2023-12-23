// from here: https://github.com/avionic-design/cuda-debayer/blob/master/src/bayer2rgb_kernel.h

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <time.h>
#include <cstdlib>
struct cuda_vars;

cudaError_t bayer2rgb_init(struct cuda_vars **gpu_vars, uint32_t width,
		uint32_t height, uint8_t bpp, bool thermal);
cudaError_t bayer2rgb_free(struct cuda_vars *gpu_vars);

cudaError_t bayer2rgb_process(struct cuda_vars *gpu_vars, const void *p,
		uint8_t **output, cudaStream_t *stream, bool get_dev_ptr);
cudaError_t rgb2yuv420p_process(uint8_t *d_in, uint8_t *d_out,
                               uint imgheight, uint imgwidth);
		
int test_cuda(void);
int test_cuda(CUcontext ctx);

typedef void (*bayer_to_rgb_t)(uint8_t *in, uint8_t *out, uint32_t imgw,
		uint32_t imgh, uint8_t bpp, int2 r, int2 gr, int2 gb, int2 b);

/**
 * CUDA Kernel Device code for RGGB
 *
 * Computes the Bilear Interpolation of missing coloured pixel from Bayer pattern.
 * Output is RGB.
 */
__global__ void bayer_to_rgb(uint8_t *in, uint8_t *out, uint32_t imgw,
		uint32_t imgh, uint8_t bpp, int2 r, int2 gr, int2 gb, int2 b);

__global__ void rgb2yuv420p(uint8_t *d_in, uint8_t *d_out,
                               uint imgheight, uint imgwidth);


