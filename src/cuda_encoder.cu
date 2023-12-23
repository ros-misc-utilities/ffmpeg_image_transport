/*
 * Copyright (C) 2016 Avionic Design GmbH
 * Meike Vocke <meike.vocke@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file contains the CUDA kernel with functions to initialise all needed
 * parameters for kernel Launch. Also the destruction of generated parameter is
 * included.
 *
 * Compute Capability 3.0 or higher required
 */

// from here: https://github.com/avionic-design/cuda-debayer/blob/master/src/bayer2rgb.cu

#include "ffmpeg_image_transport/cuda_encoder.hpp"

#define LEFT(x, y, imgw)	((x) - 1 + (y) * (imgw))
#define RIGHT(x, y, imgw)	((x) + 1 + (y) * (imgw))
#define TOP(x, y, imgw)		((x) + ((y) - 1) * (imgw))
#define BOT(x, y, imgw)		((x) + ((y) + 1) * (imgw))
#define TL(x, y, imgw)		((x) - 1 + ((y) - 1) * (imgw))
#define BL(x, y, imgw)		((x) - 1 + ((y) + 1) * (imgw))
#define TR(x, y, imgw)		((x) + 1 + ((y) - 1) * (imgw))
#define BR(x, y, imgw)		((x) + 1 + ((y) + 1) * (imgw))

#define PIX(in, x, y, imgw) \
	in[((x) + (y) * (imgw))]

#define INTERPOLATE_H(in, x, y, w) \
	(((uint32_t)in[LEFT(x, y, w)] + in[RIGHT(x, y, w)]) / 2)

#define INTERPOLATE_V(in, x, y, w) \
	(((uint32_t)in[TOP(x, y, w)] + in[BOT(x, y, w)]) / 2)

#define INTERPOLATE_HV(in, x, y, w) \
	(((uint32_t)in[LEFT(x, y, w)] + in[RIGHT(x, y, w)] + \
		in[TOP(x, y, w)] + in[BOT(x, y, w)]) / 4)

#define INTERPOLATE_X(in, x, y, w) \
	(((uint32_t)in[TL(x, y, w)] + in[BL(x, y, w)] + \
		in[TR(x, y, w)] + in[BR(x, y, w)]) / 4)

#define RED 0
#define GREEN 1
#define BLUE 2

struct cuda_vars {
	bayer_to_rgb_t kernel;
	dim3 threads_p_block;
	dim3 blocks_p_grid;

	int2 pos_r;
	int2 pos_gr;
	int2 pos_gb;
	int2 pos_b;

	uint8_t *d_bilinear;
	uint8_t *d_input;

	uint32_t width;
	uint32_t height;

	cudaStream_t streams;

	uint8_t bpp;
};

/**
 * CUDA Kernel Device code for bayer to RGB
 *
 * Computes the Bilear Interpolation of missing coloured pixel from Bayer pattern.
 * Output is RGB.
 *
 * Each CUDA thread computes four pixels in a 2x2 square. Therefore no if
 * conditions are required, which slows the CUDA kernels massively.
 *
 * The first square starts with the pixel in position 1,1. Therefore the square
 * for each thread looks like this:
 *
 * B G
 * G R
 *
 * This approach saves one pixel lines at the edges of the image in contrast to
 * the first square at 2,2 with:
 *
 * R G
 * G B
 *
 * To support other formats than RGGB we also pass the position of each color
 * channel in the 2x2 block. In the above case we get B at 0,0, Gb at 1,0,
 * Gr at 0,1 and R at 1,1.
 */
__global__ void bayer_to_rgb(uint8_t *in, uint8_t *out, uint32_t imgw,
		uint32_t imgh, uint8_t bpp, int2 r, int2 gr, int2 gb, int2 b)
{
	int x = 2 * ((blockDim.x * blockIdx.x) + threadIdx.x) + 1;
	int y = 2 * ((blockDim.y * blockIdx.y) + threadIdx.y) + 1;
	int elemCols = imgw * bpp;

	if ((x + 2) < imgw && (x - 1) >= 0 && (y + 2) < imgh && (y - 1) >= 0) {
		/* Red */
		out[(y + r.y) * elemCols + (x + r.x) * bpp + RED] =
				PIX(in, x + r.x, y + r.y, imgw);
		out[(y + r.y) * elemCols + (x + r.x) * bpp + GREEN] =
				INTERPOLATE_HV(in, x + r.x, y + r.y, imgw);
		out[(y + r.y) * elemCols + (x + r.x) * bpp + BLUE] =
				INTERPOLATE_X(in, x + r.x, y + r.y, imgw);

		/* Green on a red line */
		out[(y + gr.y) * elemCols + (x + gr.x) * bpp + RED] =
				INTERPOLATE_H(in, x + gr.x, y + gr.y, imgw);
		out[(y + gr.y) * elemCols + (x + gr.x) * bpp + GREEN] =
				PIX(in, x + gr.x, y + gr.y, imgw);
		out[(y + gr.y) * elemCols + (x + gr.x) * bpp + BLUE] =
				INTERPOLATE_V(in, x + gr.x, y + gr.y, imgw);

		/* Green on a blue line */
		out[(y + gb.y) * elemCols + (x + gb.x) * bpp + RED] =
				INTERPOLATE_V(in, x + gb.x, y + gb.y, imgw);
		out[(y + gb.y) * elemCols + (x + gb.x) * bpp + GREEN] =
				PIX(in, x + gb.x, y + gb.y, imgw);
		out[(y + gb.y) * elemCols + (x + gb.x) * bpp + BLUE] =
				INTERPOLATE_H(in, x + gb.x, y + gb.y, imgw);

		/* Blue */
		out[(y + b.y) * elemCols + (x + b.x) * bpp + RED] =
				INTERPOLATE_X(in, x + b.x, y + b.y, imgw);
		out[(y + b.y) * elemCols + (x + b.x) * bpp + GREEN] =
				INTERPOLATE_HV(in, x + b.x, y + b.y, imgw);
		out[(y + b.y) * elemCols + (x + b.x) * bpp + BLUE] =
				PIX(in, x + b.x, y + b.y, imgw);

		if (bpp == 4) {
			out[y * elemCols + x * bpp + 3] = 255;
			out[y * elemCols + (x + 1) * bpp + 3] = 255;
			out[(y + 1) * elemCols + x * bpp + 3] = 255;
			out[(y + 1) * elemCols + (x + 1) * bpp + 3] = 255;
		}
	}
}

__global__
void saxpy(int n, float a, float *x, float *y)
{
  int i = blockIdx.x*blockDim.x + threadIdx.x;
  if (i < n) y[i] = a*x[i] + y[i];
}

int test_cuda(void)
{
  int N = 1<<20;
  float *x, *y, *d_x, *d_y;
  x = (float*)malloc(N*sizeof(float));
  y = (float*)malloc(N*sizeof(float));
  

  auto ret_val = cudaMalloc(&d_x, N*sizeof(float)); 
	cudaDeviceSynchronize();
  if (ret_val != cudaSuccess) {
		fprintf(stderr, "test cudamalloc 1 %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}
  ret_val = cudaMalloc(&d_y, N*sizeof(float)); 
	cudaDeviceSynchronize();
  if (ret_val != cudaSuccess) {
		fprintf(stderr, "test cudamalloc 2 %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

  for (int i = 0; i < N; i++) {
    x[i] = 1.0f;
    y[i] = 2.0f;
  }


  ret_val = cudaMemcpy(d_x, x, N*sizeof(float), cudaMemcpyHostToDevice);
	cudaDeviceSynchronize();
  if (ret_val != cudaSuccess) {
		fprintf(stderr, "test Host to Device %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

  ret_val = cudaMemcpy(d_y, y, N*sizeof(float), cudaMemcpyHostToDevice);
	cudaDeviceSynchronize();
  if (ret_val != cudaSuccess) {
		fprintf(stderr, "test2 Host to Device %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

  // Perform SAXPY on 1M elements
  saxpy<<<(N+255)/256, 256>>>(N, 2.0f, d_x, d_y);
	cudaError_t err = cudaGetLastError();
	if (err != cudaSuccess) 
		printf("Error: %s\n", cudaGetErrorString(err));

  cudaMemcpy(y, d_y, N*sizeof(float), cudaMemcpyDeviceToHost);

  float maxError = 0.0f;
  for (int i = 0; i < N; i++)
    maxError = max(maxError, abs(y[i]-4.0f));
  printf("Max error: %f\n", maxError);

  cudaFree(d_x);
  cudaFree(d_y);
  free(x);
  free(y);
}

cudaError_t bayer2rgb_process(struct cuda_vars *gpu_vars, const void *p,
		uint8_t **output, cudaStream_t *stream, bool get_dev_ptr)
{
	cudaError_t ret_val;

	if (gpu_vars == NULL)
		return cudaErrorInitializationError;


	ret_val = cudaMemcpy(gpu_vars->d_input, p, gpu_vars->width*gpu_vars->height * sizeof(uint8_t), cudaMemcpyHostToDevice);
	if (ret_val != cudaSuccess) {
		fprintf(stderr, "Host to Device %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

	gpu_vars->kernel<<<gpu_vars->blocks_p_grid,
			gpu_vars->threads_p_block, 0,
			gpu_vars->streams
		>>>(gpu_vars->d_input,
			gpu_vars->d_bilinear,
			gpu_vars->width, gpu_vars->height, gpu_vars->bpp,
			gpu_vars->pos_r, gpu_vars->pos_gr,
			gpu_vars->pos_gb, gpu_vars->pos_b);
	cudaError_t err = cudaGetLastError();
	if (err != cudaSuccess) 
		printf("Error: %s\n", cudaGetErrorString(err));


	if (get_dev_ptr) {
		*output = (uint8_t *)gpu_vars->d_bilinear;
	} else {
	}

	*stream = gpu_vars->streams;


	return cudaSuccess;
}

cudaError_t alloc_create_cuda_data(struct cuda_vars *gpu_vars)
{
	cudaError_t ret_val = cudaSuccess;

	ret_val = cudaMalloc(&gpu_vars->d_input, gpu_vars->width * gpu_vars->height * sizeof(uint8_t));
	if (ret_val != cudaSuccess) {
		fprintf(stderr, "cudaMalloc d_input %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

	ret_val = cudaMalloc(&gpu_vars->d_bilinear, gpu_vars->width *
			gpu_vars->height * gpu_vars->bpp * sizeof(uint8_t));
	if (ret_val != cudaSuccess) {
		fprintf(stderr, "cudaMalloc d_bilinear %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

	ret_val = cudaStreamCreate(&gpu_vars->streams);
	if (ret_val != cudaSuccess) {
		fprintf(stderr, "cudaStreamCreate %d, %s\n", 0,
				cudaGetErrorString(ret_val));
		return ret_val;
	}

	return ret_val;
}

cudaError_t bayer2rgb_init(struct cuda_vars **gpu_vars_p, uint32_t width,
		uint32_t height, uint8_t bpp, bool thermal)
{
	struct cuda_vars *gpu_vars;
	bayer_to_rgb_t ir_kernel;
	cudaError_t ret_val;
	int i;

	if (gpu_vars_p == NULL)
		return cudaErrorInitializationError;

	gpu_vars = (cuda_vars *) new(struct cuda_vars);
	if (!gpu_vars)
		return cudaErrorMemoryAllocation;

	gpu_vars->width = width;
	gpu_vars->height = height;
	gpu_vars->bpp = bpp;
	gpu_vars->kernel = bayer_to_rgb;

	gpu_vars->pos_r = make_int2(1, 1);
	gpu_vars->pos_gr = make_int2(0, 1);
	gpu_vars->pos_gb = make_int2(1, 0);
	gpu_vars->pos_b = make_int2(0, 0);

	ret_val = alloc_create_cuda_data(gpu_vars);
	if (ret_val != cudaSuccess)
		goto cleanup;

	gpu_vars->threads_p_block = dim3(32, 32);
	gpu_vars->blocks_p_grid.x = (gpu_vars->width / 2 +
			gpu_vars->threads_p_block.x - 1) /
			gpu_vars->threads_p_block.x;
	gpu_vars->blocks_p_grid.y = (gpu_vars->height / 2 +
			gpu_vars->threads_p_block.y - 1) /
			gpu_vars->threads_p_block.y;

	*gpu_vars_p = gpu_vars;

	return cudaSuccess;

cleanup:
	bayer2rgb_free(gpu_vars);

	return ret_val;
}

void free_cuda_data(struct cuda_vars *gpu_vars)
{
	if (gpu_vars->d_input)
		cudaFree(gpu_vars->d_input);
	if (gpu_vars->d_bilinear)
		cudaFree(gpu_vars->d_bilinear);
	cudaStreamDestroy(gpu_vars->streams);
}

cudaError_t bayer2rgb_free(struct cuda_vars *gpu_vars)
{
	free_cuda_data(gpu_vars);

	free(gpu_vars);

	return cudaSuccess;
}



// From here: https://stackoverflow.com/questions/61457243/problem-of-converting-bgr-to-yuv420p-with-cuda
__host__ __device__ unsigned char rgb2y(int R, int G, int B){
  int Y = ((66 * R + 129 * G + 25 * B + 128) >> 8) + 16;
  return (unsigned char)((Y<0)? 0 : ((Y > 255) ? 255 : Y));}
__host__ __device__ int rgb2u(int R, int G, int B){
  int U = ((-38 * R - 74 * G + 112 * B + 128) >> 8) + 128;
  return (unsigned char)((U<0)? 0 : ((U > 255) ? 255 : U));}
__host__ __device__ int rgb2v(int R, int G, int B){
  int V = ((112 * R - 94 * G - 18 * B + 128) >> 8) + 128;
  return (unsigned char)((V<0)? 0 : ((V > 255) ? 255 : V));}

//kernel function to convert rgb to yuv420p
__global__ void rgb2yuv420p(uint8_t *d_in, uint8_t *d_out,
                               uint imgheight, uint imgwidth)
{

    int col_num = blockIdx.x*blockDim.x+threadIdx.x;
    int row_num = blockIdx.y*blockDim.y+threadIdx.y;

    if ((row_num < imgheight) && (col_num < imgwidth))
    {
//        uint32_t a = *((uint32_t *)&dinput[global_offset*3]);
        int global_offset = row_num * imgwidth * 3 + col_num * 3;

        int r,g,b;
        
        r = int(d_in[global_offset + RED]);
        g = int(d_in[global_offset + GREEN]);
        b = int(d_in[global_offset + BLUE]);

        d_out[row_num * imgwidth + col_num] = rgb2y(r,g,b);
		// https://stackoverflow.com/questions/27822017/planar-yuv420-data-layout

        if(((threadIdx.x & 1) == 0)  && ((threadIdx.y & 1) == 0)){ // 1 = 0001
			int u_offset = imgwidth*imgheight+((row_num>>1)*(imgwidth>>1))+(col_num>>1);
            d_out[u_offset] = rgb2u(r,g,b);
            int v_offset = u_offset+((imgheight>>1)*(imgwidth>>1));
            d_out[v_offset] = rgb2v(r,g,b);
        }
    }
}

cudaError_t rgb2yuv420p_process(uint8_t *d_in, uint8_t *d_out,
                               uint imgheight, uint imgwidth)
{
	cudaError_t ret_val;
	dim3 threadsPerBlock(32, 32);
	dim3 blocksPerGrid((imgwidth + threadsPerBlock.x - 1) / threadsPerBlock.x,
					(imgheight + threadsPerBlock.y - 1) / threadsPerBlock.y);

	//run rgb->yuv420p kernel function
	rgb2yuv420p<<<blocksPerGrid, threadsPerBlock>>>(d_in, d_out, imgheight, imgwidth);
	cudaError_t err = cudaGetLastError();
	if (err != cudaSuccess) 
		printf("Error: %s\n", cudaGetErrorString(err));

	return cudaSuccess;
}