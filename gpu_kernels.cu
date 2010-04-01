extern "C" {
	#include <stdio.h>
	#include <assert.h>
	#include "simulation.h"
	#include "gpu_kernels.h"
}

#include <cudpp.h>

#define square(x) ((x)*(x))

#define check_cuda_error() {\
	if (cudaError_t e = cudaGetLastError()) { \
		fprintf(stderr, "%s:%i: %s\n", __FILE__, __LINE__, \
				cudaGetErrorString(e)); \
		exit(-1); \
	} }

__global__ static void neighbourhood(unsigned int *neighbours,
		unsigned int *flags, const float *d_distances, const int n,
		const int self, const int eps_sq) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	if (ix < n) {
		neighbours[ix] = ix;
		if (d_distances[self * n + ix] <= eps_sq)
			flags[ix] = 1;
	}
}

inline static void run_kernel(int *neighbours, int n, int self,
		float *d_distances, int eps_sq) {
	static unsigned int *d_flags = NULL, *d_neighbours = NULL,
		*d_neighbours_sorted = NULL;
	unsigned int blocksize = 64, flags_bytes = n * sizeof(*d_flags),
				 neighbours_bytes = n * sizeof(int);
	static size_t *d_num_valid_elems = NULL;
	size_t num_valid_elems;
	dim3 blocks(n / blocksize), threads(blocksize);

	if (!d_flags) {
		cudaMalloc((void**) &d_flags, flags_bytes);
		cudaMalloc((void**) &d_neighbours, neighbours_bytes);
		cudaMalloc((void**) &d_neighbours_sorted, neighbours_bytes);
		cudaMalloc((void**) &d_num_valid_elems, sizeof(*d_num_valid_elems));
	}
	cudaMemset(d_flags, 0, flags_bytes);
	check_cuda_error();

	neighbourhood<<<blocks, threads>>>(d_neighbours, d_flags, d_distances, n,
			self, eps_sq);
	check_cuda_error();

	CUDPPHandle theCudpp;
	cudppCreate(&theCudpp);
	CUDPPConfiguration config;
	config.datatype = CUDPP_INT;
	config.algorithm = CUDPP_COMPACT;
	config.options = CUDPP_OPTION_FORWARD;
	CUDPPHandle planhandle = 0;
	CUDPPResult result = cudppPlan(theCudpp, &planhandle, config, n, 1, 0);
	if (CUDPP_SUCCESS != result)
	{
		printf("Error creating CUDPPPlan\n");
		exit(-1);
	}
	cudppCompact(planhandle, d_neighbours_sorted, d_num_valid_elems,
			d_neighbours, d_flags, n);

	result = cudppDestroyPlan(planhandle);
    if (CUDPP_SUCCESS != result)
    {
        printf("Error destroying CUDPPPlan\n");
        exit(-1);
    }

	check_cuda_error();
	cudaThreadSynchronize();
	check_cuda_error();

	cudaMemcpy(&num_valid_elems, d_num_valid_elems,
			sizeof(*d_neighbours_sorted), cudaMemcpyDeviceToHost);
	cudaMemcpy(neighbours, d_neighbours_sorted, neighbours_bytes,
			cudaMemcpyDeviceToHost);
	neighbours[num_valid_elems] = INT_MAX;
}

int find_neighbours(int *neighbours, int n, int self, float *d_distances,
		int eps) {
	int neighbour_id = 0;
	run_kernel(neighbours, n, self, d_distances, eps * eps);
	return neighbour_id;
}

__global__ void count_distance(const boid *boids, float *distance, const int n)
{
   int ix = blockIdx.x * blockDim.x + threadIdx.x,
	   iy = blockIdx.y * blockDim.y + threadIdx.y;
   if (ix < n && iy < n)
	   distance[ix + n * iy] = square(boids[iy].y - boids[ix].y) +
		   square(boids[iy].x - boids[ix].x);
}

void reload_distance_cache(float *d_cache, float *h_cache, boid *boids, int n) {
	int blocksize = 16, cache_bytes = n * n * sizeof(float),
		boids_bytes = n * sizeof(boid);
	dim3 threads(blocksize, blocksize);
	dim3 blocks(n / blocksize, n / blocksize);
	static boid *d_boids = NULL;
	if (!d_boids)
		cudaMalloc((void**) &d_boids, boids_bytes);
	cudaMemcpy(d_boids, boids, boids_bytes, cudaMemcpyHostToDevice);
	count_distance<<<blocks, threads>>>(d_boids, d_cache, n);
	cudaThreadSynchronize();
	cudaMemcpy(h_cache, d_cache, cache_bytes, cudaMemcpyDeviceToHost);
}
