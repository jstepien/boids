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

static void find_neighbours(int *d_neighbours_sorted, int n, int self,
		float *d_distances, int eps) {
	eps *= eps;
	static unsigned int *d_flags = NULL, *d_neighbours = NULL;
	unsigned int blocksize = 64, flags_bytes = n * sizeof(*d_flags),
				 neighbours_bytes = n * sizeof(int);
	static size_t *d_num_valid_elems = NULL;
	size_t num_valid_elems;
	dim3 blocks(n / blocksize), threads(blocksize);

	if (!d_flags) {
		cudaMalloc((void**) &d_flags, flags_bytes);
		cudaMalloc((void**) &d_neighbours, neighbours_bytes);
		cudaMalloc((void**) &d_num_valid_elems, sizeof(*d_num_valid_elems));
	}
	cudaMemset(d_flags, 0, flags_bytes);
	check_cuda_error();

	neighbourhood<<<blocks, threads>>>(d_neighbours, d_flags, d_distances, n,
			self, eps);
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
	check_cuda_error();
	int delim = INT_MAX;
	cudaMemcpy(d_neighbours_sorted + num_valid_elems - 1, &delim, sizeof(delim),
			cudaMemcpyHostToDevice);
	check_cuda_error();
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

__device__ void separation(boid *boids, int self, const int *neighbours,
		const int n, const float *distance_cache) {
	float x = 0, y = 0;
	int count = 0, divisor, i;
	const int weight = 50;
	for (i = 0; neighbours[i] != INT_MAX; ++i) {
		int index = neighbours[i];
		float distance = sqrtf(distance_cache[self + n * index]) + 0.01f;
		x += (boids[self].x - boids[index].x) / distance;
		y += (boids[self].y - boids[index].y) / distance;
		++count;
	}
	divisor = count * weight;
	boids[self].fx = x / divisor;
	boids[self].fy = y / divisor;
}

__device__ void alignment(boid *boids, boid *self, const int *neighbours) {
	float vx = 0, vy = 0;
	int count = 0, i;
	const int weight = 10;
	for (i = 0; neighbours[i] != INT_MAX; ++i) {
		int index = neighbours[i];
		vx += boids[index].vx;
		vy += boids[index].vy;
		++count;
	}
	self->fx += vx / count / weight;
	self->fy += vy / count / weight;
}

__device__ void cohesion(const boid *boids, boid *self, const int *neighbours) {
	const int weight = 1000;
	float x = 0, y = 0;
	int i;
	for (i = 0; neighbours[i] != INT_MAX; ++i) {
		x += boids[neighbours[i]].x;
		y += boids[neighbours[i]].y;
	}
	x = x / i - self->x;
	y = y / i - self->y;
	self->fx += x / weight;
	self->fy += y / weight;
}

__global__ void calculate_forces(boid *boids, const float *distance_cache,
		const int n, const int *neighbours) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	if (ix < n && *(neighbours + n * ix) != INT_MAX) {
		separation(boids, ix, neighbours + n * ix, n, distance_cache);
		alignment(boids, boids + ix, neighbours + n * ix);
		cohesion(boids, boids + ix, neighbours + n * ix);
	}
}

void calculate_all_forces(boid* h_boids, int n, int eps, float *d_distance_cache) {
	int blocksize = 64, neighbours_bytes = n * n * sizeof(int),
		boids_bytes = n * sizeof(boid);
	dim3 threads(blocksize), blocks(n / blocksize);
	static boid *d_boids = NULL;
	static int *d_neighbours = NULL;
	if (!d_boids) {
		cudaMalloc((void**) &d_boids, boids_bytes);
		cudaMalloc((void**) &d_neighbours, neighbours_bytes);
		assert(d_boids && d_neighbours);
	}
	check_cuda_error();
	for (int i = 0; i < n; ++i)
		find_neighbours(d_neighbours + n * i, n, i, d_distance_cache, eps);
	check_cuda_error();
	cudaMemcpy(d_boids, h_boids, boids_bytes, cudaMemcpyHostToDevice);
	check_cuda_error();
	calculate_forces<<<blocks, threads>>>(d_boids,
			d_distance_cache, n, d_neighbours);
	check_cuda_error();
	cudaThreadSynchronize();
	check_cuda_error();
	cudaMemcpy(h_boids, d_boids, boids_bytes, cudaMemcpyDeviceToHost);
	check_cuda_error();
}
