#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <cuda_runtime.h>
#include <cudpp.h>
extern "C" {
	#include "simulation.h"
}

#define square(x) ((x)*(x))
#define MAX_SPEED 4

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

void reload_distance_cache(float *d_cache, boid *d_boids, int n) {
	const int blocksize = 16;
	dim3 threads(blocksize, blocksize);
	dim3 blocks(n / blocksize, n / blocksize);
	count_distance<<<blocks, threads>>>(d_boids, d_cache, n);
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

void calculate_all_forces(boid* d_boids, int n, int eps, float *d_distance_cache) {
	const int blocksize = 64;
	int neighbours_bytes = n * n * sizeof(int);
	dim3 threads(blocksize), blocks(n / blocksize);
	static int *d_neighbours = NULL;
	if (!d_neighbours) {
		cudaMalloc((void**) &d_neighbours, neighbours_bytes);
		assert(d_neighbours);
	}
	check_cuda_error();
	for (int i = 0; i < n; ++i)
		find_neighbours(d_neighbours + n * i, n, i, d_distance_cache, eps);
	check_cuda_error();
	calculate_forces<<<blocks, threads>>>(d_boids, d_distance_cache, n,
			d_neighbours);
	check_cuda_error();
}

__device__ static void normalize_speed(boid* boids) {
	const float limit_sq = square(MAX_SPEED);
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	float speed_sq = square(boids[ix].vx) + square(boids[ix].vy);
	if (speed_sq > limit_sq) {
		float coeff = MAX_SPEED / sqrtf(speed_sq);
		boids[ix].vy *= coeff;
		boids[ix].vx *= coeff;
	}
}

__global__ static void apply_forces(boid* boids, float dt, int width,
		int height) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	boids[ix].vx += boids[ix].fx * dt;
	boids[ix].vy += boids[ix].fy * dt;
	boids[ix].fx = boids[ix].fy = 0;
	normalize_speed(boids);
	boids[ix].x += boids[ix].vx * dt;
	if (boids[ix].x >= width)
		boids[ix].x -= width;
	else if (boids[ix].x < 0)
		boids[ix].x += width;
	boids[ix].y += boids[ix].vy * dt;
	if (boids[ix].y >= height)
		boids[ix].y -= height;
	else if (boids[ix].y < 0)
		boids[ix].y += height;
}

void apply_all_forces(boid* d_boids, int n, float dt, int width, int height) {
	const int blocksize = 64;
	dim3 threads(blocksize), blocks(n / blocksize);
	apply_forces<<<blocks, threads>>>(d_boids, dt, width, height);
	check_cuda_error();
}

static float* prepare_distance_cache(int n) {
	float *d_distance_cache = NULL;
	assert(n > 0);
	cudaMalloc((void**) &d_distance_cache, n * n * sizeof(float));
	assert(d_distance_cache);
	return d_distance_cache;
}

static boid * prepare_device_boids(int n) {
	int boids_bytes = n * sizeof(boid);
	boid *d_boids = NULL;
	assert(n > 0);
	cudaMalloc((void**) &d_boids, boids_bytes);
	assert(d_boids);
	return d_boids;
}

void simulate(simulation_params *sp) {
	static float *d_distance_cache = NULL;
	static boid *d_boids = NULL;
	int boids_bytes = sp->n * sizeof(boid);
	if (!d_distance_cache) {
		d_distance_cache = prepare_distance_cache(sp->n);
		d_boids = prepare_device_boids(sp->n);
	}
	cudaMemcpy(d_boids, sp->boids, boids_bytes, cudaMemcpyHostToDevice);
	check_cuda_error();
	reload_distance_cache(d_distance_cache, d_boids, sp->n);
	calculate_all_forces(d_boids, sp->n, sp->eps, d_distance_cache);
	apply_all_forces(d_boids, sp->n, sp->dt, sp->width, sp->height);
	cudaThreadSynchronize();
	check_cuda_error();
	cudaMemcpy(sp->boids, d_boids, boids_bytes, cudaMemcpyDeviceToHost);
	check_cuda_error();
}
