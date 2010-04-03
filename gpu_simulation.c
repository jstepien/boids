#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include "simulation.h"
#include "gpu_kernels.h"

float *distance_cache = NULL;
float *d_distance_cache = NULL;
int distance_cache_size = 0;
boid *boids = NULL;

static void prepare_distance_cache(boid *_boids, int n) {
	assert(_boids);
	assert(n > 0);
	distance_cache = (float*) malloc(sizeof(*distance_cache) * n * n);
	assert(distance_cache);
	cudaMalloc((void**) &d_distance_cache, n * n * sizeof(float));
	boids = _boids;
	distance_cache_size = n;
}

static void free_distance_cache() {
	free(distance_cache);
	distance_cache = NULL;
	distance_cache_size = 0;
	boids = NULL;
}

void simulate(simulation_params *sp) {
	int i = 0;
	if (!distance_cache)
		prepare_distance_cache(sp->boids, sp->n);
	reload_distance_cache(d_distance_cache, distance_cache, sp->boids, sp->n);
	calculate_all_forces(sp->boids, sp->n, sp->eps, d_distance_cache);
	apply_all_forces(sp->boids, sp->n, sp->dt, sp->width, sp->height);
}
