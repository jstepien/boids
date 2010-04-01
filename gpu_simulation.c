#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include "simulation.h"
#include "gpu_kernels.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4

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

static void normalize_speed(boid *self) {
	float speed_sq = square(self->vx) + square(self->vy);
	float limit_sq = square(MAX_SPEED);
	if (speed_sq > limit_sq) {
		float coeff = MAX_SPEED / sqrtf(speed_sq);
		self->vy *= coeff;
		self->vx *= coeff;
	}
}

static void apply_forces(simulation_params *sp, boid* boid) {
	boid->vx += boid->fx * sp->dt;
	boid->vy += boid->fy * sp->dt;
	boid->fx = boid->fy = 0;
	normalize_speed(boid);
	boid->x += boid->vx * sp->dt;
	if (boid->x >= sp->width)
		boid->x -= sp->width;
	else if (boid->x < 0)
		boid->x += sp->width;
	boid->y += boid->vy * sp->dt;
	if (boid->y >= sp->height)
		boid->y -= sp->height;
	else if (boid->y < 0)
		boid->y += sp->height;
}

void simulate(simulation_params *sp) {
	int i = 0;
	if (!distance_cache)
		prepare_distance_cache(sp->boids, sp->n);
	reload_distance_cache(d_distance_cache, distance_cache, sp->boids, sp->n);
	calculate_all_forces(sp->boids, sp->n, sp->eps, d_distance_cache);
	for (i = 0; i < sp->n; ++i)
		apply_forces(sp, &sp->boids[i]);
}
