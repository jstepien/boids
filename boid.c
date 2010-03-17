#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#include <stdio.h>
#include "boid.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4

float *distance_cache = NULL;
int distance_cache_size = 0;
boid *boids = NULL;

inline unsigned int boid_distance(int a, int b) {
	return distance_cache[a + distance_cache_size * b];
}

float boid_real_distance(int a, int b) {
	return sqrtf(boid_distance(a, b));
}

void boid_prepare_distance_cache(boid *_boids, int n) {
	assert(_boids);
	assert(n > 0);
	distance_cache = malloc(sizeof(*distance_cache) * n * n);
	assert(distance_cache);
	boids = _boids;
	distance_cache_size = n;
}

void boid_reload_distance_cache() {
	int i, j;
#pragma omp parallel for private(j)
	for (i = 0; i < distance_cache_size; ++i)
		for (j = i; j < distance_cache_size; ++j)
			distance_cache[i + distance_cache_size * j] =
				distance_cache[j + distance_cache_size * i] =
				square(boids[j].y - boids[i].y) +
				square(boids[j].x - boids[i].x);
}

void boid_free_distance_cache() {
	free(distance_cache);
	distance_cache = NULL;
	distance_cache_size = 0;
	boids = NULL;
}

void boid_normalize_speed(boid *self) {
	float speed_sq = square(self->vx) + square(self->vy);
	float limit_sq = square(MAX_SPEED);
	if (speed_sq > limit_sq) {
		float coeff = MAX_SPEED / sqrtf(speed_sq);
		self->vy *= coeff;
		self->vx *= coeff;
	}
}
