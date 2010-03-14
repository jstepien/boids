#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#include "boid.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4
#define UNDEF (FLT_MAX)

float *distance_cache = NULL;
int distance_cache_size = 0;
boid *boids = NULL;

unsigned int boid_distance(int a, int b) {
#if 0
	int index = a + distance_cache_size * b;
	assert(index < 640 * 480);
	assert(index >= 0);
	if (distance_cache[index] == UNDEF)
		distance_cache[index] = square(boids[b].y - boids[a].y) +
			square(boids[a].x - boids[b].x);
	return distance_cache[index];
#endif
	return square(boids[b].y - boids[a].y) +
			square(boids[a].x - boids[b].x);
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
	boid_clear_distance_cache();
}

void boid_clear_distance_cache() {
	for (int i = 0; i < distance_cache_size; ++i)
		distance_cache[i] = UNDEF;
#ifndef NDEBUG
	for (int i = 0; i < distance_cache_size; ++i)
		assert(distance_cache[i] == UNDEF);
#endif
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
