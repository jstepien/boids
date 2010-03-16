#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#include <stdio.h>
#include "boid.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4
#define UNDEF (FLT_MAX)

float *distance_cache = NULL;
int distance_cache_size = 0;
boid *boids = NULL;

unsigned int boid_distance(int a, int b) {
	static unsigned int cache_hits = 0, cache_misses = 0;
	int index = a + distance_cache_size * b,
		second_index = b + distance_cache_size * a;
	assert(index < square(distance_cache_size));
	assert(index >= 0);
	if (distance_cache[index] == UNDEF) {
#ifndef NDEBUG
		++cache_misses;
#endif
		if (distance_cache[second_index] != UNDEF)
			return distance_cache[second_index];
		else
			distance_cache[index] = square(boids[b].y - boids[a].y) +
				square(boids[a].x - boids[b].x);
	}
#ifndef NDEBUG
	else
		++cache_hits;
	if (cache_hits >= 10000000 || cache_misses >= 10000000) {
		printf("Cache: %i hits, %i misses, %i%%\n", cache_hits, cache_misses,
				100 * cache_hits / (cache_hits + cache_misses));
		cache_misses = cache_hits = 0;
	}
#endif
	return distance_cache[index];
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
	int limit = square(distance_cache_size);
	for (int i = 0; i < limit; ++i)
		distance_cache[i] = UNDEF;
#ifndef NDEBUG
	for (int i = 0; i < limit; ++i)
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
