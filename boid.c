#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#include "boid.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4

unsigned int boid_distance(boid *a, boid *b) {
	return square(b->y - a->y) + square(a->x - b->x);
}

float boid_real_distance(boid *a, boid *b) {
	return sqrtf(boid_distance(a, b));
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
