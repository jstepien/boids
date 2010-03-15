#ifndef BOID_H_
#define BOID_H_

typedef struct {
	float x;
	float y;
	float vx;
	float vy;
	float fx;
	float fy;
} boid;

unsigned int boid_distance(boid *a, boid *b);

float boid_real_distance(boid *a, boid *b);

void boid_normalize_speed(boid *self);

#endif /* BOID_H_ */
