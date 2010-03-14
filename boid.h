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

unsigned int boid_distance(int a, int b);

float boid_real_distance(int a, int b);

void boid_prepare_distance_cache(boid *boids, int size);

void boid_clear_distance_cache();

void boid_free_distance_cache();

#endif /* BOID_H_ */
