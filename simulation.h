#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "boid.h"

typedef struct {
	int x, y;
} pair;

typedef struct {
	int width;
	int height;
	boid *boids;
	int n;
	int eps;
	float dt;
	pair *attractor;
} simulation_params;

void simulate(simulation_params *params);

#endif /* SIMULATION_H_ */
