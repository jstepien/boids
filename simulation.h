#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "boid.h"

void simulate(int width, int height, boid* boids, int n, int eps, float dt);

#endif /* SIMULATION_H_ */
