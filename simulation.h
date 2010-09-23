#ifndef SIMULATION_H_
#define SIMULATION_H_

#define TURNING_SPEED M_PI/32

typedef struct {
	float x;
	float y;
	float vx;
	float vy;
	float fx;
	float fy;
} boid;

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
	char *intensity;
	pair *attractor;
} simulation_params;

void simulate(simulation_params *params);

#endif /* SIMULATION_H_ */
