#include "cpu_simulation.c"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "simulation.h"

#define WIDTH 640
#define HEIGHT 480

#define EPS 10
#define DT 0.1f

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

static void print_state(simulation_params *sp) {
	printf("[");
	for (int i = 0; i < sp->n; ++i) {
		printf("[%f,%f,%f,%f],", sp->boids[i].x, sp->boids[i].y,
				sp->boids[i].vx, sp->boids[i].vy);
	}
	printf("]\n");
}

static boid* build_flock(int n, char **argv) {
	int i, j, cur = 0, argv_index = 0;
	float sqrt_n = ceilf(sqrtf(n)), dx = WIDTH / (sqrt_n + 1),
		  dy = HEIGHT / (sqrt_n + 1);
	boid *boids = calloc(n, sizeof(boid));
	if (!boids)
		return NULL;
	if (argv) {
		do {
			boids[cur].x = atof(argv[argv_index++]);
			boids[cur].y = atof(argv[argv_index++]);
			boids[cur].vx = atof(argv[argv_index++]);
			boids[cur].vy = atof(argv[argv_index++]);
		} while (++cur < n);
	} else {
		for (i = 0; i < sqrt_n; ++i) {
			for (j = 0; j < sqrt_n && cur < n; ++j) {
				boids[cur].vx = sinf((1559 * cur) ^ 50969);
				boids[cur].vy = cosf((1567 * cur) ^ 51853);
				boids[cur].y = (i + 1) * dy;
				boids[cur].x = (j + 1) * dx;
				++cur;
			}
		}
	}
	return boids;
}

static void simulation_step(simulation_params *sp) {
	simulate(sp);
	print_state(sp);
}

static int correct(int n) {
	if (n > 0 && (n % 64 == 0))
		return 1;
	else {
		fprintf(stderr, "%s:%i: n has to be a multiple of 64\n", __FILE__,
				__LINE__);
		return 0;
	}
}

int main(int argc, char* argv[]) {
	simulation_params sp = {WIDTH, HEIGHT, NULL, -1, EPS, DT, NULL, NULL};
	if (argc < 2)
		sp.n = 256;
	else
		sp.n = atoi(argv[1]);
	if (!correct(sp.n))
		return 1;
	sp.boids = build_flock(sp.n, argc > 2 ? (argv + 2) : NULL);
	simulation_step(&sp);
	free(sp.boids);
	return 0;
}
