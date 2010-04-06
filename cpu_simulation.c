#include <math.h>
#include <assert.h>
#include <glib.h>
#include <stdlib.h>
#include "simulation.h"

#define square(x) ((x)*(x))
#define MAX_SPEED 4

/*
 * Yeah, global variables are ugly, but without them boid_distane couldn't have
 * been inlined -- inlining gives speeeed!
 */
float *distance_cache = NULL;
int distance_cache_size = 0;
boid *boids = NULL;

static inline unsigned int boid_distance(int a, int b) {
	return distance_cache[a + distance_cache_size * b];
}

static inline float boid_real_distance(int a, int b) {
	return sqrtf(boid_distance(a, b));
}

static void prepare_distance_cache(boid *_boids, int n) {
	assert(_boids);
	assert(n > 0);
	distance_cache = malloc(sizeof(*distance_cache) * n * n);
	assert(distance_cache);
	boids = _boids;
	distance_cache_size = n;
}

static void reload_distance_cache() {
	int i, j;
#pragma omp parallel for private(j)
	for (i = 0; i < distance_cache_size; ++i)
		for (j = i; j < distance_cache_size; ++j)
			distance_cache[i + distance_cache_size * j] =
				distance_cache[j + distance_cache_size * i] =
				square(boids[j].y - boids[i].y) +
				square(boids[j].x - boids[i].x);
}

static void free_distance_cache() {
	free(distance_cache);
	distance_cache = NULL;
	distance_cache_size = 0;
	boids = NULL;
}

static void normalize_speed(boid *self) {
	float speed_sq = square(self->vx) + square(self->vy);
	float limit_sq = square(MAX_SPEED);
	if (speed_sq > limit_sq) {
		float coeff = MAX_SPEED / sqrtf(speed_sq);
		self->vy *= coeff;
		self->vx *= coeff;
	}
}

static void separation(boid *boids, int this, GList *others) {
	float x = 0, y = 0;
	int count = 0, divisor;
	const int weight = 50;
	do {
		int index = GPOINTER_TO_INT(others->data);
		float distance = boid_real_distance(this, index) + 0.01f;
		assert(distance > 0);
		x += (boids[this].x - boids[index].x) / distance;
		y += (boids[this].y - boids[index].y) / distance;
		++count;
	} while (others = g_list_next(others));
	divisor = count * weight;
	boids[this].fx = x / divisor;
	boids[this].fy = y / divisor;
}

static void alignment(boid *boids, boid *this, GList *others) {
	GList *current = g_list_first(others);
	float vx = 0, vy = 0;
	int count = 0;
	const int weight = 10;
	do {
		int index = GPOINTER_TO_INT(current->data);
		vx += boids[index].vx;
		vy += boids[index].vy;
		++count;
	} while (current = g_list_next(current));
	this->fx += vx / count / weight;
	this->fy += vy / count / weight;
}

static void cohesion(boid *boids, boid *this, GList *others) {
	GList *current = g_list_first(others);
	float x = 0, y = 0;
	int count = 0;
	const int weight = 1000;
	do {
		int index = GPOINTER_TO_INT(current->data);
		x += boids[index].x;
		y += boids[index].y;
		++count;
	} while (current = g_list_next(current));
	x = x / count - this->x;
	y = y / count - this->y;
	this->fx += x / weight;
	this->fy += y / weight;
}

static void attraction(boid *this, int x, int y) {
	const int weight = 100000;
	float dx = x - this->x, dy = y - this->y;
	float coeff = sqrt(dx * dx + dy * dy) / weight;
	dx *= coeff;
	dy *= coeff;
	this->fx += dx;
	this->fy += dy;
}

static GList *find_neighbours(boid* boids, int n, int this, int eps) {
	unsigned int squared_eps = eps * eps;
	GList *list = NULL;
	while (--n >= 0) {
		if (boid_distance(this, n) < squared_eps && n != this)
			list = g_list_append(list, GINT_TO_POINTER(n));
	}
	return list;
}

static void calculate_forces(boid* boids, int n, int this, int eps) {
	GList *list = find_neighbours(boids, n, this, eps);
	if (list) {
		separation(boids, this, list);
		alignment(boids, boids + this, list);
		cohesion(boids, boids + this, list);
		g_list_free(list);
	}
}

static void apply_forces(simulation_params *sp, boid* boid) {
	boid->vx += boid->fx * sp->dt;
	boid->vy += boid->fy * sp->dt;
	boid->fx = boid->fy = 0;
	normalize_speed(boid);
	boid->x += boid->vx * sp->dt;
	if (boid->x >= sp->width)
		boid->x -= sp->width;
	else if (boid->x < 0)
		boid->x += sp->width;
	boid->y += boid->vy * sp->dt;
	if (boid->y >= sp->height)
		boid->y -= sp->height;
	else if (boid->y < 0)
		boid->y += sp->height;
}

void simulate(simulation_params *sp) {
	int i = 0;
	if (!g_thread_supported())
		g_thread_init(NULL);
	if (!distance_cache)
		prepare_distance_cache(sp->boids, sp->n);
	reload_distance_cache();
	#pragma omp parallel for
	for (i = 0; i < sp->n; ++i)
		calculate_forces(sp->boids, sp->n, i, sp->eps);
	if (sp->attractor)
		#pragma omp parallel for
		for (i = 0; i < sp->n; ++i)
			attraction(sp->boids + i, sp->attractor->x, sp->attractor->y);
	#pragma omp parallel for
	for (i = 0; i < sp->n; ++i)
		apply_forces(sp, &sp->boids[i]);
}
