#include <math.h>
#include <assert.h>
#include <glib.h>
#include "boid.h"
#include "simulation.h"

static void separation(boid *boids, int this, GList *others) {
	GList *current = g_list_first(others);
	float x = 0, y = 0;
	int count = 0;
	const int weight = 50;
	do {
		int index = GPOINTER_TO_INT(current->data);
		float distance = boid_real_distance(this, index) + 0.01f;
		assert(distance > 0);
		x += (boids[this].x - boids[index].x) / distance;
		y += (boids[this].y - boids[index].y) / distance;
		++count;
	} while (current = g_list_next(current));
	boids[this].fx = x / count / weight;
	boids[this].fy = y / count / weight;
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

static GList *find_neighbours(boid* boids, int n, int this, int eps) {
	GList *list = NULL;
	while (--n >= 0) {
		if (n == this)
			continue;
		if (boid_distance(this, n) < eps * eps)
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
	boid_normalize_speed(boid);
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
	boid_reload_distance_cache();
	for (i = 0; i < sp->n; ++i)
		calculate_forces(sp->boids, sp->n, i, sp->eps);
	for (i = 0; i < sp->n; ++i)
		apply_forces(sp, &sp->boids[i]);
}
