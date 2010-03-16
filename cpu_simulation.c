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
		float distance = boid_real_distance(boids + this, boids + index) +
			0.01f;
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

static void calculate_forces(boid* boids, int n, int this, int eps) {
	GList *list = NULL;
	while (--n >= 0) {
		if (n == this)
			continue;
		float distance = boid_distance(boids + this, boids + n);
		if (distance < eps * eps)
			list = g_list_append(list, GINT_TO_POINTER(n));
	}
	if (list) {
		separation(boids, this, list);
		alignment(boids, boids + this, list);
		cohesion(boids, boids + this, list);
		g_list_free(list);
	}
}

void simulate(int width, int height, boid* boids, int n, int eps, float dt) {
	int i = 0;
	for (i = 0; i < n; ++i)
		calculate_forces(boids, n, i, eps);
	for (i = 0; i < n; ++i) {
		boids[i].vx += boids[i].fx * dt;
		boids[i].vy += boids[i].fy * dt;
		boids[i].fx = boids[i].fy = 0;
		boid_normalize_speed(boids + i);
		boids[i].x += boids[i].vx * dt;
		if (boids[i].x >= width)
			boids[i].x -= width;
		else if (boids[i].x < 0)
			boids[i].x += width;
		boids[i].y += boids[i].vy * dt;
		if (boids[i].y >= height)
			boids[i].y -= height;
		else if (boids[i].y < 0)
			boids[i].y += height;
	}
}
