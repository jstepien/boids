#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <SDL.h>
#include <glib.h>
#include "boid.h"

#define WIDTH 640
#define HEIGHT 480
#define DEPTH 32

#define NUM 512
#define EPS 20

void setpixel(SDL_Surface *screen, int x, int y, Uint8 r, Uint8 g, Uint8 b) {
	Uint32 *pixmem32;
	Uint32 colour;  
	colour = SDL_MapRGB(screen->format, r, g, b);
	pixmem32 = (Uint32*) screen->pixels + y * screen->pitch / 4 + x;
	*pixmem32 = colour;
}


void DrawScreen(SDL_Surface* screen, boid* boids, int n) { 
	if (SDL_MUSTLOCK(screen) && SDL_LockSurface(screen) < 0)
		exit(1);
	memset(screen->pixels, 0, HEIGHT * screen->pitch);
	while (--n >= 0)
		setpixel(screen, boids[n].x, boids[n].y, 0xff, 0xff, 0xff);
	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_Flip(screen); 
}

void separation(boid *boids, int this, GList *others) {
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

void alignment(boid *boids, boid *this, GList *others) {
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

void cohesion(boid *boids, boid *this, GList *others) {
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

void calculate_forces(boid* boids, int n, int this) {
	GList *list = NULL;
	while (--n >= 0) {
		if (n == this)
			continue;
		float distance = boid_distance(boids + this, boids + n);
		if (distance < EPS * EPS)
			list = g_list_append(list, GINT_TO_POINTER(n));
	}
	if (list) {
		separation(boids, this, list);
		alignment(boids, boids + this, list);
		cohesion(boids, boids + this, list);
		g_list_free(list);
	}
}

void simulate(boid* boids, int n, float dt) {
	int i = 0;
	for (i = 0; i < n; ++i)
		calculate_forces(boids, n, i);
	for (i = 0; i < n; ++i) {
		boids[i].vx += boids[i].fx * dt;
		boids[i].vy += boids[i].fy * dt;
		boids[i].fx = boids[i].fy = 0;
		boid_normalize_speed(boids + i);
		boids[i].x += boids[i].vx * dt;
		if (boids[i].x >= WIDTH)
			boids[i].x -= WIDTH;
		else if (boids[i].x < 0)
			boids[i].x += WIDTH;
		boids[i].y += boids[i].vy * dt;
		if (boids[i].y >= HEIGHT)
			boids[i].y -= HEIGHT;
		else if (boids[i].y < 0)
			boids[i].y += HEIGHT;
	}
}

boid* build_flock(int n) {
	boid *boids = calloc(n, sizeof(boid));
	int sum = n;
	if (!boids)
		return NULL;
	while (--n >= sum / 2) {
		boids[n].vx = 2;
		boids[n].vy = 0;
		boids[n].y = -50 + n;
		boids[n].x = sinf(boids[n].y / 10) * 40 + 50;
	}
	while (--n >= 0) {
		boids[n].vy = 3;
		boids[n].vx = 0;
		boids[n].x = 50 + n;
		boids[n].y = sinf(boids[n].x / 10) * 40 + 50;
	}
	return boids;
}

int main(int argc, char* argv[]) {
	SDL_Surface *screen;
	SDL_Event event;
	int keypress = 0, probes = 0, time_total = 0;
	const int probes_per_avg = 100;
	char buffer[256];
	boid *boids;
	if (SDL_Init(SDL_INIT_VIDEO) < 0 )
		return 1;
	screen = SDL_SetVideoMode(WIDTH, HEIGHT, DEPTH, SDL_HWSURFACE);
	if (!screen) {
		SDL_Quit();
		return 1;
	}
	boids = build_flock(NUM);
	assert(boids);
	while (!keypress) {
		struct timeval now, then;
		gettimeofday(&then, NULL);
		simulate(boids, NUM, 0.1f);
		gettimeofday(&now, NULL);
		DrawScreen(screen, boids, NUM);
		while (SDL_PollEvent(&event)) {      
			switch (event.type) {
				case SDL_QUIT:
					keypress = 1;
					break;
				case SDL_KEYDOWN:
					keypress = 1;
					break;
			}
		}
		if (now.tv_usec - then.tv_usec > 0)
			time_total += now.tv_usec - then.tv_usec;
		else
			--probes;
		if (++probes == probes_per_avg) {
			snprintf(buffer, sizeof(buffer), "Simlations/s: %f",
					1e6f * probes_per_avg / time_total);
			SDL_WM_SetCaption(buffer, buffer);
			probes = time_total = 0;
		}
	}
	free(boids);
	SDL_Quit();
	return 0;
}
