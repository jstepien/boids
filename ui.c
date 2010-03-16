#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <SDL.h>
#include "boid.h"
#include "simulation.h"

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
		simulate(WIDTH, HEIGHT, boids, NUM, EPS, 0.1f);
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
