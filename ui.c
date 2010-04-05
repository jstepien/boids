#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <SDL.h>
#include "simulation.h"

#define WIDTH 640
#define HEIGHT 480
#define DEPTH 32

#define NUM 512
#define EPS 10
#define DT 0.1f

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

void setpixel(SDL_Surface *screen, int x, int y, Uint8 r, Uint8 g, Uint8 b) {
	Uint32 *pixmem32;
	Uint32 colour;  
	colour = SDL_MapRGB(screen->format, r, g, b);
	pixmem32 = (Uint32*) screen->pixels + y * screen->pitch / 4 + x;
	*pixmem32 = colour;
}

static const char loading_sprite[][25] = {
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, },
	{1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, },
	{1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, },
	{1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, },
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, },
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, },
};

static void draw_loading(SDL_Surface *scrn) {
	const int x_off = WIDTH / 2 - ARRAY_SIZE(*loading_sprite),
		  y_off = HEIGHT / 2 - ARRAY_SIZE(loading_sprite);
	int x, y;
	if (SDL_MUSTLOCK(scrn) && SDL_LockSurface(scrn) < 0)
		exit(1);
	for (y = 0; y < 2 * ARRAY_SIZE(loading_sprite); y += 2)
		for (x = 0; x < 2 * ARRAY_SIZE(*loading_sprite); x += 2)
			if (loading_sprite[y / 2][x / 2]) {
				setpixel(scrn, x + x_off, y + y_off, 0xff, 0xff, 0xff);
				setpixel(scrn, x + 1 + x_off, y + y_off, 0xff, 0xff, 0xff);
				setpixel(scrn, x + x_off, y + 1 + y_off, 0xff, 0xff, 0xff);
				setpixel(scrn, x + 1 + x_off, y + 1 + y_off, 0xff, 0xff, 0xff);
			}
	if (SDL_MUSTLOCK(scrn))
		SDL_UnlockSurface(scrn);
	SDL_Flip(scrn);
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

void init_video(SDL_Surface **screen) {
	if (SDL_Init(SDL_INIT_VIDEO) < 0 ) {
		*screen = NULL;
		return;
	}
	*screen = SDL_SetVideoMode(WIDTH, HEIGHT, DEPTH, SDL_HWSURFACE);
	if (!*screen)
		SDL_Quit();
}

void print_stats(int probes_per_avg, int time_total) {
	char buffer[256];
	snprintf(buffer, sizeof(buffer), "Simlations/s: %f",
			1e6f * probes_per_avg / time_total);
	SDL_WM_SetCaption(buffer, buffer);
}

void simulation_loop(SDL_Surface *screen, simulation_params *sp) {
	const int probes_per_avg = 100;
	SDL_Event event;
	int keypress = 0, probes = 0, time_total = 0;
	while (!keypress) {
		struct timeval now, then;
		gettimeofday(&then, NULL);
		simulate(sp);
		gettimeofday(&now, NULL);
		DrawScreen(screen, sp->boids, sp->n);
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
			print_stats(probes_per_avg, time_total);
			probes = time_total = 0;
		}
	}
}

int main(int argc, char* argv[]) {
	SDL_Surface *screen;
	simulation_params sp = {WIDTH, HEIGHT, NULL, NUM, EPS, DT, NULL};
	boid *boids;
	assert(sp.dt == DT);
	assert(sp.eps == EPS);
	assert(sp.attractor == NULL);
	assert(sp.n == NUM);
	init_video(&screen);
	draw_loading(screen);
	boids = build_flock(NUM);
	assert(boids);
	sp.boids = boids;
	simulation_loop(screen, &sp);
	free(boids);
	SDL_Quit();
	return 0;
}
