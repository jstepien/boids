#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <SDL.h>
#include "simulation.h"

#define WIDTH 640
#define HEIGHT 480
#define DEPTH 32

#define EPS 10
#define DT 0.1f

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

static const char loading_sprite[][25] = {
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, },
	{1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, },
	{1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, },
	{1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, },
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, },
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, },
};

static void set_pixel(SDL_Surface *screen, int x, int y,
		Uint8 r, Uint8 g, Uint8 b) {
	Uint32 *pixmem32 = (Uint32*) screen->pixels + y * screen->pitch / 4 + x;
	*pixmem32 = SDL_MapRGB(screen->format, r, g, b);
}

static void draw_loading(SDL_Surface *scrn) {
	const int x_off = WIDTH / 2 - ARRAY_SIZE(*loading_sprite),
		  y_off = HEIGHT / 2 - ARRAY_SIZE(loading_sprite);
	int x, y;
	if (SDL_MUSTLOCK(scrn) && SDL_LockSurface(scrn) < 0)
		exit(1);
	for (y = 0; y < 2 * ARRAY_SIZE(loading_sprite); y += 2)
		for (x = 0; x < 2 * ARRAY_SIZE(*loading_sprite); x += 2)
			if (loading_sprite[y / 2][x / 2]) {
				set_pixel(scrn, x + x_off, y + y_off, 0xff, 0xff, 0xff);
				set_pixel(scrn, x + 1 + x_off, y + y_off, 0xff, 0xff, 0xff);
				set_pixel(scrn, x + x_off, y + 1 + y_off, 0xff, 0xff, 0xff);
				set_pixel(scrn, x + 1 + x_off, y + 1 + y_off, 0xff, 0xff, 0xff);
			}
	if (SDL_MUSTLOCK(scrn))
		SDL_UnlockSurface(scrn);
	SDL_Flip(scrn);
}

static void draw_boids(SDL_Surface* screen, boid* boids, int n) {
	if (SDL_MUSTLOCK(screen) && SDL_LockSurface(screen) < 0)
		exit(1);
	memset(screen->pixels, 0, HEIGHT * screen->pitch);
	while (--n >= 0)
		set_pixel(screen, boids[n].x, boids[n].y, 0xff, 0xff, 0xff);
	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_Flip(screen); 
}

static boid* build_flock(int n) {
	int i, j, cur = 0;
	float sqrt_n = ceilf(sqrtf(n)), dx = WIDTH / (sqrt_n + 1),
		  dy = HEIGHT / (sqrt_n + 1);
	boid *boids = calloc(n, sizeof(boid));
	if (!boids)
		return NULL;
	for (i = 0; i < sqrt_n; ++i)
		for (j = 0; j < sqrt_n && cur < n; ++j) {
			boids[cur].vx = sinf(17 * cur);
			boids[cur].vy = cosf(3 * cur);
			boids[cur].y = (i + 1) * dy;
			boids[cur].x = (j + 1) * dx;
			++cur;
		}
	return boids;
}

static void init_video(SDL_Surface **screen, int fullscreen) {
	if (SDL_Init(SDL_INIT_VIDEO) < 0 ) {
		*screen = NULL;
		return;
	}
	int flags = SDL_HWSURFACE;
	if (fullscreen)
		flags |= SDL_FULLSCREEN;
	*screen = SDL_SetVideoMode(WIDTH, HEIGHT, DEPTH, flags);
	if (!*screen)
		SDL_Quit();
}

static void print_stats(int probes_per_avg, int time_total) {
	char buffer[ARRAY_SIZE("Simlations/s: 12345678901234567890\0")];
	snprintf(buffer, sizeof(buffer), "Simlations/s: %f",
			1e6f * probes_per_avg / time_total);
	SDL_WM_SetCaption(buffer, buffer);
}

static void simulation_loop(SDL_Surface *screen, simulation_params *sp) {
	const int probes_per_avg = 10;
	SDL_Event event;
	int keypress = 0, probes = 0, time_total = 0;
	while (!keypress) {
		struct timeval now, then;
		gettimeofday(&then, NULL);
		simulate(sp);
		gettimeofday(&now, NULL);
		draw_boids(screen, sp->boids, sp->n);
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

static void usage(const char* name) {
	fprintf(stderr, "Usage: %s [-f] n\n", name);
	exit(1);
}

int main(int argc, char* argv[]) {
	SDL_Surface *screen;
	boid *boids;
	int fullscreen = 0, argptr = 1;
	simulation_params sp = {WIDTH, HEIGHT, NULL, -1, EPS, DT, NULL};
	if (argc < 2)
		usage(*argv);
	if (0 == strcmp("-f", argv[1])) {
		fullscreen = 1;
		++argptr;
	}
	sp.n = atoi(argv[argptr]);
	if (sp.n <= 0)
		usage(*argv);
	init_video(&screen, fullscreen);
	draw_loading(screen);
	boids = build_flock(sp.n);
	assert(boids);
	sp.boids = boids;
	simulation_loop(screen, &sp);
	free(boids);
	SDL_Quit();
	return 0;
}
