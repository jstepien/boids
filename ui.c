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

static void draw_attractor(SDL_Surface* screen, pair *attr) {
	const char r = 0xff, g = 0x90, b = 0, size = 4;
	int i;
	for (i = 1; i < size; ++i) {
		set_pixel(screen, attr->x + i, attr->y, r, g, b);
		set_pixel(screen, attr->x, attr->y + i, r, g, b);
		set_pixel(screen, attr->x, attr->y - i, r, g, b);
		set_pixel(screen, attr->x - i, attr->y, r, g, b);
	}
	set_pixel(screen, attr->x, attr->y, r, g, b);
}

static void draw_boids(SDL_Surface* screen, simulation_params *sp) {
	int i, j;
	if (SDL_MUSTLOCK(screen) && SDL_LockSurface(screen) < 0)
		exit(1);
	memset(screen->pixels, 0, HEIGHT * screen->pitch);
	for (i = 0; i < sp->height; ++i)
		for (j = 0; j < sp->width; ++j) {
			char value = sp->intensity[i * sp->width + j];
			set_pixel(screen, j, i, value, value, value);
		}
	if (sp->attractor)
		draw_attractor(screen, sp->attractor);
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
			boids[cur].vx = sinf((1559 * cur) ^ 50969);
			boids[cur].vy = cosf((1567 * cur) ^ 51853);
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

static void set_attractor(simulation_params *sp, int x, int y) {
	assert(sp);
	if (!sp->attractor)
		sp->attractor = malloc(sizeof(*sp->attractor));
	sp->attractor->x = x;
	sp->attractor->y = y;
}

static void unset_attractor(simulation_params *sp) {
	assert(sp);
	if (!sp->attractor)
		return;
	free(sp->attractor);
	sp->attractor = NULL;
}

static void handle_events(simulation_params *sp, int *quit) {
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		switch (event.type) {
			case SDL_QUIT:
				*quit = 1;
				break;
			case SDL_KEYDOWN:
				*quit = 1;
				break;
			case SDL_MOUSEBUTTONUP:
				if (SDL_BUTTON_LEFT == event.button.button)
					set_attractor(sp, event.button.x, event.button.y);
				else if (SDL_BUTTON_RIGHT == event.button.button)
					unset_attractor(sp);
				break;
		}
	}
}

static void simulation_loop(SDL_Surface *screen, simulation_params *sp) {
	const int probes_per_avg = 10;
	int quit = 0, probes = 0, time_total = 0;
	while (!quit) {
		struct timeval now, then;
		gettimeofday(&then, NULL);
		simulate(sp);
		gettimeofday(&now, NULL);
		draw_boids(screen, sp);
		handle_events(sp, &quit);
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
	char intensity[HEIGHT * WIDTH];
	simulation_params sp = {WIDTH, HEIGHT, NULL, -1, EPS, DT, intensity, NULL};
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
