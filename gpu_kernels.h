#ifndef GPU_KERNELS_H_
#define GPU_KERNELS_H_

void reload_distance_cache(float *d_distance_cache, float *cache, boid *boids,
		int n);
void calculate_all_forces(boid* boids, int n, int eps, float *d_distance_cache);
void apply_all_forces(boid* d_boids, int n, float dt, int width, int height);

#endif /* GPU_KERNELS_H_ */
