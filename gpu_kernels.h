#ifndef GPU_KERNELS_H_
#define GPU_KERNELS_H_

int find_neighbours(int *neighbours, int n, int self,
		float *d_distances, int eps);
void reload_distance_cache(float *d_distance_cache, float *cache, boid *boids,
		int n);

#endif /* GPU_KERNELS_H_ */
