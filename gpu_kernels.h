#ifndef GPU_KERNELS_H_
#define GPU_KERNELS_H_

int find_neighbours(int *neighbours, int n, int self,
		float *d_distances, int eps);

#endif /* GPU_KERNELS_H_ */
