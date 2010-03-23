extern "C" {
	#include <stdio.h>
	#include <assert.h>
	#include "simulation.h"
	#include "gpu_kernels.h"
}

#define square(x) ((x)*(x))

#define check_cuda_error() {\
	if (cudaError_t e = cudaGetLastError()) { \
		fprintf(stderr, "%s:%i: %s\n", __FILE__, __LINE__, \
				cudaGetErrorString(e)); \
		exit(-1); \
	} }

__global__ void neighbourhood(char *neighbours, float *d_distances, int n,
		int self, int eps_sq) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	if (ix < n && d_distances[self * n + ix] <= eps_sq)
		neighbours[ix] = 1;
}

inline static void run_kernel(char *flags, int n, int self,
		float *d_distances, int eps_sq) {
	int blocksize = 64, flags_bytes = n * sizeof(char);
	dim3 blocks(n / blocksize), threads(blocksize);
	char *d_flags;
	cudaMalloc((void**) &d_flags, flags_bytes);
	check_cuda_error();
	cudaMemset(d_flags, 0, flags_bytes);
	check_cuda_error();
	neighbourhood<<<blocks, threads>>>(d_flags, d_distances, n, self, eps_sq);
	check_cuda_error();
	cudaThreadSynchronize();
	check_cuda_error();
	cudaMemcpy(flags, d_flags, flags_bytes, cudaMemcpyDeviceToHost);
	check_cuda_error();
	cudaFree(d_flags);
	check_cuda_error();
}

int find_neighbours(int *neighbours, int n, int self, float *d_distances,
		int eps) {
	int neighbour_id = 0, i;
	static char *neighbourhood_flags = NULL;
	if (!neighbourhood_flags)
		neighbourhood_flags = (char *) malloc(sizeof(char) * n);
	run_kernel(neighbourhood_flags, n, self, d_distances, eps * eps);
	for (i = 0; i < n; ++i)
		if (neighbourhood_flags[i] && i != self)
			neighbours[neighbour_id++] = i;
	neighbours[neighbour_id] = INT_MAX;
	return neighbour_id;
}
