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

__device__ inline void swap(int &a, int &b) {
	int tmp = a;
	a = b;
	b = tmp;
}

__global__ static void bitonicSort(int n, char *keys, int *values) {
	extern __shared__ int shared[];
	const unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

#define key(tid) shared[(tid)]
#define value(tid) shared[(tid) + n]

	key(tid) = keys[tid];
	value(tid) = tid;
	__syncthreads();

	for (unsigned int k = 2; k <= n; k *= 2) {
		for (unsigned int j = k / 2; j > 0; j /= 2) {
			unsigned int ixj = tid ^ j;
			if (ixj > tid) {
				if ((tid & k) == 0) {
					if (key(tid) < key(ixj)) {
						swap(value(tid), value(ixj));
						swap(key(tid), key(ixj));
					}
				} else {
					if (key(tid) > key(ixj)) {
						swap(value(tid), value(ixj));
						swap(key(tid), key(ixj));
					}
				}
			}
			__syncthreads();
		}
	}
	__syncthreads();
	if (1 == key(tid) && 0 == key(tid + 1))
		value(tid + 1) = INT_MAX;
	values[tid] = value(tid);
#undef key
#undef value
}

__global__ static void neighbourhood(char *neighbours, float *d_distances, int n,
		int self, int eps_sq) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	if (ix < n && d_distances[self * n + ix] <= eps_sq)
		neighbours[ix] = 1;
}

inline static void run_kernel(int *neighbours, int n, int self,
		float *d_distances, int eps_sq) {
	static char *neighbourhood_flags = NULL;
	int blocksize = 64, flags_bytes = n * sizeof(char),
		neighbours_bytes = n * sizeof(int), *d_neighbours;
	dim3 blocks(n / blocksize), threads(blocksize);
	char *d_flags;

	if (!neighbourhood_flags)
		neighbourhood_flags = (char *) malloc(sizeof(char) * n);
	cudaMalloc((void**) &d_flags, flags_bytes);
	cudaMalloc((void**) &d_neighbours, neighbours_bytes);
	cudaMemset(d_flags, 0, flags_bytes);

	neighbourhood<<<blocks, threads>>>(d_flags, d_distances, n, self, eps_sq);
	check_cuda_error();
	bitonicSort<<<1, n, sizeof(int) * n * 2>>>(n, d_flags, d_neighbours);
	check_cuda_error();
	cudaThreadSynchronize();
	check_cuda_error();

	cudaMemcpy(neighbours, d_neighbours, neighbours_bytes, cudaMemcpyDeviceToHost);
	cudaFree(d_flags);
	cudaFree(d_neighbours);
	//fprintf(stderr, "Done\n");
}

int find_neighbours(int *neighbours, int n, int self, float *d_distances,
		int eps) {
	int neighbour_id = 0;
	run_kernel(neighbours, n, self, d_distances, eps * eps);
	return neighbour_id;
}
