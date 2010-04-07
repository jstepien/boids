DEPS=sdl glib-2.0 gthread-2.0
CFLAGS+=`pkg-config --cflags ${DEPS}`
LDFLAGS+=`pkg-config --libs ${DEPS}`
GPULDFLAGS+=${LDFLAGS} -lcudart -lcudpp
CPUOBJECTS=ui.o cpu_simulation.o
GPUOBJECTS=ui.o gpu_simulation.o
TARGETS=cpu gpu
NVCC?=nvcc
.SUFFIXES: .cu
.PHONY: all clean
all: ${TARGETS}
cpu: ${CPUOBJECTS}
	${CC} $^ -o $@ ${LDFLAGS}
gpu: ${GPUOBJECTS}
	${CC} $^ -o $@ ${GPULDFLAGS}
clean:
	-rm -f ${TARGETS} ${CPUOBJECTS} ${GPUOBJECTS}
.cu.o:
	${NVCC} ${NVCCFLAGS} -c $^
