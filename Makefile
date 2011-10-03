DEPS=sdl
CFLAGS+=$(shell pkg-config --cflags $(DEPS))
LDFLAGS+=$(shell pkg-config --libs $(DEPS))
GPULDFLAGS+=${LDFLAGS} -lcudart -lcudpp
CPUOBJECTS=ui.o cpu_simulation.o
GPUOBJECTS=ui.o gpu_simulation.o
TARGETS=cpu gpu boids.js
EMSCRIPTEN?=emscripten
LLVM-DIS?=llvm-dis
CLANG?=clang
NVCC?=nvcc
.SUFFIXES: .cu .ll .js .llvmo
.PHONY: all clean
all: ${TARGETS}
cpu: ${CPUOBJECTS}
	${CC} $^ -o $@ ${LDFLAGS}
gpu: ${GPUOBJECTS}
	${CC} $^ -o $@ ${GPULDFLAGS}
clean:
	-rm -f ${TARGETS} ${CPUOBJECTS} ${GPUOBJECTS} *.ll *.llvmo
.cu.o:
	${NVCC} ${NVCCFLAGS} -c $^
boids.js: js.ll
	$(EMSCRIPTEN) $^ > $@ || rm $@
.c.llvmo:
	$(CLANG) -O3 -emit-llvm -c -g $< -o $@
.llvmo.ll:
	$(LLVM-DIS) -show-annotations $< -o $@
js.llvmo: js.c cpu_simulation.c simulation.h
