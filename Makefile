DEPS=sdl glib-2.0 gthread-2.0
CFLAGS+=`pkg-config --cflags ${DEPS}`
LDFLAGS+=`pkg-config --libs ${DEPS}`
CPUOBJECTS=ui.o cpu_simulation.o
.PHONY: all
all: cpu
cpu: ${CPUOBJECTS}
	${CC} $^ -o $@ ${LDFLAGS}
clean:
	-rm -f cpu ${CPUOBJECTS}
