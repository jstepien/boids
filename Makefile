DEPS=sdl glib-2.0
CFLAGS+=`pkg-config --cflags ${DEPS}`
LDFLAGS+=`pkg-config --libs ${DEPS}`
OBJECTS=ui.o boid.o
.PHONY: all
all: cpu
cpu: ${OBJECTS}
	${CC} $^ -o $@ ${LDFLAGS}
clean:
	-rm -f cpu ${OBJECTS}
