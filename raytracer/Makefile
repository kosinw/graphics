.PHONY: all build clean run

CPPFLAGS := -g -Ofast
LDFLAGS := -pthread

SOURCES ?= platform_linux.cc platform_posix.cc raytracer.cc
OBJECTS := $(SOURCES:%.cc=build/%.o)

all: build run

clean:
	$(RM) -R build/
	mkdir build

build/%.o: %.cc
	g++ $(CPPFLAGS) -c $< -o $@

build/raytracer: $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^

build: build/raytracer

run: build/raytracer
	cd build && ./raytracer && xdg-open result.bmp
