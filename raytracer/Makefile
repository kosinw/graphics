.PHONY: all build clean run

all: clean build run

clean:
	$(RM) -R build/
	mkdir build

build/raytracer: raytracer.cc
	g++ -g -Ofast -O2 -pthread -o build/raytracer platform_linux.cc platform_posix.cc raytracer.cc

debug: raytracer.cc
	g++ -g -pthread -o build/raytracer-debug platform_linux.cc platform_posix.cc raytracer.cc

build: build/raytracer

run: build/raytracer
	cd build && ./raytracer && xdg-open result.bmp
