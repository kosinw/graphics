/*
   platform_linux.cc - Platform specific code for linux.
*/

#include <unistd.h>
#include <time.h>

#include "platform.hh"
#include "raytracer.hh"

u64
GetTimeMilliseconds()
{
	timespec time;
	clock_gettime(CLOCK_MONOTONIC, &time);

	u64 time_ms = time.tv_sec * 1000 + time.tv_nsec / 1000000;

	return time_ms;
}

u64
LockedAdd(volatile u64 *value, u64 addend)
{
	return __sync_fetch_and_add(value, addend);
}
