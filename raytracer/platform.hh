/*
   platform.hh - Platform-specific defines for raytracer.cc
*/

#ifndef PLATFORM_HH
#define PLATFORM_HH

#include "raytracer.hh"

#define THREAD_PROC_RET void*

struct thread;

u64 GetTimeMilliseconds();
u64 LockedAdd(volatile u64 *, u64);

thread* CreateTaskThread(THREAD_PROC_RET (*dummy)(void*), void*);
void CloseThreadHandle(thread*);

#endif /* PLATFORM_HH */
