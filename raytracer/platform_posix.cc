/*
   platform_posix.cc - Platform specific code for POSIX.
*/

#include "platform.hh"

#include <pthread.h>
#include <stdlib.h>

struct thread
{
	pthread_t handle;
};

thread*
CreateTaskThread(THREAD_PROC_RET (*return_proc)(void*), void* args)
{
	pthread_t t;
	pthread_create(&t, NULL, return_proc, args);

	thread* result = (thread *)calloc(sizeof(thread), 1);
	result->handle = t;

	return result;
}

void
CloseThreadHandle(thread* t)
{
	pthread_cancel(t->handle);
}
