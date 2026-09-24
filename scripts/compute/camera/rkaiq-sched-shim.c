/*
 * rkaiq-sched-shim.c: LD_PRELOAD shim for Rockchip's rkaiq_3A_server on Armbian.
 *
 * Problem: librkaiq creates its ISP statistics poll thread ("xc:isp_3a_stats_poll") with an
 * explicit SCHED_RR priority 20. The Armbian vendor kernel has CONFIG_RT_GROUP_SCHED=y and
 * systemd enables the cgroup v2 cpu controller, so any process outside the root cgroup (every
 * systemd service and every login session) gets EPERM from sched_setscheduler, glibc's
 * pthread_create fails, and the engine silently runs without statistics: exposure stays at the
 * 3 ms start value forever and the picture is dark.
 *
 * Fix: make pthread_attr_setinheritsched() ignore PTHREAD_EXPLICIT_SCHED so new threads inherit
 * the caller's normal scheduling class. The engine keeps up with 21 fps at normal priority.
 * Set RKAIQ_ALLOW_RT=1 to pass the call through unchanged (for a kernel where RT is allowed).
 *
 * Build on the board:  gcc -O2 -shared -fPIC -o rkaiq-sched-shim.so rkaiq-sched-shim.c -ldl
 * Use:                 LD_PRELOAD=/usr/local/lib/rkaiq-sched-shim.so rkaiq_3A_server
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdlib.h>

int pthread_attr_setinheritsched(pthread_attr_t *attr, int inheritsched)
{
    static int (*real)(pthread_attr_t *, int);
    if (!real)
        real = dlsym(RTLD_NEXT, "pthread_attr_setinheritsched");
    if (inheritsched == PTHREAD_EXPLICIT_SCHED && !getenv("RKAIQ_ALLOW_RT"))
        return 0; /* pretend it worked; the thread inherits SCHED_OTHER */
    return real(attr, inheritsched);
}
