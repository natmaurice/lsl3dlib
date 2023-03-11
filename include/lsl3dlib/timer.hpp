#ifndef CCL_TIMER_HPP
#define CCL_TIMER_HPP

#include <cstdint>
#include <ctime>

inline double dtime() {

    struct timespec timespec;
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    return (double)timespec.tv_sec + (double)timespec.tv_nsec / 1e9;
}

#if defined(__i386__) || defined(__x86_64__)

#include <x86intrin.h>

inline double dcycles() {
    return (double)_rdtsc();
}

#endif // __x86_64__



#if defined(__arm__) || defined(__ARM) || defined(__ARM_NEON)
inline double dcycles() {
    return 0;
}

#endif // __arm__

#endif // CCL_TIMER_HPP
