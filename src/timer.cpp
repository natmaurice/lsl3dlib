#include <lsl3dlib/timer.hpp>


#if defined(__GNUC__) && defined(__amd64__)
#include "x86intrin.h"

uint64_t dcycle() {
    return _rdtsc();    
}

#endif



#if defined(__ARM_ARCH)
#if __ARM_ARCH > 6

uint64_t dcycle(void)
{
    uint32_t pmccntr = 0;
    //asm volatile("mrs %0, pmccntr_el0" : "=r" (pmccntr));
    return (uint64_t)pmccntr;
}

#endif
#endif // __ARM_ARCH
