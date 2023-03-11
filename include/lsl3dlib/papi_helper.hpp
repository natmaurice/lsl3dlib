#ifndef CCL_BENCH_PAPI_HELPER_HPP
#define CCL_BENCH_PAPI_HELPER_HPP

#ifndef CCL_ENABLE_PAPI
#define CCL_ENABLE_PAPI true
#endif // CCL_ENABLE_PAPI


/* PAPI 5 seems to have some incompatibility issues with some glibc and an error
 * might be throw during compilation indicating that "caddr_t" is not known.
 * This should patch the issue by forcing the definition of caddr_t if not 
 * already present.
 * __USE_BSD is deprecated in newer version of the libc and has been replaced by
 *  __USE_MISC. We therefore check for both.
 */
#if !defined(__USE_BSD) && !defined(__USE_MISC)
typedef char* caddr_t;
#endif

#if CCL_ENABLE_PAPI
#include <papi.h>
#else
#endif // CCL_ENABLE_PAPI

#include <array>
#include <iostream>
#include <map>

constexpr int PAPIH_NUM_EVENTS = 2;

int papih_init();
void papih_exit();
int papih_configure();

int papih_read(long long* measurements, int measures_count);
inline double get_l1d_miss();
inline double get_l1i_miss();
inline double get_branch_misprediction();
inline double get_stalled_cycles();

extern int g_eventset;

constexpr int MAX_PAPI_EVENTS = 16;

// Mapping Event -> Slot (at most 4 slots)
// If -1 then event won't be measured
constexpr int PAPIH_L1DM = 0;
constexpr int PAPIH_L2DM = -1;
constexpr int PAPIH_L3DM = -1;
constexpr int PAPIH_L1IM = -1;
constexpr int PAPIH_STAL = -1;
constexpr int PAPIH_MISP = 1;
constexpr int PAPIH_NIIS = -1;
constexpr int PAPIH_NICT = -1;
constexpr int PAPIH_TCYC = 2;
constexpr int PAPIH_WSTAL = -1;
constexpr int PAPIH_TINS = -1;
constexpr int PAPIH_TBR = 3;

constexpr bool ENABLE_PAPI = CCL_ENABLE_PAPI;

#if CCL_ENABLE_PAPI
// Associative mapping: used to setup papi event sets
constexpr size_t PAPI_EVENT_CNT = 12;
static const std::array<std::pair<int, int>, PAPI_EVENT_CNT> g_Events {
    std::pair<int, int>{PAPI_L1_DCM, PAPIH_L1DM},
    std::pair<int, int>{PAPI_L2_DCM, PAPIH_L2DM},
    std::pair<int, int>{PAPI_L3_TCM, PAPIH_L3DM},
    std::pair<int, int>{PAPI_L1_ICM, PAPIH_L1IM},
    std::pair<int, int>{PAPI_RES_STL, PAPIH_STAL},
    std::pair<int, int>{PAPI_BR_MSP, PAPIH_MISP},
    std::pair<int, int>{PAPI_STL_ICY, PAPIH_NIIS},
    std::pair<int, int>{PAPI_STL_CCY, PAPIH_NICT},
    std::pair<int, int>{PAPI_TOT_CYC, PAPIH_TCYC},
    std::pair<int, int>{PAPI_MEM_WCY, PAPIH_WSTAL},
    std::pair<int, int>{PAPI_TOT_INS, PAPIH_TINS},
    std::pair<int, int>{PAPI_BR_CN, PAPIH_TBR},
};

static const std::map<int, std::string> g_EventAbbrv {
    std::pair<int, std::string>{PAPI_L1_DCM, "l1dm"},
    std::pair<int, std::string>{PAPI_L2_DCM, "l2dm"},
    std::pair<int, std::string>{PAPI_L3_TCM, "l3m"},
    std::pair<int, std::string>{PAPI_L1_ICM, "l1im"},
    std::pair<int, std::string>{PAPI_RES_STL, "stal"},
    std::pair<int, std::string>{PAPI_BR_MSP, "misp"},
    std::pair<int, std::string>{PAPI_STL_ICY, "niis"},
    std::pair<int, std::string>{PAPI_STL_CCY, "nict"},
    std::pair<int, std::string>{PAPI_TOT_CYC, "cycles"},
    std::pair<int, std::string>{PAPI_MEM_WCY, "wstal"},
    std::pair<int, std::string>{PAPI_MEM_WCY, "instr"},
    std::pair<int, std::string>{PAPI_BR_CN, "br"}
};
#endif // CCL_ENABLE_PAPI

inline
std::string find_event_abbrv(int eventid) {
#if CCL_ENABLE_PAPI
    auto it = g_EventAbbrv.find(eventid);
    if (it != g_EventAbbrv.end()) {
	return it->second;
    }
#endif // CCL_ENABLE_PAPI
    return "unknown";

}

inline void read_counters(long long measurements[MAX_PAPI_EVENTS]) {
#if CCL_ENABLE_PAPI
        PAPI_read(g_eventset, measurements);
#endif // CCL_ENABLE_PAPI
}

inline double get_from_measurements(long long measurements[MAX_PAPI_EVENTS], int event_code) {
    double res = 0;
    if (event_code >= 0 && event_code < 16) {
	return measurements[event_code];
    }
    return res;
}

#endif // CCL_BENCH_PAPI_HELPER_HPP
