#ifndef LSL3DLIB_PERF_HELPER_HPP
#define LSL3DLIB_PERF_HELPER_HPP


#if LSL3DLIB_USE_PERF

#include <perf-utils/perf-utils.h>

constexpr size_t PERFFORMAT_MAX_LENGTH = 4096;
constexpr int PERF_HELPER_MAX_EVENTS = 4;

extern int g_perf_parent_fd;

static const int g_perf_helper_config[] = {
    PERF_COUNT_HW_INSTRUCTIONS,
    PERF_COUNT_HW_BRANCH_MISSES,
    PERF_COUNT_HW_BRANCH_INSTRUCTIONS,
    -1
};

inline
int perf_find(int eventid) {
    for (int i = 0; i < PERF_HELPER_MAX_EVENTS; i++) {
	if (g_perf_helper_config[i] == eventid) {
	    return i;
	}
    }
    return -1;
}

int perf_helper_create(int &group_fd, int *fds, int *ids);
int perf_helper_create();
void perf_helper_destroy(int group_fd, int *fds, size_t len);
void perf_helper_destroy();

static inline
void read_counters(int fd, char* raw, size_t len) {
    read(fd, raw, len);
}

static inline
void read_counters(char* raw, size_t len) {
    read_counters(g_perf_parent_fd, raw, len);
}

int perf_helper_get_id(int eventcode);
int perf_helper_find_config(int eventid);

#endif // LSL3DLIB_USE_PERF

#endif // LSL3DLIB_PERF_HELPER_HPP
