#include <lsl3dlib/perf-helper.hpp>
#include <cstring>
#include <iostream>

#if LSL3DLIB_USE_PERF

int g_perf_parent_fd;
static int g_fds[PERF_HELPER_MAX_EVENTS];
static int g_ids[PERF_HELPER_MAX_EVENTS];


int perf_helper_create(int &group_fd, int *fds, int *ids) {

    struct perf_event_attr pe[PERF_HELPER_MAX_EVENTS];

    group_fd = 0;
    
    memset(&pe[0], 0, sizeof(pe));
    
    for (size_t i = 0; i < PERF_HELPER_MAX_EVENTS; i++) {

	if (g_perf_helper_config[i] < 0) {
	    continue;
	}
	
	pe[i].type = PERF_TYPE_HARDWARE;
	pe[i].size = sizeof(pe[0]);
	pe[i].config = g_perf_helper_config[i];
	pe[i].disabled = 1;
	pe[i].exclude_kernel = 1;
	pe[i].exclude_hv = 1;
	pe[i].read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;
    }

    if (PERF_HELPER_MAX_EVENTS > 0) {
	fds[0] = group_fd = perf_event_open(&pe[0], 0, -1, -1, 0);
	if (fds[0] == -1) {
	    std::cerr << "Error opening leader " << pe[0].config << "\n";
	    return -1;
	}

	ioctl(fds[0], PERF_EVENT_IOC_ID, &ids[0]);
	std::cout << "ids[0] = " << ids[0] << "\n";

    }

    for (size_t i = 1; i < PERF_HELPER_MAX_EVENTS; i++) {
	if (pe[i].config >= 0) {
	    fds[i] = perf_event_open(&pe[1], 0, -1, group_fd, 0);
	    if (fds[i] == -1) {
		std::cerr << "Error opening event " << pe[i].config << "\n";
		continue;
	    }
	    
	    ioctl(fds[i], PERF_EVENT_IOC_ID, &ids[i]);
	    std::cout << "ids[" << i << "] = " << ids[i] << "\n";
	}
    }

    perf_event_enable_all(g_perf_parent_fd);
    return 0;
}

int perf_helper_create() {

    memset(g_fds, 0, sizeof(g_fds));
    memset(g_fds, 0, sizeof(g_ids));
    
    return perf_helper_create(g_perf_parent_fd, g_fds, g_ids);
}

void perf_helper_destroy(int groupfd, int* fds, size_t len) {
    perf_event_disable_all(g_perf_parent_fd);
    
    close(groupfd);
    for (size_t i = 0; i < len; i++) {
	close(fds[i]);
    }
}

void perf_helper_destroy() {
    perf_helper_destroy(g_perf_parent_fd, g_fds, PERF_HELPER_MAX_EVENTS);
}

int perf_helper_get_id(int eventcode) {
    if (eventcode >= PERF_HELPER_MAX_EVENTS) {
	return -1;
    }
    return g_ids[eventcode];
}

int perf_helper_find_config(int eventid) {
    
    for (int i = 0; i < PERF_HELPER_MAX_EVENTS; i++) {
	int id = g_ids[i];
	if (id == eventid) {
	    return i;
	}
    }
    return -1;
}

#endif // LSL3DLIB_USE_PERF
