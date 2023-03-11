#include <lsl3dlib/papi_helper.hpp>

#include <iostream>
#include <algorithm>

int g_eventset = 0;

int papih_init() {
#if CCL_ENABLE_PAPI
    int ret = PAPI_library_init(PAPI_VER_CURRENT);
    if (ret != PAPI_VER_CURRENT) {
	std::cerr << "Could not load PAPI Library" << std::endl;
	    return -1;
    }   
#endif // CCL_ENABLE_PAPI    
    return 0;
}

void papih_exit() {
#if CCL_ENABLE_PAPI
    PAPI_shutdown();
#endif // CCL_ENABLE_PAPI
}

int add_papi_event(int &eventset, int event_code) {
    #if CCL_ENABLE_PAPI
    char eventname[PAPI_MAX_STR_LEN];    
    int ret = PAPI_add_event(eventset, event_code);
    if (ret != PAPI_OK) {
	switch (ret) {
	case PAPI_OK:
	    break;
	case PAPI_EINVAL:
	    PAPI_perror("Invalid event");	
	    break;
	case PAPI_ECNFLCT:
	    PAPI_perror("Event conflict");
	    break;
	case PAPI_ENOEVNT:
	    PAPI_perror("Unavailable event");
	    break;
	default:
	    PAPI_perror("Can't add event");
	    break;
	}
	
	// Check if event exists
	ret = PAPI_event_code_to_name(event_code, eventname);
	if (ret != PAPI_OK) {
	    PAPI_perror("Can't add unknown event");
	} else {
	    PAPI_perror(eventname);
	}
	return -1;

    }
#endif // CCL_ENABLE_PAPI
    return 0;
}

int add_papi_events(int& eventset) {
#if CCL_ENABLE_PAPI
    auto Events = g_Events;
    std::sort(Events.begin(), Events.end(), [](auto a, auto b) {
	return a.second < b.second;
    });
    
    int event_code, slot;
    for (const auto& it: Events) {
	event_code = it.first;
	slot = it.second;
	
	if (slot >= 0 && slot < (int)MAX_PAPI_EVENTS){
	    if (add_papi_event(eventset, event_code)) {
		std::cerr << "Could not add event\n";
		return -1;
	    }
	}
    }
#endif // CCL_ENABLE_PAPI    
    return 0;
}

int papih_configure() {
#if CCL_ENABLE_PAPI
    int eventset = PAPI_NULL;
    int ret = 0;

    if (!ENABLE_PAPI) {
	return 0;
    }
    
    ret = PAPI_create_eventset(&eventset);
    if (ret != PAPI_OK) {
	PAPI_perror("Could not create event set");
	return -1;
    }

    // Check if enough hardware counters are available
    // On x86_64 intel, there should be ~8 counters available per physical core
    // However, these counter are shared by each hyperthread, which halves the number of available
    // counters per thread. Moreoever, 1 counter is reserved.
    // This is highly platform dependent however. Hence why this check is performed.
    int hwcounters = PAPI_num_hwctrs();
    if (hwcounters < PAPIH_NUM_EVENTS) {
	std::cerr << "Not enough hardware counters available : " << PAPIH_NUM_EVENTS << "needed but only "
		  << hwcounters << " available. Make sure to disable hyperthreading for more counters."
		  << std::endl;
	return -1;
    }

    if (add_papi_events(eventset)) {
	return -1;
    } 
    
    ret = PAPI_start(eventset);
    if (ret != PAPI_OK) {
	PAPI_perror("Could not start event");
	return -1;
    }
    g_eventset = eventset;
#endif // CCL_ENABLE_PAPI
    return 0;
}

int papih_read(long long *measurements, int measure_count) {
#if CCL_ENABLE_PAPI
    //PAPI_read(int EventSet, long long *values)
    if (!ENABLE_PAPI) {
	return 0;
    }
    
    int ret = PAPI_read(g_eventset, measurements);
    if (ret != PAPI_OK){
	PAPI_perror("Could not read hardware counters");
	return -1;
    }
#endif // CCL_ENABLE_PAPI
    return 0;
}
