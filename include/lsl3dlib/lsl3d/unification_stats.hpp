#ifndef CCL_ALGOS_3D_UNIFICATION_STATS_HPP
#define CCL_ALGOS_3D_UNIFICATION_STATS_HPP

#include <map>
#include <string>
#include <cassert>

namespace unify {

constexpr long LSL_STATE_COUNT = 11;
constexpr long LSL_COUNTERS_COUNT = 4;


constexpr bool ENABLE_STATE_COUNTERS = false;


enum class UnificationState : int {
    MAIN,
    MERGE,
    UNION,
    NEXT_ERA,
    NEXT_ERA2,
    NEXT_ERB0,
    NEXT_ERB1,
    WRITE_ERA,
    TEMP_LABEL,
    TAIL,
    NEW_LABEL // Note: Adding extra entry requires to change save_counters() in stats/count_states.cpp
};



struct StateCounters {
    StateCounters();

    void ResetCounters();

    template <bool UseCounter = false>
    void Increment(const UnificationState& src_state, const UnificationState& dest_state);

    // Indicate that the given transition can be reached by state machine
    template <bool UseCounter = false>
    void SetAccessible(const UnificationState& src_state, const UnificationState& dest_state);
    
    long GetCounter(int src_source, int dest_state) const;
    long GetCounter(const UnificationState& src_state, const UnificationState& dest_state) const;

    int64_t Total() const;
    
    long counters[LSL_STATE_COUNT * LSL_STATE_COUNT];
};


struct StateStats {
    bool Add(const StateStats& other);
    
    std::map<std::string, StateCounters> counters;
};


template <bool UseCounter>
void StateCounters::Increment(const UnificationState& src_state, const UnificationState& dest_state) {
    if (UseCounter || ENABLE_STATE_COUNTERS) {

	int counter_id = static_cast<int>(src_state) * LSL_STATE_COUNT + static_cast<int>(dest_state);
	assert(counters[counter_id] >= 0);
	counters[counter_id]++;
    }
}

template <bool UseCounter>
void StateCounters::SetAccessible(const UnificationState& src_state,
				 const UnificationState& dest_state) {
    if (UseCounter || ENABLE_STATE_COUNTERS) {
	int counter_id = static_cast<int>(src_state) * LSL_STATE_COUNT + static_cast<int>(dest_state);
	if (counters[counter_id] < 0) {
	    counters[counter_id] = 0;
	}
    }
}


const char* get_state_name(const UnificationState& state);





}

#endif // CCL_ALGOS_3D_UNIFICATION_STATS_HPP
