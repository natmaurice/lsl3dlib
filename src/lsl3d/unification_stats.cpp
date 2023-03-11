#include <lsl3dlib/lsl3d/unification_stats.hpp>

#include <cmath>

namespace unify {



StateCounters::StateCounters() {
    ResetCounters();
}

void StateCounters::ResetCounters() {
    if (ENABLE_STATE_COUNTERS) {
	for (int i = 0; i < LSL_STATE_COUNT * LSL_STATE_COUNT; i++) {
	    counters[i] = -1.0;
	}
    }
}


long StateCounters::GetCounter(int src_state, int dest_state) const {
    int transition = src_state * LSL_STATE_COUNT + dest_state;
    return counters[transition];
}

long StateCounters::GetCounter(const UnificationState& src_state, const UnificationState& dest_state) const {
    return GetCounter(static_cast<int>(src_state), static_cast<int>(dest_state));    
}



int64_t StateCounters::Total() const {
    int64_t total = 0;
    for (auto src_state = static_cast<int>(UnificationState::MAIN);
        src_state <= static_cast<int>(UnificationState::NEW_LABEL); src_state++) {
	for (auto dest_state = static_cast<int>(UnificationState::MAIN);
        dest_state <= static_cast<int>(UnificationState::NEW_LABEL); dest_state++) {

	    // Transitive is callled twice. This averages the value
	    total += GetCounter(src_state, dest_state);
	}
    }
    return total;
}


bool StateStats::Add(const StateStats& other) {
    for (const auto& it: other.counters) {
	const std::string name = it.first;



	
    }
    return true;
}



const char* get_state_name(const UnificationState& state) {
    switch (state) {
    case UnificationState::MAIN:
	return "Main";
    case UnificationState::MERGE:
	return "Merge";
    case UnificationState::UNION:
	return "Union";
    case UnificationState::NEXT_ERA:
	return "Next ER (0)";
    case UnificationState::NEXT_ERA2:
	return "Next ER (1)";
    case UnificationState::NEXT_ERB0:
	return "Next ER' (0)";
    case UnificationState::NEXT_ERB1:
	return "Next ER' (1)";
    case UnificationState::WRITE_ERA:
	return "Write ERA";
    case UnificationState::TEMP_LABEL:
	return "Temp. Label";
    case UnificationState::TAIL:
	return "Tail";
    case UnificationState::NEW_LABEL:
	return "New Label";
    default:
	return "Unknown State";
    }
}


}
