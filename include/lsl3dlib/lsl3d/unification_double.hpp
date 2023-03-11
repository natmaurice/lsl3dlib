#ifndef CCL_ALGOS_3D_UNIFICATION_DOUBLE_HPP
#define CCL_ALGOS_3D_UNIFICATION_DOUBLE_HPP


#include <cstdint>

#include "lsl3dlib/features.hpp"
#include "lsl3dlib/lsl3d/unification_stats.hpp"
#include "lsl3dlib/lsl3d/unification_common.hpp"

#include <limits>
#include <simdhelpers/restrict.hpp>


namespace unify {

// ========================================
// Double Line Unfications
// ========================================


// Unification with temporary segments
template <typename LabelsSolver, typename ConfFeatures>
void unification_double_step1(const int16_t* restrict RLCi, int32_t* restrict ERAi, int16_t len,
			      AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
			       const int16_t row, const int16_t slice, StateCounters& counters);

// Unify temporary lines.
template <typename LabelsSolver, typename ConfFeatures>
void unification_combined_temp_segments(AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
					StateCounters& counters);




// Unify + Pipeline
template <typename LabelsSolver, typename ConfFeatures>
void unification_double_pipeline(const int16_t* restrict RLCi, int32_t* restrict ERAi,
				 const int16_t len, AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
				 const int16_t row, const int16_t slice, StateCounters& counters);


struct Unify_SM_Double {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;
	
	static constexpr bool ER = false;
	static constexpr bool ERA = true;
	static constexpr bool Double = true;
    };
    
    template <typename LabelsSolver, typename ConfFeatures>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi, int32_t* restrict ERAi,
			     int16_t segment_count, LabelsSolver& ET, Features& features,
			     const int16_t row, const int16_t slice, const int16_t image_width) {

	StateCounters counters;
	unification_double_step1<LabelsSolver, ConfFeatures>(
	    RLCi, ERAi, segment_count, state, ET, features, row, slice, counters);
	
        state.l_RLC1[state.l_len1]     = INT16_MAX - 1;
        state.l_RLC1[state.l_len1 + 1] = INT16_MAX - 1;
	// Unify both columns
	unification_combined_temp_segments<LabelsSolver, ConfFeatures>(state, ET, features, counters);
    }
};

struct Unify_SM_Double_PL {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool ER = false;
	static constexpr bool ERA = true;
	static constexpr bool Double = true;
    };
    
    template <typename LabelsSolver, typename ConfFeatures>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi, int32_t* restrict ERAi,
			     int16_t segment_count, LabelsSolver& ET, Features& features,
			     const int16_t row, const int16_t slice, const int16_t image_width) {

	StateCounters counters;
	unification_double_pipeline<LabelsSolver, ConfFeatures>(
	    RLCi, ERAi, segment_count, state, ET, features, row, slice, counters);
	
        state.l_RLC1[state.l_len1]     = INT16_MAX - 1;
	state.l_RLC1[state.l_len1 + 1] = INT16_MAX - 1;
    }
};


template <typename LabelsSolver, typename ConfFeatures>
__attribute__((always_inline))
void merge_segment_with(const int16_t ia, const int16_t ib,
			const int16_t* restrict rlc0, const int32_t* restrict era0,
			int16_t len0, int16_t& restrict ia0, int16_t& restrict ib0,
			int16_t& restrict er0,
			int32_t& restrict a, LabelsSolver& ET, Features& features) {

    
    
    int32_t r; 

    int16_t dbg_counter = 0;
    
    while (ib0 < ia) {
	next_erb(er0, ia0, ib0, rlc0);
    }
    if (ib < ia0) {
        return;
    }

    do {
	assert(dbg_counter++ < 2 + len0);	
	// Union
	// We could compact here
	r = era0[er0 / 2];
	r =  ET.FindRoot(r);
	if (r < a) {
	    std::swap(a, r);
	}
	ET.UpdateTable(r, a);
	features.Merge<ConfFeatures>(r, a);
	
	if (ib <= ib0) {
	    return;
	}
	next_erb(er0, ia0, ib0, rlc0);
    } while (ib >= ia0);
}



// Unification with temporary segments
template <typename LabelsSolver, typename ConfFeatures>
void unification_double_step1(const int16_t* restrict RLCi, int32_t* restrict ERAi, int16_t len,
			       AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
			       const int16_t row, const int16_t slice, StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t j0t, j1t;
    int16_t er_a = 1, er_b = 1, er_t = 1;
    int32_t a, ea_b;
    int32_t r;
    
    // Nothing to merge
    if (len == 0) {
	goto tail;
    }
    
    // Init phase
    j0a = RLCi[er_a - 1];
    j1a = RLCi[er_a];
    j0b = state.RLC1[er_b - 1];
    j1b = state.RLC1[er_b];

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA2);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA2);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::TAIL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::TAIL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::TAIL, UnificationState::TAIL);
    
  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	// New temporary segment
	state.l_ERA1[er_t / 2] = state.ERA1[er_b / 2];
	state.l_RLC1[er_t - 1] = j0b;
	state.l_RLC1[er_t]     = j1b;
	er_t += 2;
	
	er_b += 2;
	j0b = state.RLC1[er_b - 1];
	j1b = state.RLC1[er_b];
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	goto main;
    } else if (j1a < j0b) { // New lower segment is isolated
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
	goto new_label;
    }
    // goto merge_label
    j0t = std::min(j0a, j0b);

    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
  merge_label:
    ea_b = state.ERA1[er_b / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    features.AddSegment3D<ConfFeatures>(r, row, slice, j0a, j1a);
    
    if (j1a < j1b) {
	counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA2);
	goto next_er2;
    }
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    // goto next_er'
    
  next_erb:
    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = state.RLC1[er_b - 1];
    j1b = state.RLC1[er_b];

    // Speculative: will be overwritten if following condition is false
    // Kept here to prevent a double jump/branch.
    j1t = j1a; 
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
    
  transitive_merge:
    ea_b = state.ERA1[er_b / 2];
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    if (j1a < j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA2);
	goto next_er2;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;
    
  new_label:
    // Assign values for temporary segment
    j0t = j0a;
    j1t = j1a;

    a = ET.NewLabel();
    features.NewComponent3D<ConfFeatures>(a, row, slice, j0a, j1a);
    
    counters.Increment<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
	
  next_er:
    // Write temporary segment
    features.AddSegment3D<ConfFeatures>(a, row, slice, j0a, j1a);
    
    state.l_RLC1[er_t - 1] = j0t;
    state.l_RLC1[er_t]     = j1t;
    state.l_ERA1[er_t / 2] = a;
    er_t += 2;
    
    ERAi[er_a / 2] = a;
    er_a += 2;
    if (er_a >= len) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::TAIL);
	goto tail;
    }
    j0a = RLCi[er_a - 1];
    j1a = RLCi[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  next_er2:
    ERAi[er_a / 2] = a;
    er_a += 2;
    j1t = j1a; // Will be overwritten if the upper one extends farther
    if (er_a >= len) {
	state.l_RLC1[er_t - 1] = j0t;
	state.l_RLC1[er_t]     = j1b;
	state.l_ERA1[er_t / 2] = state.ERA1[er_b / 2];
	er_t += 2;

	er_b += 2; // Upper segment was before the lower one. Therefore, it is not the last one
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::TAIL);
	goto tail;
    }
    j0a = RLCi[er_a - 1];
    j1a = RLCi[er_a];

    if (j1b < j0a) { // Upper segment doesn't connect to the new lower segment
	// Write the end of the temporary segment
	state.l_RLC1[er_t - 1] = j0t;
	state.l_RLC1[er_t]     = j1b;
	state.l_ERA1[er_t / 2] = a;
	er_t += 2;

	er_b += 2;
	j0b = state.RLC1[er_b - 1];
	j1b = state.RLC1[er_b];

	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) { // New segment is isolated (top segment after)
	// Write the end of the temporary segment
	state.l_RLC1[er_t - 1] = j0t;
	state.l_RLC1[er_t]     = j1t;
	state.l_ERA1[er_t / 2] = a;
	er_t += 2;
	
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::NEW_LABEL);
	goto new_label;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MERGE);
    goto merge_label;
    
  tail:
    if (er_b >= state.len1) {
	goto end;
    }
    j0b = state.RLC1[er_b - 1];
    j1b = state.RLC1[er_b];	
    
    // Write temporary segment
    state.l_RLC1[er_t - 1] = j0b;
    state.l_RLC1[er_t]     = j1b;
    state.l_ERA1[er_t / 2] = state.ERA1[er_b / 2];
    er_t += 2;
    
    er_b += 2;

    counters.Increment<UseCounter>(UnificationState::TAIL, UnificationState::TAIL);
    goto tail;
  end:
    state.l_len1 = er_t - 1;
    return;    
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_combined_temp_segments(AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
					StateCounters& counters) {
    
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a;
    int32_t r;

    int16_t len = state.l_len0;

    
    // Nothing to merge
    if (len == 0) {
	return;
    }

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);

    
    // Init phase
    er_a = 1;
    j0a = state.l_RLC0[er_a - 1];
    j1a = state.l_RLC0[er_a];
    er_b = 1;
    j0b = state.l_RLC1[er_b - 1];
    j1b = state.l_RLC1[er_b];
    
  main:
    if (j0a > j1b) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	goto next_erb;
    } else if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
    // goto union_merge;
	
  union_merge:
    r = state.l_ERA1[er_b / 2];
    r = ET.FindRoot(r); // Might not be needed as the temporary segment shouldn't change (?)
    a = state.l_ERA0[er_a / 2];
    a = ET.FindRoot(a); // Might not be needed
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    // Here: An equivalence table might be needed to apply the merge onto the original era
    if (j1a < j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB0);
    goto next_erb; // Might be able to optimize that as some transition might not be possible
    // afterwards but still handled by majn
    
  next_er:
    er_a += 2;
    if (er_a > len) {
	goto end;
    }
    j0a = state.l_RLC0[er_a - 1];
    j1a = state.l_RLC0[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  next_erb:
    er_b += 2;
    j0b = state.l_RLC1[er_b - 1];
    j1b = state.l_RLC1[er_b];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    goto main;
  end:
    return;
    
}



template <typename LabelsSolver, typename ConfFeatures>
inline int32_t unification_prev_temp(int16_t j0t, int16_t j1t, int32_t r,
				     int16_t& restrict j0t2, int16_t& restrict j1t2, 
				     const int16_t* restrict rlc_rowt2,
				     const int32_t* restrict era_rowt2,
				     int16_t& restrict ert2,
				     LabelsSolver& ET, Features& features) {
    int32_t r2;
    while (j1t2 < j0t) {
	// Next ER''
	ert2 += 2;
	j0t2 = rlc_rowt2[ert2 - 1];
	j1t2 = rlc_rowt2[ert2];
    }
    while (j0t2 <= j1t) {
	// Contact
	r2 = era_rowt2[ert2 / 2];
	//r2 = uf.Parent(r2);
	r2 = ET.FindRoot(r2);
	r = ET.FindRoot(r);
	if (r2 < r) {
	    std::swap(r2, r);
	}
	ET.UpdateTable(r2, r);
	features.Merge<ConfFeatures>(r2, r);

	if (j1t < j1t2) {
	    break;
	}
		
	ert2 += 2;
	j0t2 = rlc_rowt2[ert2 - 1];
	j1t2 = rlc_rowt2[ert2];
    }
    return r;
}


// Unfication with temporary columns and a pipeline. This is implemented as a state machine and uses
// gotos
template <typename LabelsSolver, typename ConfFeatures>
void unification_double_pipeline(const int16_t* restrict RLCi, int32_t* restrict ERAi,
				 const int16_t len, AdjState<int16_t, int32_t>& state, LabelsSolver& ET, Features& features,
				 const int16_t row, const int16_t slice, StateCounters& counters) {

    using Seg_t = int16_t;
    using Label_t = int32_t;
    
    Seg_t j0a, j1a;
    Seg_t j0b, j1b;
    Seg_t j0t, j1t;
    Seg_t j0t2, j1t2;
    int16_t er = 1, erb = 1, ert = 1, ert2 = 1;
    Label_t a, ea_b;
    Label_t r;

       
    j0t2 = state.l_RLC0[0];
    j1t2 = state.l_RLC0[1];
    
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA2);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA2);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::TAIL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::TAIL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::TAIL, UnificationState::TAIL);

    
    // Nothing to merge
    if (len == 0) {
	goto tail;
    }

    // Init phase
    j0a = RLCi[er - 1];
    j1a = RLCi[er];
    j0b = state.RLC1[erb - 1];
    j1b = state.RLC1[erb];

  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	// Check if the previous double line touches upper segment and create a new
	// temporary segment

	r = state.ERA1[erb / 2];	    
	r = unification_prev_temp<LabelsSolver, ConfFeatures>(
	    j0b, j1b, r, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features);

	write_temp<Seg_t, Label_t>(ert, r, j0b, j1b, state.l_RLC1, state.l_ERA1);
	ert += 2;
	
	next_erb<Seg_t>(erb, j0b, j1b, state.RLC1);
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) { // New lower segment is isolated
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
	goto new_label;
    }
    // goto merge_label
    j0t = std::min(j0a, j0b);
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    
  merge_label:
    ea_b = state.ERA1[erb / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    features.AddSegment3D<ConfFeatures>(a, row, slice, j0a, j1a);
    
    if (j1a < j1b) {
	counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA2);
	goto next_er2;
    }
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    // goto next_er'
    
  next_erb:
    erb += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = state.RLC1[erb - 1];
    j1b = state.RLC1[erb];

    // Speculative: will be overwritten if following condition is false
    // Kept here to prevent a double jump/branch.
    j1t = j1a; 
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
    
  transitive_merge:
    ea_b = state.ERA1[erb / 2];
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    if (j1a < j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA2);
	goto next_er2;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;
    
  new_label:
    // Assign values for temporary segment
    j0t = j0a;
    j1t = j1a;

    a = ET.NewLabel();
    features.NewComponent3D<ConfFeatures>(a, row, slice, j0a, j1a);
    
    counters.Increment<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
	
  next_er:
    // Write temporary segment
    a = unification_prev_temp<LabelsSolver, ConfFeatures>(
	j0t, j1t, a, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features); 

    write_temp<Seg_t, Label_t>(ert, a, j0t, j1t, state.l_RLC1, state.l_ERA1);
    ert += 2;
    
    ERAi[er / 2] = a;
    er += 2;
    if (er >= len) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::TAIL);
	goto tail;
    }
    j0a = RLCi[er - 1];
    j1a = RLCi[er];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  next_er2:
    ERAi[er / 2] = a; // ?
    er += 2;
    j1t = j1a; // Will be overwritten if the upper one extends farther
    if (er >= len) {
	j1t = j1b;
	a = unification_prev_temp<LabelsSolver, ConfFeatures>(
	    j0t, j1t, a, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features); 

	write_temp<Seg_t, Label_t>(ert, a, j0t, j1t, state.l_RLC1, state.l_ERA1);
	ert += 2;

	erb += 2; // Upper segment was before the lower one. Therefore, it is not the last one
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::TAIL);
	goto tail;
    }
    j0a = RLCi[er - 1];
    j1a = RLCi[er];

    if (j1b < j0a) { // Upper segment doesn't connect to the new lower segment
	// Write the end of the temporary segment
	j1t = j1b;
        a = unification_prev_temp<LabelsSolver, ConfFeatures>(j0t, j1t, a, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features); 

	write_temp(ert, a, j0t, j1t, state.l_RLC1, state.l_ERA1);
	ert += 2;

	erb += 2;
	j0b = state.RLC1[erb - 1];
	j1b = state.RLC1[erb];

	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) { // New segment is isolated (top segment after)
	// Write the end of the temporary segment
	a = unification_prev_temp<LabelsSolver, ConfFeatures>(
	    j0t, j1t, a, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features); 
	write_temp(ert, a, j0t, j1t, state.l_RLC1, state.l_ERA1);
	ert += 2;
	
	counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::NEW_LABEL);
	goto new_label;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA2, UnificationState::MERGE);
    goto merge_label;
    
  tail:
    if (erb >= state.len1) {
	goto end;
    }
    j0b = state.RLC1[erb - 1];
    j1b = state.RLC1[erb];	

    r = state.ERA1[erb / 2];
    // Write temporary segment
    r = unification_prev_temp<LabelsSolver, ConfFeatures>(
	j0b, j1b, r, j0t2, j1t2, state.l_RLC0, state.l_ERA0, ert2, ET, features); 

    write_temp(ert, r, j0b, j1b, state.l_RLC1, state.l_ERA1);
    ert += 2;
    
    erb += 2;

    counters.Increment<UseCounter>(UnificationState::TAIL, UnificationState::TAIL);
    goto tail;
  end:
    state.l_len1 = ert - 1;
    return; 
}


}


#endif // CCL_ALGOS_3D_UNIFICATION_DOUBLE_HPP
