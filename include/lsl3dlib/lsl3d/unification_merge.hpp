#ifndef CCL_ALGOS_3D_UNIFICATION_MERGE_HPP
#define CCL_ALGOS_3D_UNIFICATION_MERGE_HPP

#include <cstdint>

#include "lsl3dlib/lsl3d/unification_stats.hpp"
#include "lsl3dlib/lsl3d/unification_common.hpp"
#include <simdhelpers/restrict.hpp>
#include <simdhelpers/aligned_alloc.hpp>
#include <simdhelpers/assume.hpp>

#include "lsl3dlib/features.hpp"

namespace unify {


// First way to perform unificationo between rows:
// 1. Merge with the first row and set a temporary label to each ERA entry if no adjacent segment
// 2. Merge with 2 other rows and use the temporary label to check if it is transisitve or not
// 3. Merge with the last row, use the temporary label during individual merges and finally create
// a new label if a segment was never merged

template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_first(const int16_t* restrict rlc_rowa, int16_t len_a,
				    const int16_t* restrict rlc_rowb, int16_t len_b,
				    int32_t* restrict era_rowa, int32_t* restrict era_rowb,
				    LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
				    StateCounters& counters);

template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_transitive(const int16_t* restrict rlc_rowa, int16_t len_a,
					 const int16_t* restrict rlc_rowb, int16_t len_b,
					 int32_t* restrict era_rowa, int32_t* restrict era_rowb,
					 LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
					 StateCounters& counters);

template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_last(const int16_t* restrict rlc_rowa, int16_t len_a,
				   const int16_t* restrict rlc_rowb, int16_t len_b,
				   int32_t* restrict era_rowa, int32_t* restrict era_rowb,
				   LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
				   StateCounters& counters);



// Second way: Generalization of the 2 row merge
// TODO: The state machine is more complex
template <typename LabelsSolver, typename FeaturesConf>
void unification_merge_generalized(int16_t const* restrict rlc_row, int16_t len,
		      int16_t const* restrict rlc_row0, int16_t len0,
		      int16_t const* restrict rlc_row1, int16_t len1,
		      int16_t const* restrict rlc_row2, int16_t len2,
		      int16_t const* restrict rlc_row3, int16_t len3,
		      int32_t* restrict era_rowa,
		      int32_t* restrict era_row0,
		      int32_t* restrict era_row1,
		      int32_t* restrict era_row2,
		      int32_t* restrict era_row3,
		      LabelsSolver& ET, Features& features);

// Third way: Similar as the first one but instead of allocating temporary labels, labels are
// allocated in the union find structure
// 2 Steps:
// 1. Merge with the first row and create a new label if no adjacent segement
// 2. Merge with the other rows.
template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_first_bis(const int16_t* restrict rlc_rowa, int16_t len_a,
					const int16_t* restrict rlc_rowb, int16_t len_b,
					int32_t* restrict era_rowa,
					int32_t* restrict era_rowb,
					LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
					StateCounters& counters);

template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_transitive_bis(const int16_t* restrict rlc_rowa, int16_t len_a,
					     const int16_t* restrict rlc_rowb, int16_t len_b,
					     int32_t* restrict era_rowa, int32_t* restrict era_rowb,
					     LabelsSolver& ET, Features& features, const int16_t row,
					     const int16_t slice, StateCounters& counters);


// Fourth way: Fairly close the the third one. In this case, we only increment the size of the union
// find after the last unification. This means that the UnionFind will remain compact than with the
// third method
// TODO
template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_step1_third(const int16_t* restrict rlc_rowa, int16_t len_a,
					  const int16_t* restrict rlc_rowb, int16_t len_b,
					  int32_t* restrict era_rowa, int32_t* restrict era_rowb,
					  LabelsSolver& ET, Features& features,
					  StateCounters& counters);

template <typename LabelsSolver, typename FeaturesConf>
inline void unification_merge_step2_third(const int16_t* restrict rlc_rowa, int16_t len_a,
					  const int16_t* restrict rlc_rowb, int16_t len_b,
					  int32_t* restrict era_rowa,
					  int32_t* restrict era_rowb,
					  LabelsSolver& ET, Features& features,
					  StateCounters& counters);






struct Unify_Nothing {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool ER = false;
	static constexpr bool ERA = true;
	static constexpr bool Double = false;
    };
    
    template <typename LabelsSolver, typename FeaturesConf>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi,  int32_t* restrict ERAi,
			     int16_t segment_count, LabelsSolver& ET, Features& features,
			     const int16_t row, const int16_t slice, const int16_t image_width) {
	// Do nothing
    }
};

struct Unify_SM_Separate {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool ER = false;
	static constexpr bool ERA = true;
	static constexpr bool Double = false;
    };
    
    template <typename LabelsSolver, typename ConfFeatures>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi, int32_t* restrict ERAi,
			     int16_t segment_count, LabelsSolver& ET,
			     Features& features, 
			     const int16_t row, const int16_t slice, const int16_t image_width) {

	StateCounters counters;
	unification_merge_first<LabelsSolver, ConfFeatures>(RLCi, segment_count, state.RLC0, state.len0, ERAi,
					state.ERA0, ET, features, row, slice, counters);
	unification_merge_transitive<LabelsSolver, ConfFeatures>(RLCi, segment_count, state.RLC1, state.len1, ERAi,
					   state.ERA1, ET, features, row, slice, counters);
	unification_merge_transitive<LabelsSolver, ConfFeatures>(RLCi, segment_count, state.RLC2, state.len2, ERAi,
					   state.ERA2, ET, features, row, slice, counters);


	unification_merge_last<LabelsSolver, ConfFeatures>(RLCi, segment_count, state.RLC3, state.len3, ERAi,
					     state.ERA3, ET, features, row, slice, counters);

    }
};

struct Unify_SM_Separate_V2 {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool ER = false;
	static constexpr bool ERA = true;
	static constexpr bool Double = false;
    };
    
    template <typename LabelsSolver, typename FeaturesConf>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi, int32_t* restrict ERAi,
			     int16_t* restrict ER, int16_t segment_count, LabelsSolver& ET,
			     Features& features, const int16_t row,
			     const int16_t slice, const int16_t image_width) {

	StateCounters counters;
	unification_merge_first_bis<LabelsSolver, FeaturesConf>(
	    RLCi, segment_count, state.RLC0, state.len0, ERAi,
	    state.ERA0, ET, features, row, slice, counters);
	
	unification_merge_transitive_bis<LabelsSolver, FeaturesConf>(
	    RLCi, segment_count, state.RLC1, state.len1, ERAi,
	    state.ERA1, ET, features, row, slice, counters);
	
	unification_merge_transitive_bis<LabelsSolver, FeaturesConf>(
	    RLCi, segment_count, state.RLC2, state.len2, ERAi,
	    state.ERA2, ET, features, row, slice, counters);
	
	unification_merge_transitive_bis<LabelsSolver, FeaturesConf>(
	    RLCi, segment_count, state.RLC3, state.len3, ERAi,
	    state.ERA3, ET, features, row, slice, counters);

    }
};




// ================================================== //
// Implementations
// ================================================== //
template <typename LabelsSolver, typename ConfFeatures>
void unification_merge_first(const int16_t* restrict rlc_rowa, int16_t len_a,
			     const int16_t* restrict rlc_rowb, int16_t len_b,
			     int32_t* restrict era_rowa, int32_t* restrict era_rowb,
			     LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
			     StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a, ea_b;
    int32_t r;
    // This is similar to the 2D version with one small difference when creating a new label
    //using enum UnificationState; // Can only do that in C++20
    
    // Nothing to merge
    if (len_a == 0) {
	return;
    }

    // Init phase
    er_a = 1;
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    er_b = 1;
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::TEMP_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);	
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA);
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.Increment<UseCounter>(UnificationState::TEMP_LABEL, UnificationState::NEXT_ERA);
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
	
    
  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	er_b += 2;
	j0b = rlc_rowb[er_b - 1];
	j1b = rlc_rowb[er_b];
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::TEMP_LABEL);
	goto new_label;
    }
    // goto merge_label
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
  merge_label:

    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    features.AddSegment3D<ConfFeatures>(a, row, slice, j0a, j1a);

    if (j1a <= j1b) {
	    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA);
	goto next_er;
    }
    // goto next_er'
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
  next_erb:

    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
  transitive_merge:

    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;    
  new_label:

    // With 2D version: Create new label using EQ.NewLabel()
    // We cannot do that here: the segment may be connected to another in another row
    // Thus, we assign a temporary label that will be used to differentiate with other connected
    // segments and used to later check if a new label has to be created (if not connected to any
    // other segment)
    a = TEMP_LABEL;
    counters.Increment<UseCounter>(UnificationState::TEMP_LABEL, UnificationState::NEXT_ERA);
  next_er:
    era_rowa[er_a / 2] = a;
    er_a += 2;
    if (er_a >= len_a) {
	goto end;
    }
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  end:
    return;    
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_merge_transitive(const int16_t* restrict rlc_rowa, int16_t len_a,
				  const int16_t* restrict rlc_rowb, int16_t len_b,
				  int32_t* restrict era_rowa, int32_t* restrict era_rowb,
				  LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
				  StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a, ea_b;
    int32_t r;
    // Nothing to merge
    if (len_a == 0) {
	return;
    }

    // Init phase
    er_a = 1;
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    er_b = 1;
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);

    
  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	er_b += 2;
	j0b = rlc_rowb[er_b - 1];
	j1b = rlc_rowb[er_b];
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) {
	// No need to allocate a new label: it either has a temporary label or it is either already
	// connected to another segment
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
	goto next_er; 
    }
    // always first encounter => can't optimize away
    a = era_rowa[er_a / 2];
    if (a != TEMP_LABEL) {
	// Merged in a previous step
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
	goto transitive_merge;
    }
    // goto merge_label
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    
  merge_label:
    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    features.AddSegment3D<ConfFeatures>(a, row, slice, j0a, j1a);

    if (j1a <= j1b) {
 	counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    // goto next_er'
    
  next_erb:
    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
  transitive_merge:
    ea_b = era_rowb[er_b / 2];
    // Getting the root of a is necessary because it could fail for the given situtation:
    // 1. A transitive merge has been performed before (possible in 3D)
    // 2. Root(a) > Root(r)
    a = ET.FindRoot(a);
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;    
  write_era:
    era_rowa[er_a / 2] = a;
    counters.Increment<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
  next_er:
    er_a += 2;
    if (er_a >= len_a) {
	goto end;
    }
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  end:
    return;
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_merge_last(const int16_t* restrict rlc_rowa, int16_t len_a,
			    const int16_t* restrict rlc_rowb, int16_t len_b,
			    int32_t* restrict era_rowa, int32_t* restrict era_rowb,
			    LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
			    StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a, ea_b;
    int32_t r;
    // Nothing to merge
    if (len_a == 0) {
	return;
    }
    assert((long long)(era_rowb) % 4 == 0 && "Wrong alignment");

    // Init phase
    er_a = 1;
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    er_b = 1;
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::NEW_LABEL, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);

  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	er_b += 2;
	j0b = rlc_rowb[er_b - 1];
	j1b = rlc_rowb[er_b];
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    }
    a = era_rowa[er_a / 2]; // always first encounter => can't optimize away
    if (j1a < j0b) {
	if (a == TEMP_LABEL) { // Commit new component
	    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
	    a = ET.NewLabel();
	    features.NewComponent3D<ConfFeatures>(a, row, slice, j0a, j1a);       
		
	    counters.Increment<UseCounter>(UnificationState::NEW_LABEL, UnificationState::WRITE_ERA);
	    goto write_era; 
	}
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERA);
	goto next_er;
    }
    if (a != TEMP_LABEL) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::UNION);
	// Merged in a previous step	
	goto transitive_merge;
    }
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    // goto merge_label
    
  merge_label:
    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    features.AddSegment3D<ConfFeatures>(a, row, slice, j0a, j1a);

    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    // goto next_er'
    
  next_erb:
    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
    
  transitive_merge:
    ea_b = era_rowb[er_b / 2];
    // Getting the root of a is necessary because it could fail for the given situtation:
    // 1. A transitive merge has been performed before (possible in 3D)
    // 2. Root(a) > Root(r)
    a = ET.FindRoot(a);
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;
    
  write_era:
    era_rowa[er_a / 2] = a;    
    counters.Increment<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
  next_er:
    er_a += 2;
    if (er_a >= len_a) {
	goto end;
    }
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  end:
    return;
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_merge_first_bis(const int16_t* restrict rlc_rowa, int16_t len_a,
				 const int16_t* restrict rlc_rowb, int16_t len_b,
				 int32_t* restrict era_rowa, int32_t* restrict era_rowb,
				 LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
				 StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a, ea_b;
    int32_t r;
    // This is similar to the 2D version with one small difference when creating a new label

    
    // Nothing to merge
    if (len_a == 0) {
	return;
    }

    // Init phase
    er_a = 1;
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    er_b = 1;
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    counters.SetAccessible<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);	
    
  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	er_b += 2;
	j0b = rlc_rowb[er_b - 1];
	j1b = rlc_rowb[er_b];
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEW_LABEL);
	goto new_label;
    }
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::MERGE);
    // goto merge_label
    
  merge_label:
    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    a = r;
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::MERGE, UnificationState::NEXT_ERB1);
    // goto next_er'
    
  next_erb:
    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB1, UnificationState::UNION);
    // else: goto transitive_merge
    
  transitive_merge:
    ea_b = era_rowb[er_b / 2];
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERA);
	goto next_er;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB1);
    goto next_erb;
    
  new_label:
    // With 2D version: Create new label using EQ.NewLabel()
    // We cannot do that here: the segment may be connected to another in another row
    // Thus, we assign a temporary label that will be used to differentiate with other connected
    // segments and used to later check if a new label has to be created (if not connected to any
    // other segment)
    a = ET.NewLabel();    
    counters.Increment<UseCounter>(UnificationState::NEW_LABEL, UnificationState::NEXT_ERA);
    
  next_er:
    era_rowa[er_a / 2] = a;
    er_a += 2;
    if (er_a >= len_a) {
	goto end;
    }
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);	
    goto main;
    
  end:
    return;    
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_merge_transitive_bis(const int16_t* restrict rlc_rowa, int16_t len_a,
				      const int16_t* restrict rlc_rowb, int16_t len_b,
				      int32_t* restrict era_rowa, int32_t* restrict era_rowb,
				      LabelsSolver& ET, Features& features, const int16_t row, const int16_t slice,
				      StateCounters& counters) {
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int16_t er_a, er_b;
    int32_t a, ea_b;
    int32_t r;
    // Nothing to merge
    if (len_a == 0) {
	return;
    }

    // Init phase
    er_a = 1;
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    er_b = 1;
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];

    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::MAIN, UnificationState::UNION);    
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::UNION);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB0);
    counters.SetAccessible<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
    counters.SetAccessible<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);

    
  main:
    if (j1b < j0a) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::NEXT_ERB0);
	er_b += 2;
	j0b = rlc_rowb[er_b - 1];
	j1b = rlc_rowb[er_b];
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::MAIN);
	goto main;
    } else if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::WRITE_ERA);
	// No need to allocate a new label: it either has a temporary label or it is either already
	// connected to another segment	
	goto next_er; 
    }
    a = era_rowa[er_a / 2]; // always first encounter => can't optimize away
    counters.Increment<UseCounter>(UnificationState::MAIN, UnificationState::UNION);    
    goto transitive_merge;
    
  next_erb:
    er_b += 2;
    // No need to check if last segment:
    // a virtual segment is expected at the end
    j0b = rlc_rowb[er_b - 1];
    j1b = rlc_rowb[er_b];
    if (j1a < j0b) {
	counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::NEXT_ERB0, UnificationState::UNION);
    // else: goto transitive_merge
    
  transitive_merge:
    ea_b = era_rowb[er_b / 2];
    // Getting the root of a is necessary because it could fail for the given situtation:
    // 1. A transitive merge has been performed before (possible in 3D)
    // 2. Root(a) > Root(r)
    a = ET.FindRoot(a);
    r = ET.FindRoot(ea_b);
    if (r < a) {
	std::swap(a, r);
    }
    ET.UpdateTable(r, a);
    features.Merge<ConfFeatures>(r, a);
    if (j1a <= j1b) {
	counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::WRITE_ERA);
	goto write_era;
    }
    counters.Increment<UseCounter>(UnificationState::UNION, UnificationState::NEXT_ERB0);
    goto next_erb;
    
  write_era:
    counters.Increment<UseCounter>(UnificationState::WRITE_ERA, UnificationState::NEXT_ERA);
    era_rowa[er_a / 2] = a;
    
  next_er:
    er_a += 2;
    if (er_a >= len_a) {
	goto end;
    }
    j0a = rlc_rowa[er_a - 1];
    j1a = rlc_rowa[er_a];
    
    counters.Increment<UseCounter>(UnificationState::NEXT_ERA, UnificationState::MAIN);
    goto main;
    
  end:
    return;
}



}

#endif // CCL_AGLOS_3D_UNIFICATION_MERGE_HPP
