#ifndef CCL_ALGOS_3D_REDUCTION_HPP
#define CCL_ALGOS_3D_REDUCTION_HPP

#include <cstdint>

#include "algos/3d/unification_common.hpp"
#include "features.hpp"

struct Reduction3D_FSM {

    
    template <typename LabelsSolver, typename ConfFeatures>
    static void ReduceLine(const int16_t* restrict RLC0, const int16_t* restrict RLC1,
			   const int32_t* restrict ERA0, const int32_t* restrict ERA1,
			   int16_t len_a, int16_t len_b, LabelsSolver& ET, Features& features) {


	int16_t er_a = 1, er_b = 1;
	int16_t j0a, j1a;
	int16_t j0b, j1b;
	int32_t label_a, label_b;

	if (len_a == 0) {
	    return;
	}
	
	// Init
	j0a = RLC1[er_a - 1];
	j1a = RLC1[er_a];
	j0b = RLC0[er_b - 1];
	j1b = RLC0[er_b];

      main:
	
	if (j1b < j0a) { // Top before
	    er_b += 2;
	    j0b = RLC0[er_b - 1];
	    j1b = RLC0[er_b];
	    goto main;
	} else if (j1a < j0b) {
	    goto next_a;
	}

	// Merge
	label_a = ERA1[er_a / 2];
	label_a = ET.FindRoot(label_a);
      merge:	
	label_b = ERA0[er_b / 2];
	label_b = ET.FindRoot(label_b);

	if (label_a < label_b) {
	    std::swap(label_a, label_b);
	}
	
	if (label_a != label_b) {
	    ET.UpdateTable(label_a, label_b);
	    features.Merge<ConfFeatures>(label_a, label_b);
	}
	
	if (j1a < j1b) {
	    goto next_a;
	}
	// goto next_b;
	
      next_b:
       	er_b += 2;

	j0b = RLC0[er_b - 1];
	j1b = RLC0[er_b];
	if (j1a < j0b) {
	    goto next_a;
	}
	goto merge;

      next_a:
	er_a += 2;
	if (er_a >= len_a) {
	    goto end;
	}

	j0a = RLC1[er_a - 1];
	j1a = RLC1[er_a];

	goto main;

      end:
	return;
    }

    
    
    template <typename LabelsSolver, typename ConfFeatures>
    static void Reduce(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi,
		       int32_t* restrict ERAi, int16_t len, LabelsSolver& ET, Features& features) {

	ReduceLine<LabelsSolver, ConfFeatures>(RLCi, state.RLC1, ERAi, state.ERA1, len, state.len1, ET, features);
	ReduceLine<LabelsSolver, ConfFeatures>(RLCi, state.RLC2, ERAi, state.ERA2, len, state.len2, ET, features);
	ReduceLine<LabelsSolver, ConfFeatures>(RLCi, state.RLC3, ERAi, state.ERA3, len, state.len3, ET, features);
	
    }
};

#endif // CCL_ALGOS_3D_REDUCTION_HPP
