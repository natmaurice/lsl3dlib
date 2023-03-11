#ifndef CCL_ALGOS_3D_REDUCE_FSM_HPP
#define CCL_ALGOS_3D_REDUCE_FSM_HPP

#include <cstdint>
#include <lsl3dlib/lsl3d/unification_common.hpp>
#include <utility>
#include <iostream>
#include <lsl3dlib/features.hpp>

struct Reduce_FSM {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;
    };

    
    template <typename LabelsSolver, typename ConfFeatures>
    static void ReduceLine(const Conf::Seg_t* restrict RLC0, const Conf::Seg_t* restrict RLC1,
		       const int32_t* restrict ERA0, const int32_t* restrict ERA1,
			   int16_t len_b, int16_t len_a, LabelsSolver& ET, Features& features);

    
    template <typename LabelsSolver, typename ConfFeatures>
    static void Reduce(AdjState<Conf::Seg_t, Conf::Label_t>& state, Conf::Seg_t* RLCi, int32_t *ERAi,
		       int16_t len, LabelsSolver& ET, Features& features);
    
};

// RLC0: Previous line
// RLC1: Current line
template <typename LabelsSolver, typename ConfFeatures>
void Reduce_FSM::ReduceLine(const int16_t* restrict RLC1, const int16_t* restrict RLC0,
			    const int32_t* restrict ERA1, const int32_t* restrict ERA0,
			    int16_t len1, int16_t len0, LabelsSolver& ET, Features& features) {

    int16_t er1 = 1, er0 = 1;
    int16_t j0a, j1a;
    int16_t j0b, j1b;
    int32_t label1, label0;

    if (len1 == 0) {
	return;
    }
	
    // Init
    j0a = RLC1[er1 - 1];
    j1a = RLC1[er1];
    j0b = RLC0[er0 - 1];
    j1b = RLC0[er0];

  main:
	
    if (j1b < j0a) { // Top before
	er0 += 2;
	j0b = RLC0[er0 - 1];
	j1b = RLC0[er0];
	goto main;
    } else if (j1a < j0b) {
	goto next_a;
    }

    // Chunk b must precede Chunk a, therefore
    // all labels in a > all labels in b
    // Furthermore, the tree in Chunk a has been flattened => no need for FindRoot, GetLabel is
    // enough
    // Note: FindRoot is still needed for Chunk b because of potentially transitive merge across
    // chunks
	
    label1 = ERA1[er1 / 2];
    label1 = ET.FindRoot(label1);
  merge:
    label0 = ERA0[er0 / 2];
    label0 = ET.FindRoot(label0);

    
    if (label0 < label1) {
	std::swap(label1, label0);
    }
    
    ET.UpdateTable(label0, label1);
    features.Merge<ConfFeatures>(label0, label1);
    if (label1 != label0) {
	//std::cout << "ET[" << label0 << "] = "  << label1 << "\n";
    }

    if (j1a < j1b) {
	goto next_a;
    }
    // goto next_b;
	
  next_b:
    er0 += 2;

    j0b = RLC0[er0 - 1];
    j1b = RLC0[er0];
    if (j1a < j0b) {
	goto next_a;
    }
    goto merge;

  next_a:
    er1 += 2;
    if (er1 >= len1) {
	goto end;
    }

    j0a = RLC1[er1 - 1];
    j1a = RLC1[er1];

    goto main;

  end:
    return;    
}

template <typename LabelsSolver, typename ConfFeatures>
void Reduce_FSM::Reduce(AdjState<Conf::Seg_t, Conf::Label_t> &state, Conf::Seg_t *RLCi, int32_t *ERAi,
			int16_t len, LabelsSolver &ET, Features& features) {
    
    Reduce_FSM::ReduceLine<LabelsSolver, ConfFeatures>(
	RLCi, state.RLC1, ERAi, state.ERA1, len, state.len1, ET, features);
    
    Reduce_FSM::ReduceLine<LabelsSolver, ConfFeatures>(
	RLCi, state.RLC2, ERAi, state.ERA2, len, state.len2, ET, features);
    
    Reduce_FSM::ReduceLine<LabelsSolver, ConfFeatures>(
	RLCi, state.RLC3, ERAi, state.ERA3, len, state.len3, ET, features);
}

#endif // CCL_ALGOS_3D_REDUCE_FSM_HPP
