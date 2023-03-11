#ifndef CCL_ALGOS_3D_UNIFICATION_ER_HPP
#define CCL_ALGOS_3D_UNIFICATION_ER_HPP

#include <cstdint>

#include <lsl3dlib/lsl/lsl_utils.hpp>
#include "lsl3dlib/lsl3d/unification_common.hpp"
#include <lsl3dlib/features.hpp>

#include <simdhelpers/restrict.hpp>


namespace unify {


// Utility functions

inline void lsl_get_segment(const int16_t* restrict rlc_row, int32_t er, int32_t width,
			    int16_t& segment_start, int16_t& segment_end);

inline void lsl_get_segmentz(const int16_t* restrict rlc_row, int16_t er, int32_t width,
		      int16_t& segment_start, int16_t& segment_end);

template <typename LabelsSolver, typename ConfFeatures>
inline void lsl_3d_combine(int16_t er, int16_t segment_start, int16_t segment_end,
			   const int16_t* restrict ER0, int32_t* restrict ERA0,
			   int32_t& restrict label, LabelsSolver ET,
			   Features& features,
			   const int16_t row, const int16_t slice);

template <typename LabelsSolver, typename ConfFeatures>
inline void unification_er(
    int32_t width, int32_t height, int16_t* restrict ER0, int16_t* restrict ER1, int32_t*** ERA,
    int16_t * restrict rlc_row, int32_t* restrict era_row,
    int32_t segment_count, LabelsSolver ET, Features& features, int32_t row,
    int32_t slice, int32_t& nea);

template <typename LabelsSolver, typename ConfFeatures>
inline void unification_z_er(
    int32_t width, int32_t height, int16_t* restrict ER0, int16_t* restrict ER1, int32_t*** ERA,
    int16_t * restrict rlc_row, int32_t* restrict era_row,
    int32_t segment_count, LabelsSolver& ET, Features& features,
    int32_t row, int32_t slice, int32_t& nea);
    

template <typename LabelsSolver, typename ConfFeatures>
void unification_z_er(int16_t * restrict RLCi, int32_t* restrict ERAi, int32_t len,
		      AdjState<int16_t, int32_t>& state, LabelsSolver& ET,
		      Features& features, int16_t row, int16_t slice,
		      int16_t image_width);


struct Unify_ER {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool ER = true;
	static constexpr bool ERA = true;
	static constexpr bool Double = false;
    };

    template <typename LabelsSolver, typename ConfFeatures>
    static inline void Unify(AdjState<int16_t, int32_t>& state, int16_t* restrict RLCi, int32_t* restrict ERAi,
			     int16_t segment_count, LabelsSolver& ET,
			     Features& features, const int16_t row,
			     const int16_t slice, int16_t image_width) {
	
        unification_z_er<LabelsSolver&, ConfFeatures>(RLCi, ERAi, segment_count, state, ET, features,
						     row, slice, image_width);
    }
};



// Implementations

void lsl_get_segment(const int16_t* restrict rlc_row, int32_t er, int32_t width,
		     int16_t& segment_start, int16_t& segment_end) {
    segment_start = rlc_row[er - 1];
    segment_end =   rlc_row[er];

    // Take care of corner neighbours
    if (segment_start > 0) {
	segment_start--;
    }
    if (segment_end < width - 1) {
	segment_end++;
    }
}

void lsl_get_segmentz(const int16_t* restrict rlc_row, int16_t er, int32_t width,
		     int16_t& segment_start, int16_t& segment_end) {
    segment_start = rlc_row[er - 1];
    segment_end =   rlc_row[er] - 1; // Take care of zero addressing

    // Take care of corner neighbours
    if (segment_start > 0) {
	segment_start--;
    }
    if (segment_end < width - 1) {
	segment_end++;
    }
}

template <typename LabelsSolver, typename ConfFeatures>
void lsl_3d_combine(int16_t er, int16_t segment_start, int16_t segment_end,
		    const int16_t* restrict ER0, int32_t* restrict ERA0, int32_t& restrict label,
		    LabelsSolver& ET, Features& features, const int16_t row,
		    const int16_t slice) {

    
    int16_t er0 = ER0[segment_start];
    int16_t er1 = ER0[segment_end];

    // Does not work
    if (er0 % 2 == 0) {
	er0++;
    }
    if (er1 % 2 == 0){
	er1--;
    }

    if (er1 >= er0) {
	int32_t segment_id = ::algo::to_era_index(er0);
	int32_t ea = ERA0[segment_id];
	int32_t ancestor = ET.FindRoot(ea);
	// Propagate the changes from other rows
	if (label < ancestor) {
	    ET.UpdateTable(ancestor, label);
	    features.Merge<ConfFeatures>(ancestor, label);
	    ancestor = label;
	} else if (label != INT32_MAX) {
	    ET.UpdateTable(label, ancestor);
	    features.Merge<ConfFeatures>(label, ancestor);
	} else {
	    // [BUG] segment_start & segment_end => area to check rather than segment boundaries
	    features.AddSegment3D<ConfFeatures>(ancestor, row, slice, segment_start, segment_end);
	}
	
	for (int16_t erk = er0 + 2; erk <= er1; erk += 2) {
	    segment_id = ::algo::to_era_index(erk);
	    int32_t eak = ERA0[segment_id];
	    int32_t ancestork = ET.FindRoot(eak);
	    if (ancestor < ancestork) {
		ET.UpdateTable(ancestork, ancestor);
		features.Merge<ConfFeatures>(ancestork, ancestor);
	    } else if (ancestor > ancestork) {
		ET.UpdateTAble(ancestor, ancestork);
		features.Merge<ConfFeatures>(ancestor, ancestork);
		ancestor = ancestork;
	    }
	}
	label = ancestor;
    } 
}

template <typename LabelsSolver, typename ConfFeatures>
void lsl_combine_z(int16_t er, int16_t segment_start, int16_t segment_end,
		    const int16_t* restrict ER0, int32_t* restrict ERA0, int32_t& restrict label,
		   LabelsSolver& ET, Features& features, const int16_t row,
		   const int16_t slice) {

    // Note: ER0 is expected to have left and right borders
    int16_t er0 = ER0[segment_start - 1]; 
    int16_t er1 = ER0[segment_end];    
    
    // Does not work
    if (er0 % 2 == 0) {
	er0++; 
    }
    if (er1 % 2 == 0){
	er1--;
    }

    if (er1 >= er0) {
	int32_t segment_id = ::algo::to_era_index(er0);
	int32_t ea = ERA0[segment_id];
	int32_t ancestor = ET.FindRoot(ea);
	// Propagate the changes from other rows
	if (label < ancestor) {
	    ET.UpdateTable(ancestor, label);
	    features.Merge<ConfFeatures>(ancestor, label);
	    ancestor = label;
	} else if (label != INT32_MAX) {
	    ET.UpdateTable(label, ancestor);
	    features.Merge<ConfFeatures>(label, ancestor);
	} else {
	    // [BUG] segment_start & segment_end => area to check rather than segment boundaries
	    features.AddSegment3D<ConfFeatures>(ancestor, row, slice, segment_start, segment_end);
	    
	}
	
	for (int16_t erk = er0 + 2; erk <= er1; erk += 2) {
	    segment_id = ::algo::to_era_index(erk);
	    int32_t eak = ERA0[segment_id];
	    int32_t ancestork = ET.FindRoot(eak);
	    if (ancestor < ancestork) {
		assert(ET.GetLabel(ancestor) == ancestor);
	        ET.UpdateTable(ancestork, ancestor);
		features.Merge<ConfFeatures>(ancestork, ancestor);
	    } else if (ancestor > ancestork) {
		assert(ET.GetLabel(ancestor) == ancestor);
	        ET.UpdateTable(ancestor, ancestork);
		features.Merge<ConfFeatures>(ancestor, ancestork);
		ancestor = ancestork;
	    }
	}
	label = ancestor;
    } 
    
}




template <typename LabelsSolver, typename ConfFeatures>
void unification_er(
    int32_t width, int32_t height, int16_t* restrict ER0, int16_t* restrict ER1, int32_t*** ERA,
    int16_t * restrict rlc_row, int32_t* restrict era_row,
    int32_t segment_count, LabelsSolver& ET, Features& features, int32_t row,
    int32_t slice, int32_t& nea) {

    const int32_t slice_pitch = width * (height + 1);
    const int32_t row_pitch = width;
    const int32_t line_border = row_pitch;

    // Get the correct pointers for each row in the ER0/1 matrices
    // Also handles the borders at the beginning of each slice (line_border)
    const int16_t* restrict er0_row = ER1 + row_pitch * (row - 1) + line_border;
    const int16_t* restrict er1_row = ER0 + row_pitch * row + line_border;
    const int16_t* restrict er2_row = er1_row - row_pitch;
    const int16_t* restrict er3_row = er1_row + row_pitch;

    // There may be an aliasing in the case of empty lines.
    // This should't an issue however as the content of an empty is never accessed
    int32_t* restrict ERA0 = ERA[slice][row - 1];
    int32_t* restrict ERA1 = ERA[slice - 1][row];
    int32_t* restrict ERA2 = ERA[slice - 1][row - 1];
    int32_t* restrict ERA3 = ERA[slice - 1][row + 1];
    
    int16_t ner = segment_count;
    for (int32_t er = 1; er < ner; er += 2) {
	int16_t segment_start, segment_end;
	lsl_get_segment(rlc_row, er, width, segment_start, segment_end);
	
	int32_t label = INT32_MAX;
        lsl_3d_combine<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, er0_row, ERA0, label, ET, row, slice);
	lsl_3d_combine<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, er1_row, ERA1, label, ET, row, slice);
	lsl_3d_combine<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, er2_row, ERA2, label, ET, row, slice);
	lsl_3d_combine<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, er3_row, ERA3, label, ET, row, slice);

	// If label == EQ.Size() => No neighbour has been found
	// Therefore create a new label
	if (label == INT32_MAX) {
	    label = ET.NewComponent(); // Increment the number of elements in the unionfind structure
	    features.NewComponent3D<ConfFeatures>(label, row, slice, segment_start, segment_end + 1);
	    nea = label;
	}
	const int16_t era_offset = er / 2;
	era_row[era_offset] = label;
    }
}



template <typename LabelsSolver, typename ConfFeatures>
void unification_z_er(int16_t * restrict RLCi, int32_t* restrict ERAi, int32_t len,
		      AdjState<int16_t, int32_t>& state, LabelsSolver& ET,
		      Features& features, int16_t row, int16_t slice,
		      int16_t image_width) {
    int32_t label;
    
    for (int32_t er = 1; er < len; er += 2) {
	int16_t segment_start, segment_end;
	//lsl_get_segmentz(RLCi, er, image_width, segment_start, segment_end);

	segment_start = RLCi[er - 1];
	segment_end   = RLCi[er];
	
        label = INT32_MAX;
        lsl_combine_z<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, state.ER0, state.ERA0, label, ET, features,
	    row, slice);
	
	lsl_combine_z<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, state.ER1, state.ERA1, label, ET, features,
	    row, slice);
	
	lsl_combine_z<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, state.ER2, state.ERA2, label, ET, features,
	    row, slice);
	
	lsl_combine_z<LabelsSolver, ConfFeatures>(
	    er, segment_start, segment_end, state.ER3, state.ERA3, label, ET, features,
	    row, slice);

	// If label == EQ.Size() => No neighbour has been found
	// Therefore create a new label
	if (label == INT32_MAX) {
	    label = ET.NewLabel(); // Increment the number of elements in the unionfind structure
	    //std::cout << "NewLabel() = " << label << "\n";
	    features.NewComponent3D<ConfFeatures>(label, row, slice, segment_start, segment_end);
	}
	const int16_t era_offset = ::algo::to_era_index(er);
        ERAi[era_offset] = label;
    }
}

template <typename LabelsSolver, typename ConfFeatures>
void unification_z_er_double(
    const int16_t* restrict ER0, const int16_t* restrict ER1,
    const int16_t * restrict rlc1, const int16_t* rlc0,
    int32_t* restrict era0, int16_t len0,
    const int32_t* restrict era1, int16_t len1,
    int16_t* restrict rlct,
    int32_t* restrict erat, int16_t& lent, LabelsSolver& ET,
    Features& features, const int16_t row, const int16_t slice) {


    int16_t er0 = 1, er1 = 1, ert = 1;
    int32_t label = INT32_MAX;
    int32_t ancestor = -1;

    int16_t segment_start0, segment_end0; 
    int16_t segment_start1, segment_end1;
    int16_t segment_start = -1, segment_end = -1;

    // For each segment on the current line
    for (er1 = 1; er1 < len1; er1 += 2) {

	lsl_get_segmentz(rlc1, er1, len1, segment_start1, segment_end1);

	int16_t sbeg0, send0;
	sbeg0 = ER0[segment_start1];
	send0 = ER0[segment_end1];
	if (sbeg0 % 2 == 0) {
	    sbeg0++;
	}
	if (send0 % 2 == 0) {
	    send0--;
	}

	if (sbeg0 > er0) { // If there was a gap between in the segment chain
	    // Save temporary segement
	    write_temp(ert, label, segment_start, segment_end, rlct, erat);
	    ert += 2;

	    // Catch up and write top segments
	    for (; er0 < sbeg0; er0 += 2) {
		lsl_get_segmentz(rlc0, er0, len0, segment_start0, segment_end0);

		label = era0[er0 / 2];
		label = ET.FindRoot(label);
		write_temp(ert, label, segment_start0, segment_end0, rlct, erat);
		ert += 2;
	    }
	}

	if (sbeg0 <= send0) { // If connected to at least 1 segment on the upper line

	    if (label )
	    label = std::min(label, ancestor); 
	    er0 += 2;

	    // Perform consecutives fusions
	    for (; er0 < send0; er0 += 2) {
		lsl_get_segmentz(rlc0, er0, len0, segment_start0, segment_end0);

		ancestor = era0[er0 / 2];
		ancestor = ET.FindRoot(ancestor);
		if (ancestor < label) {
		    std::swap(label, ancestor);		
		}
	        ET.UpdateTable(ancestor, label);
	    }
	}
    }

    // A temporary segment is added on the next iteration if the previous segment was disconnected
    // We also have to do this after the loop to check for the last segment.
    if (len0 > 0) { 
	write_temp(ert, label, segment_start, segment_end, rlct, erat);
    }
    
    for (; er0 < len0; er0 += 2) {
	lsl_get_segmentz(rlc0, er0, len0, segment_start0, segment_end0);

	label = era0[er0 / 2];
	label = ET.FindRoot(label);

	write_temp(ert, label, segment_start0, segment_end0, rlct, erat);
    }
}


}

#endif // CCL_ALGOS_3D_UNIFICATION_ER_HPP
