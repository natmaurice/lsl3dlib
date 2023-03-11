#ifndef CCL_ALGOS_3D_LSL_FEATURES_HPP
#define CCL_ALGOS_3D_LSL_FEATURES_HPP

#include <lsl3dlib/features.hpp>
#include <simdhelpers/restrict.hpp>


struct FeatureComputation_None {

    struct Conf {
	using Seg_t = int16_t;
    };

    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(Conf::Seg_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
		      Features& features, size_t label_count, int depth, int height, int width) {
    }

    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(int16_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
			     Features& features, size_t min_label, size_t max_label,
			     int x0, int y0, int z0, int x1, int y1, int z1) {
    }
    
};

struct FeatureComputation_Line {

    struct Conf {
	using Label_t = int32_t;
	using Seg_t = int16_t;
    };
    
    template <typename ConfFeatures>
    static void CalcFeatures(const Conf::Seg_t* restrict RLCi, int32_t* restrict ERAi, int16_t len,
			     int old_label_count, Features& features, int slice, int row, int col) {
	
        for (int er = 1; er < len; er += 2) {
	    Conf::Seg_t segment_start = RLCi[er - 1];
	    Conf::Seg_t segment_end = RLCi[er];

	    Conf::Label_t label = ERAi[er / 2];

	    if (label > old_label_count) {
		features.NewComponent3D<ConfFeatures>(label, row, slice, segment_start, segment_end);
		//std::cout << "NewComponent3D: ([" << segment_start << ", " << segment_end << "], "
		//	  << row << ", " << slice << ") " << label << ", S = " << features.S[label]
		//	  << ", lo_col = " << features.lo_col[label] << "\n"; 
	    } else {
		features.AddSegment3D<ConfFeatures>(label, row, slice, segment_start, segment_end);
		//std::cout << "AddSegment3D: ([" << segment_start << ", " << segment_end << "], "
		//	  << row << ", " << slice << ") " << label << ", S = " << features.S[label]
		//	  << ", lo_col = " << features.lo_col[label] << "\n"; 
	    }
	}
    }

    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(typename Conf::Seg_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
			     Features& features, size_t label_count, int depth, int height, int width) {
    }

};

// Feature computation on-the-fly = done during unification
using FeatureComputation_OTF = FeatureComputation_None;



struct FeatureComputation {

    struct Conf {
	using Seg_t = int16_t;
    };
    
    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(int16_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
			     Features& features, size_t min_label, size_t max_label,
			     int x0, int y0, int z0, int x1, int y1, int z1) {

	features.Init<ConfFeatures>(min_label, max_label);
	//std::cout << "-- Feature Computation -- \n";
	//std::cout << "min_label = " << min_label << ", max_label = " << max_label << "\n";
	
	for (int slice = z0; slice <= z1; slice++) {
	    for (int row = y0; row <= y1; row++) {
		int ner = Lengths[slice][row];
		for (int er = 1; er < ner; er += 2) {

		    int segstart = RLC[slice][row][er - 1];
		    int segend = RLC[slice][row][er];
		    int label = ERA[slice][row][er / 2];
		    label = ET.GetLabel(label);

		    features.AddSegment3D<ConfFeatures>(label, row, slice, segstart, segend);

		}
	    }
	}
    }

    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(Conf::Seg_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
			     Features& features, size_t label_count, int depth, int height, int width) {

	// Initialize features
	// Ignore '0' label -> start at 1
	features.Init<ConfFeatures>(label_count);
	
	for (int slice = 0; slice < depth; slice++) {
	    for (int row = 0; row < height; row++) {
		
		const uint16_t segment_count = Lengths[slice][row];
		const int16_t* restrict RLCi = RLC[slice][row];
		const int32_t* restrict ERAi = ERA[slice][row];
		
		for (uint16_t er = 1; er < segment_count; er += 2) {

		    const int16_t segment_start = RLCi[er - 1];
		    const int16_t segment_end = RLCi[er];
		    int32_t label = ERAi[er / 2];
		    label = ET.GetLabel(label);

		    features.AddSegment3D<ConfFeatures>(
			label, row, slice, segment_start, segment_end);
		}
	    }
	}
    }

    
};

#endif // CCL_ALGOS_3D_LSL_FEATURES_HPP
