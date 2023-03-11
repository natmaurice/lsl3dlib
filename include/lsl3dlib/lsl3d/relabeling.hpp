#ifndef CCL_ALGOS_3D_RELABELING_HPP
#define CCL_ALGOS_3D_RELABELING_HPP

#include <opencv2/core.hpp>

#include "lsl3dlib/lsl/relabeling.hpp"

#include <simdhelpers/defs.hpp>
#include <simdhelpers/restrict.hpp>
#include "lsl3dlib/features.hpp"


#include "lsl3dlib/compat.hpp"
#include "lsl3dlib/lsl3d/lsl3d.hpp"


namespace algo {


template <typename SegmentWriteFun>
struct Relabeling_Z_Generic {

    struct Conf {
	using Seg_t = typename SegmentWriteFun::Conf::Seg_t;
	using Label_t = typename SegmentWriteFun::Conf::Label_t;

	static constexpr bool DO_NOTHING = false;
    };

    
    template <typename ConfLSL, typename LabelsSolver>
    static inline void Relabel(LSL3D_CCL_t<ConfLSL, LabelsSolver>& ccl);
};

template <typename SegmentWriteFun> template <typename ConfLSL, typename LabelsSolver>
void Relabeling_Z_Generic<SegmentWriteFun>::Relabel(LSL3D_CCL_t<ConfLSL, LabelsSolver>& ccl) {
    
    int width = ccl.width;
    int height = ccl.height;
    int depth = ccl.depth;
    
    for (int16_t slice = 0; slice < depth; slice++) {
	for (int16_t row  = 0; row < height; row++) {
	    const int16_t* restrict RLCi = ccl.RLC[slice][row];
	    const int32_t* restrict ERAi = ccl.ERA[slice][row];	    
	    const int16_t segment_count = ccl.Lengths[slice][row];
	    int32_t* restrict dstrow = ccl.labels.template ptr<int32_t>(slice, row);
	    	    
	    int16_t segment_start = 0;
	    int16_t segment_end = 0;
	    
	    for (int er = 1; er < segment_count; er += 2) {
		uint16_t segment_id = er / 2;
		
		segment_start = RLCi[er - 1];
		SegmentWriteFun::Write(dstrow, 0, segment_end, segment_start);
		segment_end = RLCi[er];
	    
		uint32_t ea = ERAi[segment_id];
		uint32_t label = ccl.ET.GetLabel(ea);
		
		assert(ea <= width * height * depth);
		SegmentWriteFun::Write(dstrow, label, segment_start, segment_end);
	    }
	    SegmentWriteFun::Write(dstrow, 0, segment_end, width);
	}
    }
}


template <typename LabelsSolver>
uint32_t relabeling(cv::Mat1i& EA, int32_t*** ERA,
		   int16_t*** rlc, LabelsSolver& ET);

template <typename LabelsSolver>
uint32_t relabeling_v2(cv::Mat1i& EA, int32_t*** ERA,
		      int16_t*** rlc, LabelsSolver& ET);

template <typename LabelsSolver>
uint32_t relabeling_z(cv::Mat1i& EA, int32_t*** ERA,
		     int16_t*** rlc, LabelsSolver& ET);

// Same as previous except that the entire line is first set to 0 prior to writing lines
template <typename LabelsSolver>
uint32_t relabeling_z_2(cv::Mat1i& EA, int32_t*** ERA,
			int16_t*** rlc, LabelsSolver& ET);


// Same as previous except that the entire line is first set to 0 prior to writing lines
template <typename LabelsSolver>
uint32_t relabeling_z_border_2(cv::Mat1i& EA, int32_t*** ERA,
			    int16_t*** rlc, LabelsSolver& ET);

template <typename LabelsSolver>
uint32_t relabeling_z_no_era(cv::Mat1i& EA,
			    int16_t*** rlc, LabelsSolver& ET);

void relabel_nothing(cv::Mat1i &labels, int32_t*** ERA, int16_t*** RLC);

struct Relabeling_Nothing {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = true;
    };
    
    // Not declared as inline: if LTO is not enabled then compiler won't be able to optimize away
    // the call. This is needed for measuring cost of RLE
    template <typename ConfLSL, typename LabelsSolver>
    static void Relabel(LSL3D_CCL_t<ConfLSL, LabelsSolver>& ccl) {
	relabel_nothing(ccl.labels, ccl.ERA, ccl.RLC);
    }
};

struct Relabeling {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET) {
	relabeling(labels, ERA, RLC, Lengths, ET);
    }
};

struct Relabeling_V2 {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET) {
	relabeling_v2<LabelsSolver>(labels, ERA, RLC, Lengths, ET);
    }
};

struct Relabeling_Z {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET) {
	relabeling_z<LabelsSolver>(labels, ERA, RLC, Lengths, ET);
    }
};


struct Relabeling_Z_V2 {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET) {
	relabeling_z_2<LabelsSolver>(labels, ERA, RLC, Lengths, ET);
    }
};



struct Relabeling_Z_Border_V2 {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET);
};

struct Relabeling_Z_NOERA {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>
    static inline void Relabel(cv::Mat1i& labels, int16_t*** RLC, int16_t** Lengths, LabelsSolver& ET) {
	relabeling_z_no_era<LabelsSolver>(labels, RLC, Lengths, ET);
    }
};


struct WriteSegmentFill {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    static inline void Write(Conf::Label_t* restrict line, Conf::Label_t label,
			     Conf::Seg_t segment_start, Conf::Seg_t segment_end) {
	std::fill(line + segment_start, line + segment_end, label);
    }
};

using Relabeling_Z_Border = Relabeling_Z_Generic<WriteSegmentFill>;


template <typename LabelsSolver>
uint32_t relabeling(cv::Mat1i& labels, int32_t*** ERA,
		    int16_t*** RLC, int16_t** Lengths, LabelsSolver& ET) {
    int width, height, depth;
    int rowstride, slicestride;

    GetMatSize(labels, width, height, depth);
    GetMatStrides<uint32_t>(labels, rowstride, slicestride);    

    
    uint32_t nea = 0;
    

    for (uint16_t slice = 0; slice < depth; slice++) {
	for (uint16_t row  = 0; row < height; row++) {

	    const int16_t* RLCi = RLC[slice][row];
	    const int32_t* ERAi = ERA[slice][row];
	    const uint16_t segment_count = Lengths[slice][row];
	    int32_t* dstrow = labels.ptr<int32_t>(slice, row);
	    
	    uint16_t segment_start = 0;
	    uint16_t segment_end = -1;

	    std::fill(RLCi, RLCi + width, 0);
	    for (uint16_t col = 1; col < segment_count; col += 2) {
		uint16_t segment_id = col / 2;

		segment_start = RLCi[col - 1];		
		segment_end = RLCi[col];
	    
		uint32_t label = ERAi[segment_id];
		uint32_t ea = ET.GetLabel(label);
				
		assert(ea <= width * height * depth);
		std::fill(dstrow + segment_start, dstrow + segment_end + 1, ea);
		//lsl_3d_draw_segment(dstrow, ea, segment_start, segment_end);
	    }
	}
    }
    return nea;
}


template <typename LabelsSolver>
inline uint32_t relabeling_z(cv::Mat1i& labels, int32_t*** ERA,
			     int16_t*** RLC, int16_t** Lengths, LabelsSolver& ET) {
    uint32_t nea = 0;
    
    int width, height, depth;
    int rowstride, slicestride;

    GetMatSize(labels, width, height, depth);
    GetMatStrides<uint32_t>(labels, rowstride, slicestride);    

        
    for (uint16_t slice = 0; slice < depth; slice++) {
	for (uint16_t row  = 0; row < height; row++) {

	    const int16_t* restrict RLCi = RLC[slice][row];
	    const int32_t* restrict ERAi = ERA[slice][row];
	    int32_t* restrict dstrow = labels.ptr<int32_t>(slice, row);
	    const uint16_t segment_count = Lengths[slice][row];
	    
	    uint16_t segment_start = 0;
	    uint16_t segment_end = 0;

	    //std::fill(dstrow, dstrow + width, 0);
	    for (uint16_t col = 1; col < segment_count; col += 2) {
		uint16_t segment_id = col / 2;
		
		segment_start = RLCi[col - 1];
		lsl::WriteSegmentScalar::Write(dstrow, 0, segment_end, segment_start);
		segment_end = RLCi[col];
		
		uint32_t label = ERAi[segment_id];
		uint32_t ea = ET.GetLabel(label);
		
		assert(ea <= width * height * depth);
		lsl::WriteSegmentScalar::Write(dstrow, ea, segment_start, segment_end);
	    }
	    lsl::WriteSegmentScalar::Write(dstrow, 0, segment_end, width);
	}
    }
    return nea;
}


// Step 3+5
template <typename LabelsSolver>
uint32_t relabeling_v2(cv::Mat1i& labels, int32_t*** ERA,
		       int16_t*** RLC, int16_t** Lengths, LabelsSolver& ET) {
    int width, height, depth;
    int rowstride, slicestride;

    GetMatSize(labels, width, height, depth);
    GetMatStrides<uint32_t>(labels, rowstride, slicestride);    

    
    uint32_t nea = 0;
        
    for (uint16_t slice = 0; slice < depth; slice++) {
	for (uint16_t row  = 0; row < height; row++) {

	    const int16_t* restrict RLCi = RLC[slice][row];
	    const int32_t* restrict ERAi = ERA[slice][row];
	    int32_t* restrict dstrow = labels.ptr<int32_t>(slice, row);
	    const uint16_t segment_count = Lengths[slice][row];
	    
	    uint16_t segment_start = 0;
	    uint16_t segment_end = -1;

	    for (uint16_t col = 1; col < segment_count; col += 2) {
		uint16_t segment_id = col / 2;

		segment_start = RLCi[col - 1];
		std::fill(dstrow + segment_end + 1, dstrow + segment_start, 0);
		segment_end = RLCi[col];
	    
		uint32_t label = ERAi[segment_id];
		uint32_t ea = ET.GetLabel(label);
	    
		assert(ea <= width * height * depth);
		std::fill(dstrow + segment_start, dstrow + segment_end + 1, ea);
		//lsl_3d_draw_segment(dstrow, ea, segment_start, segment_end);
	    }
	    std::fill(dstrow + segment_end + 1, dstrow + width, 0);	    
	}
    }
    return nea;
}


// Merge for STDZ version + handle RLC borders
template <typename LabelsSolver>
uint32_t relabeling_z_no_era(cv::Mat1i& labels, int16_t*** RLC, int16_t** Lengths, LabelsSolver& ET) {
    
    int width, height, depth;
    int rowstride, slicestride;

    GetMatSize(labels, width, height, depth);
    GetMatStrides<uint32_t>(labels, rowstride, slicestride);    
    
    uint32_t nea = 0;
    
	    
    uint32_t uf_offset = 1;
    
    for (uint16_t slice = 0; slice < depth; slice++) {
		
	for (uint16_t row  = 0; row < height; row++) {

	    const int16_t* restrict RLCi = RLC[slice][row];
	    const uint16_t segment_count = Lengths[slice][row];
	    int32_t* restrict dstrow = labels.ptr<int32_t>(slice, row);
	    
	    uint16_t segment_start = 0;
	    uint16_t segment_end = 0;
	    
	    for (uint16_t col = 1; col < segment_count; col += 2) {
		uint16_t segment_id = col / 2;
		
		segment_start = RLCi[col - 1];
		std::fill(dstrow + segment_end, dstrow + segment_start, 0);
		segment_end = RLCi[col];


		uint32_t label = ET.GetLabel(uf_offset + segment_id);
		
		
		std::fill(dstrow + segment_start, dstrow + segment_end, label);
	    }
	    std::fill(dstrow + segment_end, dstrow + width, 0);

	    uf_offset += (segment_count / 2);
	}
    }
    return nea;
}


template <typename LabelsSolver>
void Relabeling_Z_Border_V2::Relabel(cv::Mat1i& labels, int32_t*** ERA, int16_t*** RLC,
				     int16_t** Lengths, LabelsSolver& ET) {
    int width, height, depth;
    int rowstride, slicestride;

    GetMatSize(labels, width, height, depth);
    GetMatStrides<uint32_t>(labels, rowstride, slicestride);    

    
    for (int slice = 0; slice < depth; slice++) {	
	for (int row = 0; row < height; row++) {
	    const int16_t* restrict RLCi = RLC[slice][row];
	    const int32_t* restrict ERAi = ERA[slice][row];
	    const int16_t rlen = Lengths[slice][row];
	    int32_t* restrict dstrow = labels.ptr<int32_t>(slice, row);
	    
	    uint16_t seg_start = 0;
	    uint16_t seg_end = 0;
	    for (int er = 1; er < rlen; er += 2) {
	        seg_start = RLCi[er - 1];
		uint32_t l = ERAi[er / 2];
		l = ET.GetLabel(l);	       
		seg_end = RLCi[er];
		
		for (uint16_t col = seg_start; col < seg_end; col++) {
		    dstrow[col] = l;
		}		
	    }
	    	    
	}	
    }
}


}



#endif // CCL_ALGOS_3D_RELABELING_HPP
