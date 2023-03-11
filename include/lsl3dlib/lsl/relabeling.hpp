#ifndef CCL_ALGOS_RELABELING_HPP
#define CCL_ALGOS_RELABELING_HPP

#include <opencv2/core.hpp>
#include <cstdint>
#include <simdhelpers/restrict.hpp>
#include "lsl3dlib/lsl/write-segment.hpp"
#include "lsl.hpp"


namespace lsl {


template <typename SegmentWriteFun>
struct Relabeling_Z_Generic {

    struct Conf {
	using Seg_t = typename SegmentWriteFun::Conf::Seg_t;
	using Label_t = typename SegmentWriteFun::Conf::Label_t;
    };

    template <typename ConfLSL, typename LabelsSolver>
    static inline void Relabel(LSL_CCL_t<ConfLSL, LabelsSolver>& lsl);
};

template <typename SegmentWriteFun> template <typename ConfLSL, typename LabelsSolver>
void Relabeling_Z_Generic<SegmentWriteFun>::Relabel(LSL_CCL_t<ConfLSL, LabelsSolver>& lsl) {
    int width = lsl.width;
    int height = lsl.height;
    
    for (int row = 0; row < height; row++) {
	int16_t segment_count = lsl.Lengths[row];
	const typename Conf::Seg_t* restrict RLCi = lsl.RLC[row];
	const typename Conf::Label_t* restrict ERAi = lsl.ERA[row];

	typename Conf::Label_t* restrict dstline = lsl.labels.template ptr<int32_t>(row);
	
	typename Conf::Seg_t segment_start = 0;
	typename Conf::Label_t segment_end = 0;
	
	for (int er = 1; er < segment_count; er += 2) {
	    segment_start = RLCi[er - 1];

	    SegmentWriteFun::Write(dstline, segment_end, segment_start, 0);
	    segment_end = RLCi[er];
	    
	    typename Conf::Label_t label = ERAi[er / 2];
	    label = ET_GET_LABEL(lsl.ET, label);
	    
	    SegmentWriteFun::Write(dstline, segment_start, segment_end, label);
	}
	SegmentWriteFun::Write(dstline, segment_end, width, 0);
	
    }
}

using Relabeling_Z = Relabeling_Z_Generic<WriteSegmentScalar>;

}

#endif // CCL_ALGOS_RELABELING_HPP
