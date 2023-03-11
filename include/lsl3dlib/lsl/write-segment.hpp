#ifndef CCL_ALGOS_WRITE_SEGMENT_HPP
#define CCL_ALGOS_WRITE_SEGMENT_HPP


#include <opencv2/core.hpp>
#include <cstdint>
#include <simdhelpers/restrict.hpp>

namespace lsl {


struct WriteSegmentScalar {
        
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;
    };
    
    static inline void Write(int32_t* restrict line, int16_t segment_start, int16_t segment_end,
			     int32_t label) {
	for (int16_t i = segment_start; i <= segment_end; i++) {
	    line[i] = label;
	}
    }    
};



}

#endif // CCL_ALGOS_WRITE_SEGMENT_HPP
