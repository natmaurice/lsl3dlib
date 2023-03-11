#ifndef CCL_ALGOS_3D_RELABELING_SSE_HPP
#define CCL_ALGOS_3D_RELABELING_SSE_HPP

#include <opencv2/core.hpp>

#include <lsl3dlib/lsl/relabeling.hpp>

#include <simdhelpers/defs.hpp>
#include <simdhelpers/restrict.hpp>

#ifdef __SSE4_2__
#include <x86intrin.h>
#include <emmintrin.h>
#endif // __SSE4_2__

#include <lsl3dlib/compat.hpp>

#include <lsl3dlib/lsl3d/relabeling.hpp>
#include <simdhelpers/utils-sse.hpp>
#include <simdhelpers/simd-wrapper.hpp>


namespace algo {
namespace sse {

struct WriteSegmentSSE {
    
    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;
    };
    
    static inline void Write(Conf::Label_t* restrict line, Conf::Label_t label,
			     Conf::Seg_t segment_start, Conf::Seg_t segment_end) {
	__m128i val = _mm_set1_epi32(label);
	for (int16_t i = segment_start; i < segment_end; i += 4) {
	    _mm_storeu_si128((__m128i*)(line + i), val);
	}
    }
};



using Relabeling_Z = algo::Relabeling_Z_Generic<WriteSegmentSSE>;

/*struct Relabeling_Z {
    static inline void Relabel(cv::Mat1i& labels, ERATable& ERA, RLCTable& RLC,
			       LabelsSolver& ET) {
	sse::relabeling_z(labels, ERA, RLC, uf);
    }
    };*/


struct Relabeling_Z_V2 {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;
    };
    
    template <typename LabelsSolver>    
    static inline void Relabel(cv::Mat1i& labels, int32_t*** ERA, Conf::Seg_t*** RLC, int16_t** Lengths,
			       LabelsSolver& ET);
};

template <typename LabelsSolver>
void Relabeling_Z_V2::Relabel(MAT3D_i32& labels, int32_t*** ERA, Conf::Seg_t*** RLC, int16_t** Lengths,
			      LabelsSolver& ET) {
    
    int width, height, depth;

    GetMatSize(labels, width, height, depth);

    
    for (int slice = 0; slice < depth; slice++) {	
	for (int row = 0; row < height; row++) {

	    const Conf::Seg_t* restrict RLCi = RLC[slice][row];
	    const int32_t* restrict ERAi = ERA[slice][row];
	    int rlen = Lengths[slice][row];
	    Conf::Label_t* restrict dstrow = MAT3D_PTR(labels, Conf::Label_t, slice, row);
	    
	    uint16_t seg_start = 0;
	    uint16_t seg_end = 0;
	    for (int er = 1; er < rlen; er += 2) {
		_mm_storeu_si128((__m128i*)(dstrow + seg_end), _mm_setzero_si128());

	        seg_start = RLCi[er - 1];
		Conf::Label_t l = ERAi[er / 2];
		l = ET.GetLabel(l);
		seg_end = RLCi[er];
	        
		//write_segment_sse(dstrow, l, seg_start, seg_end);
		__m128i val = _mm_set1_epi32(l);
		for (int col = seg_start; col < seg_end; col += 4) {
		    _mm_storeu_si128((__m128i*)(dstrow + seg_end), val);
		}
	    }
	    for (int c = seg_end; c < width; c++) {
		dstrow[c] = 0;
	    }	    
	}	
    }
}


struct Relabeling_Pixel {

    struct Conf {
	using Seg_t = int16_t;
	using Label_t = int32_t;

	static constexpr bool DO_NOTHING = false;

	static constexpr size_t SIMD_WORDS = 2;
    };
    
    template <typename ConfLSL, typename LabelsSolver>
    static inline void Relabel(LSL3D_CCL_t<ConfLSL, LabelsSolver>& ccl) {

	int width = ccl.width;
	int height = ccl.height;
	int depth = ccl.depth;


	//std::cout << "\n=== Relabeling Pixel ===\n";
	constexpr int16_t TILE_W = 4;
	
	for (int slice = 0; slice < depth; slice++) {
	    for (int row = 0; row < height; row++) {

		const uint8_t* restrict srcrow = ccl.image.template ptr<uint8_t>(slice, row);
		int32_t* restrict ERAi = ccl.ERA[slice][row];
		int32_t* restrict dstrow = ccl.labels.template ptr<int32_t>(slice, row);
		const int16_t len = ccl.Lengths[slice][row] / 2;
		
		int n = 0;

		// Prolog
		__m128i vea;
		for (int i = 0; i < len; i++) { // Gather parent labels
		    int32_t l = ERAi[i];
		    //std::cout << "l = " << l << " ";
		    ERAi[i] = ET_GET_LABEL(ccl.ET, l);
		}
		//std::cout << "\n";
		vea = _mm_loadu_si128((__m128i*)(ERAi));
		//vea = _mm_cvtepu32_epi64(vea);
		//vea = _mm_slli_si128(vea, 4);
		
		//std::cout << "(" << slice << ", " << row << ") vea = " << SIMDWrapper<4>(vea) << "\n";

		
		// Main loop
		__m128i last = _mm_set1_epi8(0);
		for (int col = 0; col < width; col += 16) {
		    __m128i in = _mm_load_si128((__m128i*)(srcrow + col));
		    __m128i f = _mm_xor_si128(in, ::sse::vec_right_8x16(last, in));

		    last = in;
		    
		    //std::cout << "       in = " << SIMDWrapper<1>(in) << "\n";
		    //std::cout << "       f  = " << SIMDWrapper<1>(f) << "\n";

		    int mask = ::sse::movemask_8x16(f);

		    // Calculate local ER

		    __m128i shufm = _mm_set_epi8(3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0);
		    __m128i incr4 = _mm_set1_epi8(4);

		    // Convert to positive numbers
		    __m128i fpos = _mm_abs_epi8(f);
		    
		    // Treat as 32 bits and shift by 8 bits (and then 16 bits)
		    // _mm_slli_si128 is not used here because computing partial prefix sum on
		    // 4x 8 bits words

		    __m128i f_ppsum = _mm_add_epi8(fpos, _mm_slli_epi32(fpos, 8));
		    f_ppsum = _mm_add_epi8(f_ppsum, _mm_slli_epi32(f_ppsum, 16));

		    //std::cout << "       F_PPSUM = " << SIMDWrapper<1>(f_ppsum) << "\n";

		    
		    f_ppsum = _mm_and_si128(f_ppsum, _mm_set1_epi8(0xfe)); // Remove last bit
		    //f_ppsum = _mm_and_si128(f_ppsum, _mm_set1_epi8(3)); // Modulo 4
		    // Multiply each bytes by 4 using bit shifting
		    // prior to individual conversion to 4x8 bits
		    f_ppsum = _mm_slli_epi32(f_ppsum, 1);
		    
		    // Set -1 where pixel=0
		    // negative value => shuffle destination (label) set to 0 (what we want here)
		    f_ppsum = _mm_or_si128(f_ppsum, _mm_andnot_si128(in, _mm_set1_epi8(-4)));

		    //std::cout << "       F_PPSUM = " << SIMDWrapper<1>(f_ppsum) << "\n";
		    
		    for (int J = 0; J < 4; J++) {

			if (col + J * 4 >= width) {
			    break;
			}
			
			// 1) Extract low 4x8bits integers from f
			__m128i f0 = _mm_shuffle_epi8(f_ppsum, shufm);

			//f0 = _mm_sub_epi8(f0, _mm_set1_epi8(2 * nrel));
			
			int cnt0 = __builtin_popcount(mask & 0xf);

			//std::cout << "       f<" << J << "> = " << SIMDWrapper<1>(f0) << "\n";
			
			// Compute prefix sum of f0
			const __m128i incr = _mm_set_epi8(3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0);
			__m128i er = _mm_add_epi8(f0, incr);
			
			//std::cout << "       er<" << J << "> = " << SIMDWrapper<1>(er) << "\n";

			
			// Get label
			__m128i l0 = _mm_shuffle_epi8(vea, er);
			//std::cout << "       vea<" << J << "> = " << SIMDWrapper<4>(vea) << "\n";
			//std::cout << "       l0<" << J << "> = " << SIMDWrapper<4>(l0) << "\n";

			
			_mm_store_si128((__m128i*)(dstrow + col + J * TILE_W), l0);


			int n2 = n + cnt0;

			//std::cout << "       n<" << J << "> = " << n << ", cnt0 = " << cnt0 << ", n2 = "<< n2 << "\n";
			
			vea = _mm_loadu_si128((__m128i*)(ERAi + n2/2));
			//vea = _mm_cvtepu32_epi64(vea);

			//if (!(n2 & 1)) {
			//    vea = _mm_slli_si128(vea, 4);
			//}
			
			mask >>= 4;

			shufm = _mm_add_epi8(shufm, incr4);
			n = n2;
			//std::cout << "\n";
		    }
		    
		}
	    }
	}
    }
};

}
}


#endif // CCL_ALGOS_3D_RELABELING_HPP
          
