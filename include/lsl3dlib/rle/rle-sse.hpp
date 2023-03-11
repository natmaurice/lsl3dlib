#ifndef CCL_ALGOS_RLE_X86_HPP
#define CCL_ALGOS_RLE_X86_HPP

#ifndef __x86_64__
#error "X86-64 included despite non-x86-64 platform"
#endif // __x86_64__

#include <cstdint>
#include <x86intrin.h>
#include <immintrin.h>
#include <emmintrin.h>

#include <cassert>


#include <lsl3dlib/utility.hpp>

#include <simdhelpers/utils-sse.hpp>
#include <simdhelpers/compress/compress-sse.hpp>
#include <simdhelpers/restrict.hpp>

namespace rle {

extern unsigned char LUT16x8[256 * 16] __attribute__ ((aligned (16)));

#ifdef __SSE4_2__

namespace sse {

extern unsigned char LUT8x16[256 * 256 * 16] __attribute__((aligned(16)));

inline int lut_init_8x16() {
    for (int i = 0; i < 65536; i++) {

	unsigned char* lut = &LUT8x16[i * 16];
	
	int mask = i;
	int j = 0;
        for (int k = 0; k < 16; k++) {
	    lut[j] = k;
	    if (mask & 1) {
		j++;
	    }
	    mask >>= 1;
	}
        for (; j < 16; j++) {
	    lut[j] = 255;
	}
    }
    return 0;    
}

inline int16_t* compress_and_store_sse4(__m128i entry, __m128i ids, int16_t* restrict rlc_row) {
    // Convert 16 bits 'front' data back to 8 bits.
    // This does not create any data loss issue as 1/2 bytes aren't actually useful
    entry = _mm_packus_epi16(entry, _mm_setzero_si128()); 
    int mask = _mm_movemask_epi8(entry);

    __m128i shuffle = ((__m128i*)rle::LUT16x8)[mask]; // Get shuffle from LUT
    __m128i rlc_entry = _mm_shuffle_epi8(ids, shuffle);
    _mm_storeu_si128((__m128i*)rlc_row, rlc_entry);
    return rlc_row + __builtin_popcount(mask);
}

inline uint16_t compress_8x16_sse(__m128i vmask, __m128i ids, __m128i& res) {
    int mask = _mm_movemask_epi8(vmask);
	
    __m128i shuffle = ((__m128i*)rle::sse::LUT8x16)[mask]; // Get shuffle from LUT
    res = _mm_shuffle_epi8(ids, shuffle);
    return __builtin_popcount(mask);

}

inline uint16_t compress_16x16(__m128i entry, __m128i ids, __m128i& res) {
    int mask = _mm_movemask_epi8(entry);
    __m128i shuffle = ((__m128i*)rle::sse::LUT8x16)[mask];
    res = _mm_shuffle_epi8(ids, shuffle);
    return __builtin_popcount(mask);
}

inline int16_t rle_stdz_sse4(const uint8_t* restrict image_row,
			     int16_t* restrict rlc_row,
			     int16_t width);
template<uint8_t FG = 1>
inline int16_t rle_stdz_er_sse4(const uint8_t* restrict image_row,
				int16_t* restrict RLC,
				int16_t* restrict ER,
				int16_t width);

struct STDZ {

    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = false;
	static constexpr size_t SIMD_WORDS = 2; // Number of 64-bits words per SIMD vector
    };

    template<uint8_t FG = 1>
    static inline uint16_t Line(const uint8_t* restrict image_row, Conf::Seg_t* restrict RLCi,
				int16_t* restrict ERi, int16_t width) {
	return rle_stdz_sse4(image_row, RLCi, width);
    }
};

struct STDZ_V2 {
    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = false;
	static constexpr size_t SIMD_WORDS = 2; // Number of 64-bits words per SIMD vector
    };

    template<uint8_t FG = 1>
    static inline int16_t Line(const uint8_t* restrict image_row, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width);
};


struct STDZ_V3 {

    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = false;
	static constexpr size_t SIMD_WORDS = 2; // Number of 64-bits words per SIMD vector
    };
    
    template <uint8_t FG>    
    static inline int16_t Line(const uint8_t* restrict input, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width);
};



struct STDZ_ER {
    
    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = true;
	static constexpr size_t SIMD_WORDS = 2; // Number of 64-bits words per SIMD vector
    };

    template <uint8_t FG = 1>
    static inline uint16_t Line(const uint8_t* restrict image_row, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width) {
	return rle_stdz_er_sse4<FG>(image_row, RLCi, ERi, width);
    }
};


inline int16_t rle_stdz_sse4(const uint8_t* restrict image_row,
			     int16_t* restrict rlc_row,
			     int16_t width) {    
    int16_t* old_rlc_row = rlc_row;
    __m128i last = _mm_set1_epi8(0);
    // Used to get proper line numbers after applying a mask
    __m128i ids = _mm_set_epi16(7, 6, 5, 4, 3, 2, 1, 0);
    __m128i increment = _mm_set1_epi16(8); // Used to increment previous vector

    const uint16_t max_width = width - width % 8;
    for (uint16_t i = 0; i < max_width; i += 8) {
	__m128i entry = _mm_loadl_epi64((__m128i*)(image_row + i)); // Load 8x8 elements (64 bits)
	entry = _mm_cvtepu8_epi16(entry); // Extends each element so that they occupy 16 bits each

	// Edge detection
	__m128i shift = _mm_alignr_epi8(entry, last, 14);
	__m128i f = _mm_cmpeq_epi8(entry, shift); // 1 step of XOR => create a mask
	f = _mm_xor_si128(f, _mm_set1_epi8(-1)); // Inverse data to complete XOR

	last = entry;

	// Perform compression and store indices in `rlc_row`
	rlc_row = compress_and_store_sse4(f, ids, rlc_row);
	assert(rlc_row - old_rlc_row <= width);
	
	ids = _mm_add_epi16(ids, increment);        
    }
    
    // Handle last elements using scalar RLE
    __m128i m = _mm_bsrli_si128(last, 14); // Move most significantl word to least significanta word
    uint8_t prev = _mm_cvtsi128_si64(m) & 0x1; 
    assert(prev == 0 || prev == 1);
    int16_t er = 0;
    uint16_t front;
    uint8_t val;
    for (int16_t col = max_width; col < width; col++) {
	val = image_row[col] & 1;
	front = val ^ prev;
	rlc_row[er] = col;
	er += front;
	prev = val;
    }
    rlc_row += er;
    if (prev > 0) {
	rlc_row[0] = width;
	rlc_row++;
    }

    int16_t n = rlc_row - old_rlc_row;
    
    // Border management: used to simplify  unfication step
    old_rlc_row[n] = INT16_MAX - 1;
    old_rlc_row[n + 1] = INT16_MAX - 1;
    
    return n; // Count number of elements
}

template <uint8_t FG>
int16_t STDZ_V2::Line(const uint8_t *restrict line, Conf::Seg_t *restrict RLC, int16_t* restrict ERi,
		      int16_t width) {
    
    __m128i last = _mm_setzero_si128();
    int16_t* restrict RLCi = RLC;
    
    __m128i index8a = _mm_set_epi16(7, 6, 5, 4, 3, 2, 1, 0);
    __m128i index8b = _mm_set_epi16(15, 14, 13, 12, 11, 10, 9, 8);
    
    __m128i incr16 = _mm_set1_epi16(16);

    
    for (int i = 0; i < width; i += 16){
	// Load 16 bytes from line.
	// While loading 32 bytes is possible with AVX2, this would make the following code more
	// complex (notably the compress).
	// Since the output array has 16 bits elements, this wouldn't change much.
	
	__m128i in = _mm_loadu_si128((__m128i*)(line + i));
	__m128i f;

	IF_CONSTEXPR (FG == 0xff) {
	    f = _mm_xor_si128(in, ::sse::vec_right_8x16(last, in));	    
	} else {
	    f = _mm_cmpeq_epi8(in, ::sse::vec_right_8x16(last, in));
	    f = _mm_xor_si128(f, _mm_set1_epi8(-1));
	}	
	
	__m128i rlc0, rlc1;

	//std::cout << "in = " << SIMDWrapper<1>(in) << "\n";
	//std::cout << "f = " << SIMDWrapper<1>(f) << "\n";
	int m = ::sse::movemask_8x16(f);
	int m0 = m & 0xFF;
	int m1 = m >> 8;
	//std::cout << "m0 = " << std::hex << m0
	//	  << ", m1 = " << m1
	//	  << std::dec << "\n";
	int popcnt0 = compress::sse::compress_16x8(index8a, m0, rlc0);
	int popcnt1 = compress::sse::compress_16x8(index8b, m1, rlc1);

	//std::cout << "rlc0 = " << SIMDWrapper<2>(rlc0) << "\n";
	//std::cout << "rlc1 = " << SIMDWrapper<2>(rlc1) << "\n\n";
	
	_mm_storeu_si128((__m128i*)RLCi, rlc0);
	_mm_storeu_si128((__m128i*)(RLCi + popcnt0), rlc1);
	
	index8a = _mm_add_epi16(index8a, incr16);
	index8b = _mm_add_epi16(index8b, incr16);
	
	last = in;
	RLCi += popcnt0 + popcnt1;
    }

    //std::cout << "\n";
    
    // Insert a segment end at the tail
    // If not done then a full line would not be properly processed
    *RLCi++ = width; 
    int n = RLCi - RLC;
    if (n % 2 == 1) n--;
    
    // Border management: used to simplify  unfication step
    RLC[n] = INT16_MAX - 1;
    RLC[n + 1] = INT16_MAX - 1;

    return n;

}

// STD16v3<_mm_LUT_compress_m8_epi8>
template <uint8_t FG>
int16_t STDZ_V3::Line(const uint8_t *restrict line, Conf::Seg_t *restrict RLC, int16_t* restrict ERi,
		      int16_t width) {
    
    int16_t *restrict RLCi = RLC;
    
    __m128i last = _mm_setzero_si128();
    // Used to get proper line numbers after applying a mask
    __m128i index16 = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
    __m128i incr16 = _mm_set1_epi16(16), J = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
    
    int i = 0;
    
    for (i = 0; i < width; i += 16) {
	__m128i in = _mm_loadu_si128((__m128i*)(line + i));
	__m128i f;
	if (FG == 0xff) {
            f = _mm_xor_si128(in, _mm_alignr_epi8(in, last, 15));
	} else {
            f = _mm_cmpeq_epi8(in, _mm_alignr_epi8(in, last, 15));
            f = _mm_xor_si128(f, _mm_set1_epi8(-1));
	}
	last = in;

	__m128i rlc = index16;
	int rlc_incr = compress_8x16_sse(f, rlc, rlc);

	__m128i lrlc, hrlc;
          
	lrlc = _mm_add_epi16(J, _mm_cvtepi8_epi16(rlc));
	hrlc = _mm_add_epi16(J, _mm_cvtepi8_epi16(_mm_srli_si128(rlc, 8)));
          
	_mm_storeu_si128((__m128i*)RLCi, lrlc);
	_mm_storeu_si128((__m128i*)(RLCi + 8), hrlc);

	RLCi += rlc_incr;
	J = _mm_add_epi16(J, incr16);
    }
    *RLCi++ = width;
    
    int n = RLCi - RLC;

    if (n % 2 == 1) n--;
    
    RLC[n] = INT16_MAX - 1;
    RLC[n + 1] = INT16_MAX - 1;
    
    //if (Conf::ERA) ERA[i+1] = ERA[i] + (n-1)/2;
    return n;
}

template<uint8_t FG>
inline int16_t rle_stdz_er_sse4(const uint8_t* restrict image,
			     int16_t* restrict RLC,
			     int16_t* restrict ER,
			     int16_t width) {
    
    int16_t *restrict ERi = ER;
    int16_t *restrict RLCi = RLC;
    
    __m128i last = _mm_setzero_si128();
    __m128i ver = _mm_setzero_si128();
    // Used to get proper line numbers after applying a mask
    __m128i index16 = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
    __m128i incr16 = _mm_set1_epi16(16), J = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);

    ERi[-1] = 0;
    
    int i = 0;
    
    for (i = 0; i < width; i += 16) {
          __m128i in = _mm_loadu_si128((__m128i*)(image + i));
          __m128i f;
          if (FG == 0xff) {
            f = _mm_xor_si128(in, _mm_alignr_epi8(in, last, 15));
          } else { 
            f = _mm_cmpeq_epi8(in, _mm_alignr_epi8(in, last, 15));
            f = _mm_xor_si128(f, _mm_set1_epi8(-1));
          }
          last = in;

          __m128i rlc = index16;
	  int rlc_incr = compress_8x16_sse(f, rlc, rlc);

          __m128i lfscan, hfscan, lver, hver, lrlc, hrlc;
          __m128i fscan = f;

	  // Prefix sum
	  fscan = _mm_add_epi8(fscan, _mm_slli_si128(fscan, 1));
	  fscan = _mm_add_epi8(fscan, _mm_slli_si128(fscan, 2));
	  fscan = _mm_add_epi8(fscan, _mm_slli_si128(fscan, 4));
	  lfscan = _mm_cvtepi8_epi16(fscan);
	  hfscan = _mm_add_epi8(fscan, _mm_srli_si128(fscan, 8));
	  hfscan = _mm_cvtepi8_epi16(hfscan);
          

          lrlc = _mm_add_epi16(J, _mm_cvtepi8_epi16(rlc));
          hrlc = _mm_add_epi16(J, _mm_cvtepi8_epi16(_mm_srli_si128(rlc, 8)));


	  lver = _mm_sub_epi16(ver, lfscan);
	  hver = _mm_sub_epi16(ver, hfscan);
	  _mm_storeu_si128((__m128i*)(ERi + i), lver);
	  _mm_storeu_si128((__m128i*)(ERi + i + 8), hver);
	  ver = _mm_shuffle_epi8(hver, _mm_set1_epi16(0x0f0e));
          
          _mm_storeu_si128((__m128i*)RLCi, lrlc);
          _mm_storeu_si128((__m128i*)(RLCi + 8), hrlc);

	  RLCi += rlc_incr;
          J = _mm_add_epi16(J, incr16);
    }
    *RLCi++ = width;
    
    int n = RLCi - RLC;

    if (n % 2 == 1) n--;
    
    RLC[n] = INT16_MAX - 1;
    RLC[n + 1] = INT16_MAX - 1;
    
    //if (Conf::ERA) ERA[i+1] = ERA[i] + (n-1)/2;
    return n;
}

}

#endif // __SSE4_2__

}

#endif // CCL_ALGOS_RLE_X86_HPP
