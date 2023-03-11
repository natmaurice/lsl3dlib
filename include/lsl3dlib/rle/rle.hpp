#ifndef CCL_ALGOS_RLE_RLE_HPP
#define CCL_ALGOS_RLE_RLE_HPP

#include <cstdint>
#include <cstddef>

#include <simdhelpers/restrict.hpp>



namespace rle {

// These constants describe how much margin should each image row have.
// This is important to take into consideration as some algorithms (ie: the SIMD one)
// may perform out-of-bounds accesses (for implentation simplicity)
// As a result, input/output data should have extra allocated space
constexpr int16_t RLE_IMG_MARGIN_BEFORE = 1; // Image rows should have at least 1 element before
constexpr int16_t RLE_IMG_MARGIN_AFTER = 16; // Image rows should have at least 16 elements after
constexpr int16_t RLE_IMG_EXTRA_SPACE = RLE_IMG_MARGIN_BEFORE + RLE_IMG_MARGIN_AFTER;

constexpr int16_t RLE_ER_MARGIN_BEFORE = 0;
constexpr int16_t RLE_ER_MARGIN_AFTER = 16; // RLC rows should have 2 elements after
//constexpr int16_t RLE_ER_EXTRA_SPACE = RLE_RLC_MARGIN_BEFORE + RLE_RLC_MARGIN_AFTER;



inline int16_t rle_std_er(const uint8_t* restrict image_row,
			  int16_t* restrict rlc_row,
			  int16_t* restrict ER, int16_t width);

inline int16_t rle_rlc_er(const uint8_t* restrict image_row,
			  int16_t* restrict rlc_row,
			  int16_t* restrict ER, int16_t width);

inline int16_t rle_std(const uint8_t* restrict image_row,
		       int16_t* restrict rlc_row,
		       int16_t width);

inline int16_t rle_stdz_er(const uint8_t* restrict image_row,
			   int16_t* restrict rlc_row,
			   int16_t* restrict ER, int16_t width);


inline int16_t rle_stdz(const uint8_t* restrict image_row,
			int16_t* restrict rlc_row,
			int16_t width);


struct STD {
public:
    
    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = false;
	static constexpr size_t SIMD_WORDS = 1; // Number of 64-bits words per SIMD vector
    };

    template <uint8_t FG = 1>
    static inline int16_t Line(const uint8_t* restrict input, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width) {
	return rle_std(input, RLCi, width);
    }
};

struct STD_ER {
public:

    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = true;
	static constexpr size_t SIMD_WORDS = 1; // Number of 64-bits words per SIMD vector
    };
	
    template <uint8_t FG = 1>
    static inline int16_t Line(const uint8_t* restrict input, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, uint16_t width) {
	return rle_std_er(input, RLCi, ERi, width);
    }
};

struct STDZ {
public:

    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = false;
	static constexpr size_t SIMD_WORDS = 1; // Number of 64-bits words per SIMD vector
    };
    
    template <uint8_t FG = 1>
    static inline int16_t Line(const uint8_t* restrict input, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width) {
	return rle_stdz(input, RLCi, width);
    }
};

struct STDZ_ER {
public:

    struct Conf {
	static constexpr bool IsBitonal = false;
	using Seg_t = int16_t;
	static constexpr bool ER = true;
	static constexpr size_t SIMD_WORDS = 1; // Number of 64-bits words per SIMD vector
    };
    
    template <uint8_t FG = 1>
    static inline int16_t Line(const uint8_t* restrict input, Conf::Seg_t* restrict RLCi,
			       int16_t* restrict ERi, int16_t width) {
	return rle_stdz_er(input, RLCi, ERi, width);
    }
};



// Implementations
int16_t rle_std_er(const uint8_t* restrict image_row,
		   int16_t* restrict rlc_row,
		   int16_t* restrict ER, int16_t width) {
    int8_t prev = 0;
    int8_t front = 0;
    int8_t b = 0;
    int16_t er = 0;
    int8_t val;

    for (int16_t col = 0; col < width; col++) {
        val = image_row[col] & 1; // > 0 is only needed when processing image in YACCLAB
	front = val ^ prev;
	rlc_row[er] = col - b;
	b ^= front;
	er += front;
	ER[col] = er;
	prev = val;
    }
    val = 0;
    front = val ^ prev;
    rlc_row[er] = width - b;
    er += front;
    return er;
}


int16_t rle_std(const uint8_t* restrict image_row,
		int16_t* restrict rlc_row,
		int16_t width) {
    
    uint8_t prev = 0;
    uint8_t front = 0;
    uint8_t b = 0;
    int16_t er = 0;
    uint8_t val;

    for (int16_t col = 0; col < width; col++) {
	 // & 1 is only needed when processing image in YACCLAB as the input images are {0; 255}
	// rather than {0; 1}
        val = image_row[col] & 1;
	front = val ^ prev;
	rlc_row[er] = col - b;
	b ^= front;
	er += front;
	prev = val;
    }
    val = 0;
    front = val ^ prev;
    rlc_row[er] = width - b;
    er += front;

    // Border management: used to simplify  unfication step
    rlc_row[er] = INT16_MAX - 1;
    rlc_row[er + 1] = INT16_MAX - 1;

    return er;
}

int16_t rle_stdz_er(const uint8_t* restrict image_row,
		    int16_t* restrict rlc_row,
		    int16_t* restrict ER, int16_t width) {
    uint8_t prev = 0;
    uint8_t front = 0;
    int16_t er = 0;
    uint8_t val;
    
    for (uint16_t col = 0; col < width; col++) {
	// & 1 is only needed when processing image in YACCLAB as the input images are {0; 255}
	// rather than {0; 1}
        val = image_row[col] & 1; 
	front = val ^ prev;
	rlc_row[er] = col;
	er += front;
	ER[col] = er;
	prev = val;
    }
    if (prev != 0) {
	rlc_row[er] = width;
    }
    er += prev;

    // Border management: used to simplify  unfication step
    rlc_row[er] = INT16_MAX - 1;
    rlc_row[er + 1] = INT16_MAX - 1;
    
    return er;
}

int16_t rle_stdz(const uint8_t* restrict image_row,
		 int16_t* restrict rlc_row,
		 int16_t width) {
    
    uint8_t prev = 0;
    uint8_t front = 0;
    int16_t er = 0;
    uint8_t val;
    
    for (int16_t col = 0; col < width; col++) {
	// & 1 is only needed when processing image in YACCLAB as the input images are {0; 255}
	// rather than {0; 1}
        val = image_row[col] & 1; 
	front = val ^ prev;
	rlc_row[er] = col;
	er += front;
	prev = val;
    }
    if (prev != 0) {
	rlc_row[er] = width;
    }
    er += prev;

    // Border management: used to simplify  unfication step
    rlc_row[er] = INT16_MAX - 1;
    rlc_row[er + 1] = INT16_MAX - 1;

    return er;
}


int16_t rle_rlc_er(const uint8_t* restrict image_row,
		   int16_t* restrict rlc_row,
		   int16_t* restrict ER, int16_t width) {
    int8_t prev = 0;
    int8_t front = 0;
    int8_t b = 0;
    int16_t er = 0;
    int8_t val;

    for (int16_t col = 0; col < width; col++) {
        val = image_row[col];
	front = val ^ prev;
	if (front != 0){
	    rlc_row[er] = col - b;
	    b ^= 1;
	    er++;
	}
	ER[col] = er;
	prev = val;
    }
    val = 0;
    front = val ^ prev;
    rlc_row[er] = width - b;
    er += front;
    
    // Border management: used to simplify  unfication step
    rlc_row[er] = INT16_MAX - 1;
    rlc_row[er + 1] = INT16_MAX - 1;
    
    return er;
}


template <typename Derived>
struct RLE {
    void Execute(const uint8_t* restrict input, int16_t* restrict RLCi, int16_t* restrict ERi, int16_t width) {
	static_cast<Derived*>(this)->Execute(input, RLCi, ERi, width);
    }
};

}

#endif // CCL_ALGOS_RLE_RLE_HPP
