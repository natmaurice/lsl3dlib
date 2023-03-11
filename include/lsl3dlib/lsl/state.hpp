#ifndef CCL_ALGOS_STATE_HPP
#define CCL_ALGOS_STATE_HPP

#include "unionfind.hpp"
#include "rle_table.hpp"
#include "restrict.hpp"
#include "aligned_alloc.hpp"


template <class Config>
struct State {
    UnionFind<int32_t> uf;
    RLCTable RLC;
    ERATable ERA;        

    int32_t er_slice_pitch;
    int32_t er_row_pitch;


    int16_t* restrict ER;
    int16_t* restrict ERp0;
    int16_t* restrict ERp1;
    
    static constexpr size_t ALIGNMENT = 64;
    
    State();
    ~State();
    
    void Alloc(int32_t width, int32_t height, int32_t depth);
    void Free();
    void Setup(int16_t width, int16_t height, int16_t depth);
};




template <class Config>
State<Config>::State() : er_row_pitch(0), er_slice_pitch(0), ERp0(nullptr), ERp1(nullptr) {
}

template <class Config>
State<Config>::~State() {
    Free();
}

template <class Config>
void State<Config>::Alloc(int32_t width, int32_t height, int32_t depth) {

    int32_t size = width * height * depth;
    int32_t max_comp_count = size / 4 + height * depth + width + 1;
    if constexpr (!Config::ERA) {
	max_comp_count = size / 2 + 3;
    }
    uf.Resize(max_comp_count);
    
    constexpr int16_t EXTRA = RLE_RLC_EXTRA_SPACE;

    if constexpr (Config::RLC) {
	RLC.Reserve(width + EXTRA, height, depth);
    }

    if constexpr (Config::ERA) {
	ERA.Reserve(width + 2, height, depth);
    }

    if constexpr (Config::ER) {
	constexpr size_t ER_TOP_BORDER = 1;
	constexpr size_t ER_BOTTOM_BORDER = 1;
	constexpr size_t ER_BORDER = ER_TOP_BORDER + ER_BOTTOM_BORDER;

	er_row_pitch = calc_stride(width, ALIGNMENT);	
	er_slice_pitch = er_row_pitch * (height + ER_BORDER);
	
	ER = aligned_new<int16_t>(ALIGNMENT, 2 * er_slice_pitch);
	ERp0 = ER + er_row_pitch;
	ERp1 = ERp0 + er_slice_pitch;
    }
}

template <class Config>
void State<Config>::Free() {
    
    if constexpr (Config::RLC) {
	RLC.Free();
    }
    
    if constexpr (Config::ERA) {
	ERA.Free();
    }
    
    if constexpr (Config::ER) {
	aligned_delete(ER, ALIGNMENT);
	ERp0 = ERp1 = ER = nullptr;
    }
}

template <class Config>
void State<Config>::Setup(int16_t width, int16_t height, int16_t depth) {
    int16_t* data_ptr = RLC.Ptr(0);
    data_ptr[0] = INT16_MAX - 1; // Bigger than width: virtual line won't be taken into consideration
    data_ptr[1] = INT16_MAX - 1; // We substract 1 to avoid overflows if adding 1

    		    
    if constexpr (Config::RLC) {

	RLC.SetLineSize( -1, 0, 0);
	RLC.SetPtr(data_ptr, -1,  0);
    
	for (int16_t i = -1; i <= height; i++) {
	    // First slice
	    RLC.SetPtr(data_ptr, i, -1);
	    RLC.SetLineSize(i, -1, 0);
	}
	for (int16_t i = -1; i < depth; i++) {
	    // First rows
	    RLC.SetPtr(data_ptr, -1, i);
	    RLC.SetLineSize(-1, i, 0);
	}    
	RLC.SetPtr(data_ptr + 2, 0,  0);	
    }    	

    if constexpr (Config::ER) {
	// Set border of ER
	// Note: This is inefficient because only 1 plane + 2 borders have to be set to 0
	// rather than 0. 
	std::fill(ER, ER + 2 * er_slice_pitch, 0);	
    }
    
}

#endif // CCL_ALGOS_STATE_HPP
