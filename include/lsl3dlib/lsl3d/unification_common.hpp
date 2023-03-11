#ifndef CCL_ALGOS_3D_UNIFICATION_COMMON_HPP
#define CCL_ALGOS_3D_UNIFICATION_COMMON_HPP

#include <cstdint>
#include <cassert>
#include <limits>

#include <simdhelpers/restrict.hpp>

constexpr uint32_t TEMP_LABEL = std::numeric_limits<int32_t>::max();



template <typename Seg_t, typename Label_t>
struct AdjState {

    
    
    Seg_t* restrict RLC0 = nullptr;
    Seg_t* restrict RLC1 = nullptr;
    Seg_t* restrict RLC2 = nullptr;
    Seg_t* restrict RLC3 = nullptr;

    Label_t* restrict ERA0 = nullptr;
    Label_t* restrict ERA1 = nullptr; 
    Label_t* restrict ERA2 = nullptr;
    Label_t* restrict ERA3 = nullptr;
    
    int16_t* restrict ER0 = nullptr;
    int16_t* restrict ER1 = nullptr;
    int16_t* restrict ER2 = nullptr;
    int16_t* restrict ER3 = nullptr;

    int16_t len0 = 0;
    int16_t len1 = 0;
    int16_t len2 = 0;
    int16_t len3 = 0;

    // Double Lines
    Seg_t* restrict l_RLC0 = nullptr;
    Seg_t* restrict l_RLC1 = nullptr;
    Label_t* restrict l_ERA0 = nullptr;
    Label_t* restrict l_ERA1 = nullptr;

    int16_t l_len0 = 0;
    int16_t l_len1 = 0;

    // For Unification without ERA    
    Label_t uf_offset = 0;
    Label_t uf_offset0 = 0;
    Label_t uf_offset1 = 0;
    Label_t uf_offset2 = 0;
    Label_t uf_offset3 = 0;
};

// Some utility functions to make unification code clearer
template <typename Seg_t>
inline void next_er(int16_t& er, Seg_t& j0, Seg_t& j1, const Seg_t* restrict rlc_row) {
    j0 = rlc_row[er - 1];
    j1 = rlc_row[er];
    er += 2;
}

template <typename Seg_t>
inline void next_erb(int16_t& er, Seg_t& j0, Seg_t& j1, const Seg_t* restrict rlc_row) {
    er += 2;
    j0 = rlc_row[er - 1];
    j1 = rlc_row[er];
}

template <typename Label_t>
inline void write_era(int16_t er, Label_t label, Label_t* era_row) {
    era_row[er / 2] = label;
}

template <typename Seg_t, typename Label_t>
inline void write_temp(int16_t ert, Label_t label, Seg_t j0t, Seg_t j1t, Seg_t* restrict rlc_rowt,
		       Label_t* restrict era_rowt) {
    assert(j0t < j1t);
    assert(label > 0);
    
    rlc_rowt[ert - 1] = j0t;
    rlc_rowt[ert] = j1t;
    era_rowt[ert / 2] = label;
}


namespace unify {

static constexpr bool UseCounter = false;

}

#endif // CCL_ALGOS_3D_UNIFICATION_COMMON_HPP
