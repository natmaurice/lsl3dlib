#ifndef LSL3DLIB_LSL3D_LSL3D_HPP_
#define LSL3DLIB_LSL3D_LSL3D_HPP_

#include <cstdint>

#include <lsl3dlib/compat.hpp>


template <typename Conf, typename LabelsSolver>
struct LSL3D_CCL_t {
    
    
    typename Conf::Seg_t*** RLC = nullptr;
    int32_t*** ERA = nullptr;
    int16_t* ER = nullptr;
    int16_t** Lengths = nullptr;
    LabelsSolver ET;
    
    MAT3D_i32 labels;
    MAT3D_ui8 image;

    int width;
    int height;
    int depth;
};

#endif // LSL3DLIB_LSL3D_LSL3D_HPP_
