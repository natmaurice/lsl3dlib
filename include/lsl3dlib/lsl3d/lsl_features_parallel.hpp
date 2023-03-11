#ifndef CCL_ALGOS_3D_LSL_FEATURES_PARALLEL_HPP
#define CCL_ALGOS_3D_LSL_FEATURES_PARALLEL_HPP

#include "features.hpp"
#include <simdhelpers/restrict.hpp>

struct FeaturesCalc_Parallel_None {
    
    template <typename LabelsSolver, typename ConfFeatures>
    static void CalcFeatures(int16_t*** RLC, int32_t*** ERA, int16_t** Lengths, LabelsSolver& ET,
		      Features& features, size_t label_count, int depth, int height, int width) {
    }

};


#endif // CCL_ALGOS_3D_LSL_FEATURES_PARALLEL_HPP
