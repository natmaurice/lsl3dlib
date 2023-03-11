#ifndef CCL_ALGOS_LSL_UTILS_HPP
#define CCL_ALGOS_LSL_UTILS_HPP

#include <cstdint>
#include <limits>

namespace algo {

constexpr uint16_t to_era_index(uint32_t segment_id) {
    return (segment_id - 1) / 2;
}

constexpr uint16_t MAX_UINT16 = std::numeric_limits<int16_t>::max();

// Count the number of intersections between segments in 2 lines.
// Each line must be terminated by a INT16_MAX segment
// This is fairly similar to what the unification merge does but lighter
// since it does not take labels into account
int16_t count_intersections(const int16_t* rlc0, int16_t len0,
			    const int16_t* rlc1, int16_t len1);

// Calculate the number of foreground pixels from RLE segments
// This is useful when calculating statistics on the image witout iterating twice on it
int16_t count_foreground(const int16_t* rlc0, int16_t len);

}

#endif // CCL_ALGOS_LSL_UTILS_HPP
