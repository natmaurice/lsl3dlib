#ifndef CCL_UTILITY_HPP
#define CCL_UTILITY_HPP

#include <cstdint>
#include <cstddef>
#include <cassert>


#include <utility>
#include <vector>


#include <simdhelpers/utils.hpp>
#include <simdhelpers/assume.hpp>


#if __cplusplus > 201703L
#define IF_CONSTEXPR if constexpr
#else
#define IF_CONSTEXPR if
#endif // __cplusplus



// Declarations

template<typename T>
void iota(T* data, T size);


inline unsigned isqrt(unsigned n) {
    unsigned i = 0;
    if (n == 0) return 0;
    
    while (i * i < n) i++;
    return i - 1;
}

template <typename U, typename T>
constexpr T dot(U arr[], T x0, T x1) {
    return arr[0] * x0 + arr[1] * x1;
}

template <typename U, typename T>
constexpr T dot(U arr[], T x0, T x1, T x2) {
    return arr[0] * x0 + arr[1] * x1 + arr[2] * x2;
}


template<typename T>
T positive_min_val();

// Implementations
uint16_t minp(uint16_t a, uint16_t b, uint16_t c, uint16_t d);

inline void touch(void* data, size_t size, size_t page_size = 4096) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(data);
    for (size_t i = 0; i < size; i += page_size) {
	bytes[i] = 0;
    }
    if (size > 0) {
	// If array isn't aligned to a page then its last element may be on a separate page
	// We therefore access it to ensure that it is allocated
	bytes[size - 1] = 0;
    }    
}


template<typename  T>
void iota(T* data, T size) {
    for (auto i = 0; i < size; i++) {
	data[i] = i;
    }    
}

template <typename T>
void bittonic_sort(T* data, size_t len);


inline size_t calc_stride(size_t len, size_t alignment) {
    assert(len > 0 && alignment > 0);
    size_t stride = roundup_kpow2(len, alignment);
    return stride;
}


double find_closest(std::vector<std::pair<double, double>> data, double val);



// ==================================================
// Implementations
// ==================================================


template <typename T>
void bittonic_sort(T* data, size_t len) {
    assert(false && "not implemented yet");
}


#endif // CCL_UTILITY_HPP
