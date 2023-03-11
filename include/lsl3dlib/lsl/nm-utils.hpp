#ifndef CCL_ALGOS_NM_UTILS_HPP
#define CCL_ALGOS_NM_UTILS_HPP


template <typename T>
inline void minmax(T& a, T& b) {
    T min = a < b ? a : b;
    T max = a < b ? b : a;
    
    a = min;
    b = max;
}


#endif // CCL_ALGOS_NM_UTILS_HPP
