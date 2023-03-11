#ifndef CCL_CV_UTILS_HPP
#define CCL_CV_UTILS_HPP

/*
 * This file defines several utility functions that are meant to ease the manipulation of OpenCV 
 * data structure, especially for 3D matrices
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <iostream>
#include <random>
#include <type_traits>


#include <simdhelpers/utils.hpp>
#include <simdhelpers/restrict.hpp>
#include <simdhelpers/term_utils.hpp>


#if LSL3D_USE_OpenCV || 1

typedef cv::Mat1b MAT3D_ui8;
typedef cv::Mat1i MAT3D_i32;

#define MAT_PTR(mat, type, r) (mat.ptr<type>(r))
#define MAT3D_PTR(mat, type, s, r) (mat.ptr<type>(s, r))

#else
typedef uint8_t*** MAT3D_ui8;
typedef int32_t***  MAT3D_i32;

#define MAT_PTR(mat, type, r) (mat[s][r])
#define MAT3D_PTR(mat, type, s, r) (mat[s][r])

#endif // LSL3D_USE_OpenCV



#if LSL3D_USE_YACCLAB || 1

#define ET_GET_LABEL(ET, label) (ET.GetLabel(label))

#endif 



struct Size3D {
    int width;
    int height;
    int depth;
};


inline Size3D Get3DMatrixSize(const cv::Mat& mat) {
    Size3D size;

    if (mat.dims == 3) {
	size.depth = mat.size.p[0];
	size.height = mat.size.p[1];
	size.width = mat.size.p[2];
    } else {
	assert(mat.dims == 2);
	size.depth = 1;
	size.height = mat.rows;
	size.width = mat.cols;
    }
    return size;    
}

inline void GetMatSize(const cv::Mat& mat, int& width, int& height, int& depth) {
    if (mat.dims == 3) {
	width = mat.size.p[2];
	height = mat.size.p[1];
	depth = mat.size.p[0];
    } else {
	width = mat.cols;
	height = mat.rows;
	depth = 1;
    }
	
}

template <typename T>
inline void GetMatStrides(const cv::Mat& mat, int& rowstride, int& slicestride) {
    rowstride = mat.step[1] / sizeof(T);
    slicestride = mat.step[0] / sizeof(T);
}

template <typename T>
cv::Mat create_mat_with_border(cv::Mat& mat, int width, int height, int xbefore, int ybefore, int xafter, int yafter) {

    cv::Mat parent;
    
    constexpr size_t ALIGNMENT = 64 / sizeof(T);
    static_assert(sizeof(T) <= ALIGNMENT, "Can not align data type");
    
    xbefore = roundup_kpow2(xbefore, ALIGNMENT); 
    int xstride = xbefore + roundup_kpow2((width + xafter), ALIGNMENT);
    
    int parent_width = xstride;
    int parent_height = height + ybefore + yafter;
    
    static_assert(std::is_signed<float>::value, "Fuck compiler bugs");

    if (std::is_signed<T>::value) {	
	parent.create(parent_height, parent_width, cv::DataType<T>::type);
    } else {
	using U = typename std::make_signed<T>::type;
	parent.create(parent_height, parent_width, cv::DataType<U>::type);
    }
    parent.setTo(0);
    
    std::vector<cv::Range> rect;
    rect.push_back(cv::Range(ybefore, height + ybefore));
    rect.push_back(cv::Range(xbefore, width  + xbefore));

    mat = parent(rect);
    
    return mat;
}


template <typename T>
void create_mat_with_border(cv::Mat& mat, int width, int height, int depth,
			       int xbefore, int ybefore, int zbefore, int xafter, int yafter, int zafter) {
    
    cv::Mat parent;

    // Ensures that the beginning of each line in mat is aligned to a cache line
    constexpr size_t ALIGNMENT = 64 / sizeof(T);
    static_assert(sizeof(T) <= ALIGNMENT, "Can not align data type");
    
    xbefore = roundup_kpow2(xbefore, ALIGNMENT); 
    int xstride = xbefore + roundup_kpow2((width + xafter), ALIGNMENT);
    
    int parent_width = xstride;
    int parent_height = height + ybefore + yafter;
    int parent_depth = depth + zbefore + zafter;
    
    int parent_sizes[3];
    parent_sizes[0] = parent_depth;
    parent_sizes[1] = parent_height;
    parent_sizes[2] = parent_width;

    if constexpr (std::is_signed<T>::value) {
	parent.create(3, parent_sizes, cv::DataType<T>::type);
    } else {
	using U = typename std::make_signed<T>::type;
	parent.create(3, parent_sizes, cv::DataType<U>::type);
    }
    parent.setTo(0);
    
    std::vector<cv::Range> rect;
    rect.push_back(cv::Range(zbefore, zbefore + depth));
    rect.push_back(cv::Range(ybefore, ybefore + height));
    rect.push_back(cv::Range(xbefore, xbefore + width));

    mat = parent(rect);

    {
	int w, h, d;
	int rowstride, slicestride;
	int prowstride, pslicestride;

	GetMatSize(mat, w, h, d);
	GetMatStrides<T>(mat, rowstride, slicestride);
	GetMatStrides<T>(parent, prowstride, pslicestride);
		
	assert(w == width && h == height && d == depth);
	assert(rowstride == prowstride && slicestride == pslicestride);
    }
}

template <typename T>
cv::Mat create_mat_from_data(const T arr[], int width, int height) {
    cv::Mat mat;
    create_mat_with_border<T>(mat, width, height, 0, 0, 0, 0);

    for (int row = 0; row < height; row++) {
	T* restrict dstline = mat.ptr<T>(row);
	const T* srcline = arr + width * row;
	for (int col = 0; col < width; col++) {
	    dstline[col] = srcline[col];
	}
    }
    
    return mat;
}


template <typename T, typename Fun>
void generate_mat(cv::Mat& mat, Fun fun) {
    int width, height, depth;
    GetMatSize(mat, width, height, depth);


    if (mat.dims == 3) {    
	for (int slice = 0; slice < depth; slice++) {
	    for (int row = 0; row < height; row++) {
		T* restrict line = mat.ptr<T>(slice, row);
		for (int col = 0; col < width; col++){
		    line[col] = fun(col, row, slice);
		}
	    }
	}
    } else {
	assert(mat.dims == 2);
	for (int row = 0; row < height; row++) {
	    T* restrict line = mat.ptr<T>(row);
	    for (int col = 0; col < width; col++) {
		line[col] = fun(col, row, 0);
	    }
	}
    }
}

template <typename T>
void generate_random_mat(cv::Mat& mat, int seed, double p) {
    std::mt19937 mt(seed);
    std::bernoulli_distribution gen(p);

    generate_mat<T>(mat, [&](int16_t col, int16_t row, int16_t slice){
	return gen(mt);
    });

}

template <typename T>
void fill_mat_box(cv::Mat& mat, T val, int xmin, int ymin, int xmax, int ymax) {\
    int width, height, depth;
    GetMatSize(mat, width, height, depth);
    assert(xmin >= 0 && ymin >= 0
	   && xmax < width && ymax < height);

    for (int row = ymin; row <= ymax; row++) {
	T* restrict line = mat.ptr<T>(row);
	for (int col = xmin; col <= xmax; col++){
	    line[col] = val;
	}
    }
}

template <typename T>
void fill_mat_box(cv::Mat& mat, T val, int xmin, int ymin, int zmin, int xmax, int ymax, int zmax) {
    int width, height, depth;
    GetMatSize(mat, width, height, depth);
    assert(xmin >= 0 && ymin >= 0 && zmin >= 0
	&& xmax < width && ymax < height && zmax < depth);

    for (int slice = zmin; slice <= zmax; slice++) {
	for (int row = ymin; row <= ymax; row++) {
	    T* restrict line = mat.ptr<T>(slice, row);
	    for (int col = xmin; col <= xmax; col++){
		line[col] = val;
	    }
	}
    }        
}

template <typename T>
void fill_mat(cv::Mat& mat, T val) {
    int width, height, depth;
    GetMatSize(mat, width, height, depth);

    if (mat.dims == 3) {
	for (int slice = 0; slice < depth; slice++){
	    for (int row = 0; row < height; row++) {
		T* restrict line = mat.ptr<T>(slice, row);
		std::fill(line, line + width, val);
	    }
	}
    } else {
	assert(mat.dims == 2);
	for (int row = 0; row < height; row++) {
	    T* restrict line = mat.ptr<T>(row);
	    std::fill(line, line + width, val);
	}
    }
    
}

template <typename T>
cv::Mat rotate_zaxis_mat(const cv::Mat& mat) {
    int width, height, depth;
    GetMatSize(mat, width, height, depth);
    
    cv::Mat res;
    create_mat_with_border<T>(res, height, width, depth, 0, 0, 0, 0, 0, 0);
    for (int slice = 0; slice < depth; slice++) {
	for (int row = 0; row < height; row++) {
	    for (int col = 0; col < width; col++) {
		res.at<T>(slice, col, row) = mat.at<T>(slice, row, col);
	    }
	}
    }
    return res;
}

#endif // CCL_CV_UTILS_HPP
