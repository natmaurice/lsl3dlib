#ifndef CCL_LABELS_UTILS
#define CCL_LABELS_UTILS

#include <ostream>
#include <vector>
#include <opencv2/core.hpp>
#include <simdhelpers/restrict.hpp>
#include "lsl3dlib/cv_utils.hpp"
#include "aabb.hpp"


void print_labels(std::ostream& out, int32_t *labels, long width, long height, long depth = 1);


template<typename T>
void build_histogram(const cv::Mat& labels, std::vector<long>& histogram);


// Implementations
template <typename T>
void build_histogram(const cv::Mat& labels, std::vector<long>& histogram) {
    int width, height, depth;
    GetMatSize(labels, width, height, depth);
    
    histogram.clear();
    for (auto slice = 0; slice < depth; slice++) {
	for (auto row = 0; row < height; row++) {
	    const T* restrict line = labels.ptr<T>(slice, row);
	    for (auto col = 0; col < width; col++) {
		T label = line[col];
		if (label >= histogram.size()) {
		    histogram.resize(label + 1, 0);
		}
		histogram[label]++;
	    }
	}
    }
}

void generate_random_bitmap(cv::Mat1b& bitmap, uint64_t seed, float density);

inline bool test_area(cv::Mat1i& labels, const AABB_3D& area, int32_t expected) {

    for (int32_t slice = area.lo_slice; slice <= area.hi_slice; slice++){
	for (int32_t row = area.lo_row; row <= area.hi_row; row++) {
	    const int32_t* restrict line = labels.ptr<int32_t>(slice, row);
	    for (int32_t col = area.lo_col; col <= area.hi_col; col++) {
		int32_t label = line[col];
		if (label != expected) {
		    return false;
		}
	    }
	}
    }
	
    return true;
}

void convert_to(cv::Mat& in, cv::Mat& out, int code);
void create_hsv(cv::Mat& mat, int width, int height, int depth);
void random_palette(cv::Mat1i& labels, cv::Mat& bgr_output);

#endif // CCL_LABELS_UTILS
