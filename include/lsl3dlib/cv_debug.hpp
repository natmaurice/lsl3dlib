#ifndef CCL_CV_DEBUG_HPP
#define CCL_CV_DEBUG_HPP

#define CATCH_CONFIG_CONSOLE_WIDTH 10000

#include <simdhelpers/term_utils.hpp>
#include <opencv2/core.hpp>
#include "lsl3dlib/cv_utils.hpp"
#include <iomanip>


template <typename T>
struct MatCompare {
    const cv::Mat& a;
    const cv::Mat& b;

    MatCompare(const cv::Mat& a, const cv::Mat& b) : a(a), b(b) {}

    operator bool() const {
	try {
	cv::Mat diff = a != b;
	return cv::countNonZero(diff) == 0;
	} catch (std::exception& e) {
	    std::cerr << e.what() << "\n";
	    assert(false);
	}
    }    
};

template <typename T>
void print_mat(std::ostream& out, const cv::Mat& mat) {
    int width, height, depth;
    GetMatSize(mat, width, height, depth);
    
    for (int slice = 0; slice < depth; slice++) {
	out << "------ (" <<  slice << ")" << std::endl;
	for (int row = 0; row < height; row++) {
	    out << std::setw(2) << slice << " | ";
	    const T* restrict line = mat.ptr<T>(slice, row);
	    for (int col = 0; col < width; col++) {
		out << std::setw(2) << static_cast<uint64_t>(line[col]) << " ";
	    }
	    out << "\n";
	}
    }
    
}

template <typename T>
void print_mat_diff(std::ostream& out, const cv::Mat& mat, const cv::Mat& ref) {
    int width0, height0, depth0;
    int width1, height1, depth1;

    GetMatSize(mat, width0, height0, depth0);
    GetMatSize(ref, width1, height1, depth1);

    if (width0 != width1 || height0 != height1 || depth0 != depth1) {
	out << "matrices have different sizes\n";
    }
    out << TERM_RESET;
    out << "==================================================" << std::endl;
    if (mat.dims == 3) {
    
	for (int slice = 0; slice < depth0; slice++) {
	    out << "------ (" <<  slice << ")" << std::endl;
	    for (int row = 0; row < height0; row++) {
		const T* restrict linea = mat.template ptr<T>(slice, row);
		const T* restrict lineb = ref.template ptr<T>(slice, row);
		out << std::setw(2) << row << " | ";
		for (int col = 0; col < width0; col++) {
		    T val0 = linea[col];
		    T val1 = lineb[col];
		    if (val0 != val1){
			out << TERM_RED << std::setw(2) << static_cast<uint64_t>(val0) << " ";
		    } else {
			out << TERM_GREEN << std::setw(2) << static_cast<uint64_t>(val0) << " ";
		    }
		}
		out << TERM_RESET << "\n";
	    }
	}
    } else {
	assert(mat.dims == 2);

	out << resetiosflags(std::ios_base::basefield);
	for (int row = 0; row < height0; row++) {
	    const T* restrict line0 = mat.template ptr<T>(row);
	    const T* restrict line1 = ref.template ptr<T>(row);
	    out << std::setw(2) << row << " | ";

	    for (int col = 0; col < width0; col++) {
		T val0 = line0[col];
		T val1 = line1[col];

		if (val0 != val1){
		    out << TERM_RED << std::setw(2) << static_cast<uint64_t>(val0) <<  " ";
		} else {
		    out << TERM_GREEN << std::setw(2) << static_cast<uint64_t>(val0) << " ";
		}
	    }
	    out << TERM_RESET << "\n";
	}
	
    }
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const MatCompare<T>& compare) {
    print_mat_diff<T>(out, compare.a, compare.b);
    out << "\n --- \n\n";
    print_mat_diff<T>(out, compare.b, compare.a);
    return out;
}


#endif // CCL_CV_DEBUG_HPP
