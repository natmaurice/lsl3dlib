#include "lsl3dlib/utility.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>

uint16_t minp(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    // Note: This uses underflows to avoid conditions if one value is zero
    return std::min({a - 1U, b - 1U, c - 1U, d - 1U}) + 1U;
}

double find_closest(std::vector<std::pair<double, double>> data, double desired_density) {
    double closest_distance = std::numeric_limits<float>::max();
    double closest_threshold = std::numeric_limits<float>::max();
    for (auto &[in_threshold, out_density]: data) {	
	double distance = fabs(out_density - desired_density);
	if (distance < closest_distance) {
	    closest_distance = distance;
	    closest_threshold = in_threshold;
	}
    }
    return closest_threshold;
}
