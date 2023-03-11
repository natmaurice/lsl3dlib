#ifndef CCL_ALGOS_FEATURES_HPP
#define CCL_ALGOS_FEATURES_HPP

#include <cstdint>
#include <cstddef>
#include <simdhelpers/restrict.hpp>
#include <iostream>
#include <limits>
#include <cassert>
#include <map>

#include <simdhelpers/aligned_alloc.hpp>


struct ConfFeatures2DNone {
    static constexpr bool UseAABB = false;
    static constexpr bool UseMoment = false;
    static constexpr bool UseVolume = false;

    static constexpr size_t Dims = 2;
};

struct ConfFeatures2DAll {
    static constexpr bool UseAABB = true;
    static constexpr bool UseMoment = true;
    static constexpr bool UseVolume = true;

    static constexpr size_t Dims = 2;
};


struct ConfFeatures3DNone {
    static constexpr bool UseAABB = false;
    static constexpr bool UseMoment = false;
    static constexpr bool UseVolume = false;

    static constexpr size_t Dims = 3;
};

struct ConfFeatures3DAll {
    static constexpr bool UseAABB = true;
    static constexpr bool UseMoment = true;
    static constexpr bool UseVolume = true;

    static constexpr size_t Dims = 3;
};



struct Features {

    using Label_t = int32_t;
    static constexpr size_t ALIGNMENT = 32;
    
    int64_t* restrict Sx = nullptr; // X-moment (sum of X coordinates)
    int64_t* restrict Sy = nullptr; // Y-moment (sum of Y coordinates)
    int64_t* restrict Sz = nullptr; // Z-moment (sum of Z coordinates)
    uint32_t* restrict S = nullptr; // size/area

    uint16_t* restrict lo_col = nullptr;
    uint16_t* restrict lo_row = nullptr;
    uint16_t* restrict lo_slice = nullptr;
    uint16_t* restrict hi_col = nullptr;
    uint16_t* restrict hi_row = nullptr;
    uint16_t* restrict hi_slice = nullptr;

    uint32_t size = 0;

    Features() :
	Sx(nullptr), Sy(nullptr), Sz(nullptr), S(nullptr),
	lo_col(nullptr), lo_row(nullptr), lo_slice(nullptr),
	hi_col(nullptr), hi_row(nullptr), hi_slice(nullptr), size(0) {	
    }

    ~Features() {
	// Dealloc everything.
	// Unused arrays are expected to be set to nullptr
	Dealloc<ConfFeatures3DAll>();
    }

    Features(const Features&) = delete;
    Features& operator=(const Features&) = delete;
    
    Features(Features&& features) {
	std::swap(Sx, features.Sx);
	std::swap(Sy, features.Sy);
	std::swap(Sz, features.Sz);
	
	std::swap(S, features.S);
	
	std::swap(lo_col, features.lo_col);
	std::swap(lo_row, features.lo_row);
	std::swap(lo_slice, features.lo_slice);
	
	std::swap(hi_col, features.hi_col);
	std::swap(hi_row, features.hi_row);
	std::swap(hi_slice, features.hi_slice);

	std::swap(size, features.size);
    }    

    template <typename U>
    void copy_if_not_null(U* restrict& dst, const U* restrict src, size_t size) {
	dst = nullptr;
	if (src != nullptr) {
	    dst = aligned_new<U>(size, ALIGNMENT);
	    std::copy(src, src + size, dst);
	}
    }
    
    Features Copy() {
	Features cpy;

	// Ideally, a template would be passed to the function to check what has to be 
	// Constraints with YACCLAB are why we're checking if each array is null
	copy_if_not_null<int64_t>(cpy.Sx, Sx, size);
	copy_if_not_null<int64_t>(cpy.Sy, Sy, size);
	copy_if_not_null<int64_t>(cpy.Sz, Sz, size);

	copy_if_not_null<uint32_t>(cpy.S, S, size);

	copy_if_not_null<uint16_t>(cpy.lo_col, lo_col, size);
	copy_if_not_null<uint16_t>(cpy.lo_row, lo_row, size);
	copy_if_not_null<uint16_t>(cpy.lo_slice, lo_slice, size);
	
	copy_if_not_null<uint16_t>(cpy.hi_col, hi_col, size);
	copy_if_not_null<uint16_t>(cpy.hi_row, hi_row, size);
	copy_if_not_null<uint16_t>(cpy.hi_slice, hi_slice, size);

	cpy.size = size;
	
	return cpy;
    }
    
    template <typename Conf>
    void Alloc(size_t size) {
	Dealloc<Conf>();

	assert(size > 0);
    
	this->size = size;

	if (Conf::UseMoment) {	    
	    Sx = aligned_new<int64_t>(size, ALIGNMENT);
	    Sy = aligned_new<int64_t>(size, ALIGNMENT);

	    if (Conf::Dims == 3) {
		Sz = aligned_new<int64_t>(size, ALIGNMENT);
	    }
	}
	if (Conf::UseVolume) {
	    S = aligned_new<uint32_t>(size, ALIGNMENT);
	}
	if (Conf::UseAABB) {
	    lo_col = aligned_new<uint16_t>(size, ALIGNMENT);
	    hi_col = aligned_new<uint16_t>(size, ALIGNMENT);
	    lo_row = aligned_new<uint16_t>(size, ALIGNMENT);
	    hi_row = aligned_new<uint16_t>(size, ALIGNMENT);

	    if (Conf::Dims == 3) {
		lo_slice = aligned_new<uint16_t>(size, ALIGNMENT);
		hi_slice = aligned_new<uint16_t>(size, ALIGNMENT);
	    }
	}
    }    
    
    template <typename Conf>
    void Dealloc() {
	if (Conf::UseMoment) {
	    aligned_delete(Sx, ALIGNMENT);
	    aligned_delete(Sy, ALIGNMENT);

	    if (Conf::Dims == 3) {
		aligned_delete(Sz, ALIGNMENT);
	    }
	}
	if (Conf::UseVolume) {
	    aligned_delete(S, ALIGNMENT);
	}
	if (Conf::UseAABB){	    
	    aligned_delete(lo_col, ALIGNMENT);
	    aligned_delete(lo_row, ALIGNMENT);
	    aligned_delete(hi_col, ALIGNMENT);
	    aligned_delete(hi_row, ALIGNMENT);

	    if (Conf::Dims == 3) {
		aligned_delete(lo_slice, ALIGNMENT);	    
		aligned_delete(hi_slice, ALIGNMENT);
	    }
	}
	
	Sx = nullptr;
	Sy = nullptr;
	Sz = nullptr;
	S = nullptr;

	lo_col = lo_row = lo_slice = nullptr;
	hi_col = hi_row = hi_slice = nullptr;	
    }
    
    template <typename Conf>
    void Touch() {

	if (Conf::UseMoment) {
	    std::fill(Sx, Sx + size, 0);
	    std::fill(Sy, Sy + size, 0);

	    if (Conf::Dims == 3) {
		std::fill(Sz, Sz + size, 0);
	    }
	}
	if (Conf::UseVolume){
	    std::fill(S, S + size, 0);
	}

	if (Conf::UseAABB) {
	    std::fill(lo_col,   lo_col   + size, 0);
	    std::fill(lo_row,   lo_row   + size, 0);
	    std::fill(hi_col,   hi_col   + size, 0);
	    std::fill(hi_row,   hi_row   + size, 0);

	    if (Conf::Dims == 3) {
		std::fill(lo_slice, lo_slice + size, 0);
		std::fill(hi_slice, hi_slice + size, 0);
	    }
	}
    }

    // Set default value for elements in [0; count[
    template <typename Conf>
    void Init(size_t min_label, size_t max_label) {

	
	if (Conf::UseMoment) {
	    assert(Sx != nullptr && "Sx not allocated");
	    assert(Sy != nullptr && "Sy not allocated");
	    
	    std::fill(Sx + min_label, Sx + max_label, 0);
	    std::fill(Sy + min_label, Sy + max_label, 0);
	    
	    if (Conf::Dims == 3) {
		std::fill(Sz + min_label, Sz + max_label, 0);
	    }
	}
	if (Conf::UseVolume) {
	    assert(S != nullptr && "S not allocated");
	    
	    std::fill(S + min_label, S + max_label, 0);
	}
	if (Conf::UseAABB) {
	    
	    std::fill(lo_col + min_label, lo_col + max_label, INT16_MAX);
	    std::fill(lo_row + min_label, lo_row + max_label, INT16_MAX);
	    std::fill(hi_col + min_label, hi_col + max_label, 0);
	    std::fill(hi_row + min_label, hi_row + max_label, 0);
	    
	    if (Conf::Dims == 3) {
		std::fill(lo_slice + min_label, lo_slice + max_label, INT16_MAX);
		std::fill(hi_slice + min_label, hi_slice + max_label, 0);
	    }	    
	}
    }

    template <typename Conf>
    void Init(size_t label_count) {
	Init<Conf>(0, label_count);
    }
    
    template <typename Conf>
    void Merge(Label_t i, Label_t j) {

	// Check if i == j
	// This can happen during transitive closures
	if (i != j) {

	    
	    if (Conf::UseMoment) {

		
		Sx[j] += Sx[i];
		Sy[j] += Sy[i];

		
		if (Conf::Dims == 3) {
		    Sz[j] += Sz[i];
		}
	    }
	    if (Conf::UseVolume) {
		//std::cout << "S[" << j << "] (" << S[j] << ") += S[" << i << "] (" << S[i] << ")" << std::endl;
		S[j]  += S[i];
		assert(S[i] >= 0);			    
	    }
	    if (Conf::UseAABB) {
		//std::cout << "lo_col[" << j << "] (" << lo_col[j] << ") += lo_col[" << i << "] (" << lo_col[i] << ")" << std::endl;

		lo_col[j] = std::min(lo_col[i],     lo_col[j]);
		lo_row[j] = std::min(lo_row[i],     lo_row[j]);
		hi_col[j] =   std::max(hi_col[i],   hi_col[j]);
		hi_row[j] =   std::max(hi_row[i],   hi_row[j]);

		//std::cout << "Merge: " << j << "<= " << i << ", S = " << S[j] << " (+" << S[i] << ")"
		//	  << ", lo_col = " << lo_col[i] << " (" << lo_col[j] << ")\n";
		
		if (Conf::Dims == 3) {
		    lo_slice[j] = std::min(lo_slice[i], lo_slice[j]);		
		    hi_slice[j] = std::max(hi_slice[i], hi_slice[j]);
		}
	    }
	}
    }

    
    template <typename Conf>
    void NewComponent2D(int32_t label) {
	NewComponent2D<Conf>(label, 0, 0, 0,
			     std::numeric_limits<uint16_t>::max(),
			     std::numeric_limits<uint16_t>::max(),
			     std::numeric_limits<uint16_t>::min(),
			     std::numeric_limits<uint16_t>::min());
    }

    
    template <typename Conf>
    void NewComponent2D(Label_t label, int64_t sx, int64_t sy, uint32_t s,
			uint16_t lcol, uint16_t lrow,
			uint16_t hcol, uint16_t hrow) {
	
	static_assert(Conf::Dims == 2, "NewComponent2D requires 2 dimensions");
	
	if (Conf::UseMoment) {
	    Sx[label] = sx;
	    Sy[label] = sy;
	}
	if (Conf::UseVolume) {
	    S[label]  = s;
	}
	if (Conf::UseAABB){
	    lo_col[label] = lcol;
	    lo_row[label] = lrow;
    
	    hi_col[label] = hcol;
	    hi_row[label] = hrow;
	}
    }

    template <typename Conf>
    void AddSegment2D(uint32_t i, uint16_t row, uint16_t x0, uint16_t x1) {

	static_assert(Conf::Dims == 2, "AddSegment2D requires 2 dimensions");
	
	uint16_t slen = x1 - x0;
	if (Conf::UseMoment) {
	    Sx[i] += (x0 + x1 - 1) * (x1 - x0) / 2;
	    Sy[i] += row * slen;
	}
	if (Conf::UseVolume) {
	    S[i] += slen;
	}
	if (Conf::UseAABB) {
	    lo_col[i] = std::min(x0, lo_col[i]);
	    lo_row[i] = std::min(row, lo_row[i]);
    
	    hi_col[i] = std::max<uint16_t>(x1, hi_col[i]);
	    hi_row[i] = std::max(row, hi_row[i]);
	}
    }

    
    template <typename Conf> 
    void AddPoint2D(uint32_t i, uint16_t col, uint16_t row) {

	static_assert(Conf::Dims == 2, "AddPoint2D requires 2 dimensions");
	
	if (Conf::UseMoment) {
	    Sx[i] += col;
	    Sy[i] += row;
	}
	if (Conf::UseVolume) {
	    S[i]++;
	}

	if (Conf::UseAABB){
	    lo_col[i] = std::min(col, lo_col[i]);
	    lo_row[i] = std::min(row, lo_row[i]);
    
	    hi_col[i] = std::max<uint16_t>(col + 1, hi_col[i]);
	    hi_row[i] = std::max(row, hi_row[i]);
	}
    }

    template <typename Conf>
    void NewComponent3D(uint32_t label)  {

	if (Conf::UseVolume) {
	    S[label] = 0;
	}
	
	if (Conf::UseMoment) {
	    Sx[label] = 0;
	    Sy[label] = 0;
	    Sz[label] = 0;
	}
	
	if (Conf::UseAABB) {
	    //std::cout << "[" << label << "] NewComponent3D(label): lcol = " << 0 << "\n";		    
	    
	    lo_col[label] = std::numeric_limits<int16_t>::max();
	    hi_col[label] = 0;
	
	    lo_row[label] = std::numeric_limits<int16_t>::max();
	    hi_row[label] = 0;
	
	    lo_slice[label] = std::numeric_limits<int16_t>::max();
	    hi_slice[label] = 0;
	}
    }
    
    template <typename Conf>
    void NewComponent3D(uint32_t label, uint16_t row, uint16_t slice, uint16_t x0, uint16_t x1) {

	static_assert(Conf::Dims == 3, "NewComponent(label, row, slice, x0, x1) requires 3 dimensions");
	
	uint16_t slen = x1 - x0;
	assert(slen > 0);
	
	int64_t sx = (x0 + x1 - 1) * (x1 - x0) / 2;
	int64_t sy = row * slen;
	int64_t sz = slice * slen;
    
        NewComponent3D<Conf>(label, sx, sy, sz, slen, x0, row, slice, x1, row, slice);
    }
    
    
    template <typename Conf>
    void NewComponent3D(Label_t label, int64_t sx, int64_t sy, int64_t sz, uint32_t s,
		      uint16_t lcol, uint16_t lrow, uint16_t lslice,
		      uint16_t hcol, uint16_t hrow, uint16_t hslice) {

	static_assert(Conf::Dims == 3, "NewComponent(label, row, slice, x0, x1) requires 3 dimensions");

	
	if (Conf::UseMoment) {
	    Sx[label] = sx;
	    Sy[label] = sy;
	    Sz[label] = sz;
	}
	if (Conf::UseVolume) {
	    static_assert(!Conf::UseVolume || (Conf::UseVolume && Conf::UseAABB), "AABB not enabled");
	    //std::cout << "[" << label << "] NewComponent3D: S = " << s << "\n";
	    S[label]  = s;
	}
	if (Conf::UseAABB){

	    //std::cout << "[" << label << "] NewComponent3D: lcol = " << lcol << "\n";

	    lo_col[label] = lcol;
	    lo_row[label] = lrow;
	    lo_slice[label] = lslice;
    
	    hi_col[label] = hcol;
	    hi_row[label] = hrow + 1;
	    hi_slice[label] = hslice + 1;
	}
    }

    // Updates component stats (only for CCA)
    template <typename Conf>
    void AddSegment3D(uint32_t i, uint16_t row, uint16_t slice, uint16_t x0, uint16_t x1) {
	uint16_t slen = x1 - x0;

	static_assert(Conf::Dims == 3, "NewComponent(label, row, slice, x0, x1) requires 3 dimensions");
	
	if (Conf::UseMoment) {
	    Sx[i] += (x0 + x1 - 1) * (x1 - x0) / 2;
	    Sy[i] += row * slen;
	    Sz[i] += slice * slen;
	}
	if (Conf::UseVolume) {
	    //std::cout << "[" << i << "] NewComponent3D: S (" << S[i] << ") = " << slen << "\n";

	    S[i] += slen;
	}
	if (Conf::UseAABB) {
	    lo_col[i] = std::min(x0, lo_col[i]);
	    lo_row[i] = std::min(row, lo_row[i]);
	    lo_slice[i] = std::min(slice, lo_slice[i]); // Might not be needed if new pixel
    
	    hi_col[i] = std::max<uint16_t>(x1, hi_col[i]);
	    hi_row[i] = std::max<uint16_t>(row + 1, hi_row[i]);
	    hi_slice[i] = std::max<uint16_t>(slice + 1, hi_slice[i]); //
	}
    }
    
    template <typename Conf>
    void AddPoint3D(uint32_t i, uint16_t col, uint16_t row, uint16_t slice) {

	static_assert(Conf::Dims == 3, "NewComponent(label, row, slice, x0, x1) requires 3 dimensions");
	
	if (Conf::UseMoment) {
	    Sx[i] += col; 
	    Sy[i] += row;
	    Sz[i] += slice;
	}
	if (Conf::UseVolume) {
	    S[i]++;
	}

	if (Conf::UseAABB){
	    lo_col[i] = std::min(col, lo_col[i]);
	    lo_row[i] = std::min(row, lo_row[i]);
	    lo_slice[i] = std::min(slice, lo_slice[i]); // Might not be needed if new pixel
    
	    hi_col[i] = std::max<uint16_t>(col + 1, hi_col[i]);
	    hi_row[i] = std::max<uint16_t>(row + 1, hi_row[i]);
	    hi_slice[i] = std::max<uint16_t>(slice + 1, hi_slice[i]); 
	}
    }

    template <typename Conf>
    void Shift(uint32_t i, uint32_t j) {
	if (Conf::UseMoment) {
	    Sx[i] = Sx[j];
	    Sy[i] = Sy[j];

	    if (Conf::Dims == 3) {
		Sz[i] = Sz[j];
	    }
	}
	if (Conf::UseVolume) {
	    S[i] = S[j];
	}
	if (Conf::UseAABB) {
	    lo_col[i] = lo_col[j];
	    lo_row[i] = lo_row[j];		
	    hi_col[i] = hi_col[j];
	    hi_row[i] = hi_row[j];

	    if (Conf::Dims == 3) {
		lo_slice[i] = lo_slice[j];
		hi_slice[i] = hi_slice[j];
	    }
	}
    }

    void NormalizeFrom(const Features& src, const std::map<int, int>& label_map ) {
	for (const auto& entry: label_map) {
	    int srclabel = entry.first;
	    int dstlabel = entry.second;

	    if (S != nullptr) {
		//std::cout << "S[" << dstlabel << "] = " << srclabel << " (" << src.S[srclabel] << ")\n";
		S[dstlabel] = src.S[srclabel];
	    }

	    if (Sx != nullptr) {
		Sx[dstlabel] = src.Sx[srclabel];
	    }
	    if (Sy != nullptr) {
		Sy[dstlabel] = src.Sy[srclabel];
		
	    }
	    if (Sz != nullptr) {
		Sz[dstlabel] = src.Sz[srclabel];		
	    }

	    if (lo_col != nullptr) {
		lo_col[dstlabel] = src.lo_col[srclabel];				
	    }
	    if (lo_row != nullptr) {
		lo_row[dstlabel] = src.lo_row[srclabel];		
	    }
	    if (lo_slice != nullptr) {
		lo_slice[dstlabel] = src.lo_slice[srclabel];		
	    }
	    
	    if (hi_col != nullptr) {
		hi_col[dstlabel] = src.hi_col[srclabel];
	    }
	    if (hi_row != nullptr) {
		hi_row[dstlabel] = src.hi_row[srclabel];
	    }
	    if (hi_slice != nullptr) {
		hi_slice[dstlabel] = src.hi_slice[srclabel];
	    }
	}
	
    }
    
    template <typename Conf>
    bool Equals(const Features& other, size_t label_count) const {	
	bool ok = true;
	for (size_t i = 1; i < label_count && ok; i++) {	

	    
	    if (Conf::UseVolume) {		
		int64_t s = S[i];
		int64_t s_ref = other.S[i];
		
		if (s != s_ref) {
		    std::cout << "Equals [" << i << "]: S = " << s << ", S_ref = " << s_ref << "\n";

		    return false;
		}
	    }

	    
	    if (Conf::UseMoment) {
		int64_t sx = Sx[i];
		int64_t sy = Sy[i];
		int64_t sz = Sz[i];

		int64_t sx_ref = other.Sx[i];
		int64_t sy_ref = other.Sy[i];
		int64_t sz_ref = other.Sz[i];
		
		if (sx != sx_ref) {
		    std::cout << "Equals [" << i << "]: Sx = " << sx << ", Sx_ref = " << sx_ref << "\n";
		    return false;
		}
		if (sy != sy_ref) {
		    std::cout << "Equals [" << i << "]: Sy = " << sy << ", Sy_ref = " << sy_ref << "\n";
		    return false;
		}
		if (Conf::Dims == 3) {
		    if (sz != sz_ref) {
			std::cout << "Equals [" << i << "]: Sz = " << sz << ", Sz_ref = " << sz_ref << "\n";
			return false;
		    }
		}
	    }

	    if (Conf::UseAABB) {
		if (lo_col[i] != other.lo_col[i]) {
		    std::cout << "Equals [" << i << "]: lo_col = " << lo_col[i] << ", lo_col_ref = "
			      << other.lo_col[i] << "\n";

		    return false;		    
		}	
		if (lo_row[i] != other.lo_row[i]) {
		    std::cout << "Equals [" << i << "]: lo_row = " << lo_row[i] << ", lo_row_ref = "
			      << other.lo_row[i] << "\n";

		    return false;
		}
		if (hi_col[i] != other.hi_col[i]) {
		    std::cout << "Equals [" << i << "]: hi_col = " << hi_col[i] << ", hi_col_ref = "
			      << other.hi_col[i] << "\n";

		    return false;
		}
		if (hi_row[i] != other.hi_row[i]) {
		    std::cout << "Equals [" << i << "]: lo_row = " << hi_row[i] << ", hi_row_ref = "
			      << other.hi_row[i] << "\n";

		    return false;
		}

		if (Conf::Dims == 3) {
		    if (lo_slice[i] != other.lo_slice[i]) {
			std::cout << "Equals [" << i << "]: lo_slice = " << lo_slice[i] << ", lo_slice_ref = "
			      << other.lo_slice[i] << "\n";

			return false;
		    }

		    if (hi_slice[i] != other.hi_slice[i]) {
			std::cout << "Equals [" << i << "]: hi_slice = " << hi_slice[i] << ", hi_slice_ref = "
			      << other.hi_slice[i] << "\n";

			return false;
		    }
		}
	    }
	}
	return ok;
    }
    
};


#endif // CCL_ALGOS_FEATURES_HPP
