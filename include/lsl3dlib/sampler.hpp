#ifndef CCL_SAMPLER_HPP
#define CCL_SAMPLER_HPP

#include <map>

#include <lsl3dlib/timer.hpp>

#if defined(__i386__) || defined(__x86_64__)
#include <lsl3dlib/papi_helper.hpp>
#endif // __x86_64__


template <typename MeasureFun>
class Sampler {
public:
    Sampler();
    
    inline void Start();
    inline void Stop();
    inline double Last() const;
    inline void Reset();
    inline void Store(int step, double val);

    
    inline double Get(int step) const;
    inline int Find(int step) const;
private:
    double last;
    std::map<int, double> stored;
};



struct MeasureTime {
    static inline double measure() {
	return dtime();
    }
};

#if defined(__i386__) || defined(__x86_64__)
struct MeasureCycles {
    static inline double measure() {
	return dcycles();
    };
};

struct MeasureL1DMiss {
    [[deprecated]]
    static inline double measure() {
	long long measurements[MAX_PAPI_EVENTS];
        read_counters(measurements);
	return get_from_measurements(measurements, PAPIH_L1DM);
    }
};


struct MeasureL1IMiss {
    [[deprecated]]
    static inline double measure() {
	long long measurements[MAX_PAPI_EVENTS];
        read_counters(measurements);
	return get_from_measurements(measurements, PAPIH_L1IM);
    }

};

struct MeasureBranchMispred {
    [[deprecated]]
    static inline double measure() {
	long long measurements[MAX_PAPI_EVENTS];
        read_counters(measurements);
	return get_from_measurements(measurements, PAPIH_MISP);
    }

};


struct MeasureStalledCycles {
    [[deprecated]]
    static inline double measure() {
	long long measurements[MAX_PAPI_EVENTS];
        read_counters(measurements);
	return get_from_measurements(measurements, PAPIH_STAL);
    }
};

struct MeasureDummy {
    [[deprecated]]
    static inline double measure() {
	return 0.0;
    }
};



using SamplerCycles = Sampler<MeasureCycles>;
using SamplerL1DMiss = Sampler<MeasureL1DMiss>;
using SamplerL1IMiss = Sampler<MeasureL1IMiss>;
using SamplerBranchMispred = Sampler<MeasureBranchMispred>;
using SamplerStalledCycles = Sampler<MeasureStalledCycles>;
using SamplerNoInsComplete = Sampler<MeasureStalledCycles>;
using SamplerWriteStalls = Sampler<MeasureStalledCycles>;


#endif // __x86_64__






// Implementation

template <typename MeasureFun>
Sampler<MeasureFun>::Sampler() {
}


template <typename MeasureFun>
void Sampler<MeasureFun>::Start() {
    last = MeasureFun::measure();
}

template <typename MeasureFun>
void Sampler<MeasureFun>::Stop() {
    last = MeasureFun::measure() - last;
}

template<typename MeasureFun>
double Sampler<MeasureFun>::Last() const {
    return last;
}

template <typename MeasureFun>
void Sampler<MeasureFun>::Store(int step, double val) {
    stored[step] = val;
}

template <typename MeasureFun>
double Sampler<MeasureFun>::Get(int step) const {
    auto it = stored.find(step);
    if (it != stored.end()) {
	return it->second;
    }
    return 0.0;
}

template <typename MeasureFun>
int Sampler<MeasureFun>::Find(int step) const {
    return stored.find(step) != stored.end();
}
    
#endif // CCL_SAMPLER_HPP

