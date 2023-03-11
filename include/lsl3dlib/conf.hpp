#ifndef CCL_ALGOS_CONF_HPP
#define CCL_ALGOS_CONF_HPP

#include <cstdint>

namespace algo {


struct ConfRosenfeld {
    static constexpr bool RLC = false;
    static constexpr bool ER = false;
    static constexpr bool Double = false;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = false;
    static constexpr bool ACC = false;
};

struct ConfMerge {
    static constexpr bool RLC = true;
    static constexpr bool ER = false;
    static constexpr bool Double = false;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = true;
    static constexpr bool ACC = false;
};

struct ConfDouble {
    static constexpr bool RLC = true;
    static constexpr bool ER = false;
    static constexpr bool Double = true;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = true;
    static constexpr bool ACC = false;
};

struct ConfPipeline {
    static constexpr bool RLC = true;
    static constexpr bool ER = false;
    static constexpr bool Double = true;
    static constexpr bool Pipeline = true;
    static constexpr bool ERA = true;
    static constexpr bool ACC = false;
};

struct ConfNoERA {
    static constexpr bool RLC = true;
    static constexpr bool ER = false;
    static constexpr bool Double = false;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = false;
    static constexpr bool ACC = false;
};


struct ConfER {
    static constexpr bool RLC = true;
    static constexpr bool ER = true;
    static constexpr bool Double = false;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = true;
    static constexpr bool ACC = false;
};


struct ConfERDouble {
    static constexpr bool RLC = true;
    static constexpr bool ER = true;
    static constexpr bool Double = true;
    static constexpr bool Pipeline = false;
    static constexpr bool ERA = true;
    static constexpr bool ACC = false;
};


struct ConfFeatures_All {
    static constexpr bool Moment = true;
    static constexpr bool Volume = true;
    static constexpr bool BoundingBox = true;
};

}

#endif // CCL_ALGOS_CONF_HPP
