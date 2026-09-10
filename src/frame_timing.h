#pragma once
#include <stdint.h>

// Local-clock observations, not externally calibrated physical microseconds.
struct FrameTiming {
    static const uint32_t minimumUs = 33240UL;
    static const uint32_t maximumUs = 33320UL;
    uint32_t previous = 0;
    uint32_t last = 0;
    uint32_t minimum = UINT32_MAX;
    uint32_t maximum = 0;
    uint32_t samples = 0;
    uint32_t outside = 0;
    bool primed = false;

    void record(uint32_t now) volatile {
        if (primed) {
            uint32_t delta = now - previous;
            last = delta;
            if (delta < minimum) minimum = delta;
            if (delta > maximum) maximum = delta;
            ++samples;
            if (delta < minimumUs || delta > maximumUs) ++outside;
        }
        previous = now;
        primed = true;
    }

    void reset() volatile {
        previous = last = maximum = samples = outside = 0;
        minimum = UINT32_MAX;
        primed = false;
    }
};
