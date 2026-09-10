#include <cassert>
#include <iostream>
#include "../src/frame_timing.h"

int main() {
    volatile FrameTiming s;
    s.record(0);
    assert(s.samples == 0);
    s.record(33240);
    s.record(66560); // upper boundary inclusive
    assert(s.samples == 2 && s.outside == 0);
    s.record(99799); // 33239
    s.record(133120); // 33321
    assert(s.outside == 2 && s.minimum == 33239 && s.maximum == 33321);
    s.reset();
    assert(s.samples == 0 && !s.primed && s.outside == 0);
    s.record(UINT32_MAX - 100);
    s.record(33359); // wrap: 33460 us
    assert(s.last == 33460 && s.outside == 1);
    s.reset();
    s.record(12345);
    s.record(45665);
    assert(s.last == 33320 && s.samples == 1 && s.outside == 0);
    s.record(78945); // 33280 is now inside the requested range
    assert(s.last == 33280 && s.outside == 0);
    std::cout << "PASS: first sample, inclusive bounds, outliers, reset, micros wrap\n";
}
