#include <cassert>
#ifndef SOURCE_FILE
#define SOURCE_FILE "../src/main.cpp"
#endif
#include SOURCE_FILE

void checkPayload(TC tc) {
    uint8_t expected[10] = {};
    const uint8_t digits[] = {uint8_t(tc.F % 10), uint8_t(tc.F / 10),
        uint8_t(tc.S % 10), uint8_t(tc.S / 10), uint8_t(tc.M % 10),
        uint8_t(tc.M / 10), uint8_t(tc.H % 10), uint8_t(tc.H / 10)};
    for (unsigned i = 0; i < 8; ++i) expected[i] = digits[i];
    if (DROP_FRAME) expected[1] |= 4;
    expected[8] = 0xFC;
    expected[9] = 0xBF;
    unsigned ones = 0;
    for (auto b : expected) for (; b; b >>= 1) ones += b & 1;
    if (ones & 1) expected[3] |= 8;
    for (unsigned i = 0; i < 10; ++i) assert(txBuf[i] == expected[i]);
}

void seed(TC tc) {
    setup();
    curH = tc.H; curM = tc.M; curS = tc.S; curF = tc.F;
    buildFrame(txBuf, tc);
    advanceTC_local(tc);
    buildFrame(buildBuf, tc);
}

void runFrames(TC expected, unsigned count) {
    seed(expected);
    for (unsigned frame = 0; frame < count; ++frame) {
        assert(txBitIdx == 0 && !midBitEdgePending);
        if (frame && frame % 7 == 0) {
            // A late queued payload must not be emitted as the current frame.
            buildFrame(buildBuf, TC{12, 34, 56, 7});
            readyFrameNumber = framesSent;
            bufferSwapRequest = true;
        }
        TIMER1_COMPA_vect();
        assert(framesSent == frame);
        assert(curH == expected.H && curM == expected.M &&
               curS == expected.S && curF == expected.F);
        checkPayload(expected);
        unsigned ticks = OCR1A + 1;
        unsigned edges = 1;
        while (txBitIdx != 0 || midBitEdgePending) {
            // Interleave normal preparation with long main-loop starvation.
            if (frame % 13 < 3) prepareNextFrame();
            TIMER1_COMPA_vect();
            ticks += OCR1A + 1;
            ++edges;
            assert(framesSent == frame);
            assert(edges <= 160);
        }
        assert(ticks == 160 * 416);
        checkPayload(expected);
        advanceTC_local(expected);
    }
}

int main() {
    runFrames(TC{0, 0, 0, 0}, 20000);
    runFrames(TC{0, 0, 59, 28}, 8); // Drop-frame minute boundary.
    runFrames(TC{0, 9, 59, 28}, 8); // Tenth minute does not skip 00/01.
    runFrames(TC{23, 59, 59, 28}, 8); // Midnight wrap.
    seed(TC{0, 0, 0, 10});
    TIMER1_COMPA_vect();
    framesSent = UINT32_MAX;
    readyFrameNumber = 0;
    txBitIdx = 0;
    midBitEdgePending = false;
    TIMER1_COMPA_vect();
    assert(framesSent == 0 && curF == 11);
    checkPayload(TC{0, 0, 0, 11});
    puts("PASS: 20000 frames, starvation, stale buffers, payload/parity, BMC duration, DF/minute/midnight and counter wrap");
}
