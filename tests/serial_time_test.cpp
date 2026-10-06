#include <cassert>
#ifndef SOURCE_FILE
#define SOURCE_FILE "../src/main.cpp"
#endif
#include SOURCE_FILE

void equal(TC a, TC b) {
    assert(a.H == b.H && a.M == b.M && a.S == b.S && a.F == b.F);
}
TC current() { return TC{curH, curM, curS, curF}; }
void input(const char* line) {
    Serial.input = line;
    while (Serial.available()) serviceCalibration();
}
void boundary() {
    while (txBitIdx || midBitEdgePending) TIMER1_COMPA_vect();
    TIMER1_COMPA_vect();
}
int main() {
    TC parsed;
    assert(parseClockTime("T,20,40,31,454", 14, parsed));
    equal(parsed, TC{20, 40, 31, 14});
    assert(parseClockTime("T,00,00,00,999", 14, parsed));
    equal(parsed, TC{0, 0, 0, 29});
    assert(parseClockTime("T,0,0,0,4", 9, parsed));
    equal(parsed, TC{0, 0, 0, 0});
    for (auto invalid : {"T,24,00,00,000", "T,00,60,00,000", "T,00,00,60,000",
                         "T,20,40,31,1000", "T,20,40,31,", "T,20,40,31,xxx",
                         "20:40:31.454", "T,20,,31,454", "T,20,40,31,454,1",
                         "T,20,40,31,-1", "T,20,40,31,4x", "T,20,40,31,454,"}) {
        assert(!parseClockTime(invalid, strlen(invalid), parsed));
    }
    // Independently advance legal DF labels, checking every frame in a day.
    TC expected = {0, 0, 0, 0};
    for (uint32_t n = 0; n < 2589408UL; ++n) {
        uint32_t ms = uint32_t((uint64_t(n) * 1001 + 29) / 30);
        equal(timecodeFromMilliseconds(ms), expected);
        advanceTC_local(expected);
    }
    equal(expected, TC{0, 0, 0, 0});
    setup();
    boundary();
    TC before = current();
    input("T,20,40,");
    assert(!timeSetPending);
    input("31,454\r\n");
    assert(timeSetPending);
    equal(current(), before);
    boundary();
    equal(current(), TC{20, 40, 31, 14});
    assert(!timeSetPending && !bufferSwapRequest);
    uint8_t expectedBytes[10];
    buildFrame(expectedBytes, current());
    for (unsigned i = 0; i < 10; ++i) assert(txBuf[i] == expectedBytes[i]);
    prepareNextFrame();
    boundary();
    equal(current(), TC{20, 40, 31, 15});
    for (auto invalid : {"T,24,00,00,000\n", "T,20,40,31,4540\n", "xT,20,40,31,454\n", "20:40:31.454\n"}) {
        input(invalid);
        assert(!timeSetPending);
    }
    input("T,20,40,31,454\nT,00,00,00,000\n");
    boundary();
    equal(current(), TC{0, 0, 0, 0});
    input("T,20,40");
    simulatedMicros += 500001;
    serviceCalibration();
    assert(!timeInputLength && !timeSetPending);
    input("T,00,00,00,999\n");
    boundary();
    equal(current(), TC{0, 0, 0, 29});
    boundary();
    equal(current(), TC{0, 0, 1, 0});
    uint16_t timing = halfBitUnits;
    unsigned writes = EEPROM.writes;
    assert(timing == 8320 && writes == 0);
    input("U");
    assert(requestedHalfBitUnits == 8321 && calibrationStep);
    puts("PASS: all DF frames in 24h, milliseconds, fragmented/invalid input, atomic boundary set, latest update, timeout, continued LTC and U command");
}
