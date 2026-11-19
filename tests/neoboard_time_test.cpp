#include <cassert>
#include <string>
#define OLED_TIMECODE 1
#include "../src/main.cpp"

// Fixtures independently calculated with Python zlib.crc32(payload, 0xffffffff)
// ^ 0xffffffff, matching NeoBoard's init-zero/no-final-XOR CRC32.
const char* firstPacket = "STX,100,11,2400:102000,5f200647\r\n";
const char* secondPacket = "STX,100,11,2400:104000,7a4b599b\r\n";

bool equal(TC a, TC b) {
    return a.H == b.H && a.M == b.M && a.S == b.S && a.F == b.F;
}
TC current() { return {curH, curM, curS, curF}; }
TC requested() { return {requestedH, requestedM, requestedS, requestedF}; }

void input(const std::string& bytes) {
    Serial.input += bytes;
    while (Serial.available()) {
        size_t before = Serial.input.size();
        uint16_t compare = OCR1A;
        serviceSerialInput();
        serviceNeoReply();
        assert(Serial.input.size() == before - 1); // Bounded single-byte polling.
        assert(OCR1A == compare); // Reception never changes the LTC timer.
    }
}

void boundary() {
    while (txBitIdx || midBitEdgePending) TIMER1_COMPA_vect();
    TIMER1_COMPA_vect();
    prepareNextFrame();
}

void checkParser() {
    NeoBoardTime::Parser parser;
    const struct { const char* line; uint32_t ms; } valid[] = {
        {firstPacket, 102000}, {secondPacket, 104000},
        {"STX,100,13,2400:86400000,502e884\n", 86400000}, // Unpadded CRC.
        {"STX,100,15,65535:604799999,58B9FA9D\r", 604799999},
        {"STX,100,3,0:0,271b0b45\n", 0},
        {"STX,100,6,2401:0,cbe12133\n", 0},
    };
    for (auto item : valid) {
        unsigned accepted = 0;
        for (const char* p = item.line; *p; ++p) {
            auto result = parser.feed(*p);
            assert(result != NeoBoardTime::Result::Rejected);
            if (result == NeoBoardTime::Result::Accepted) {
                ++accepted;
                assert(parser.milliseconds == item.ms);
            }
        }
        assert(accepted == 1 && !parser.active());
    }
    for (auto line : {
        "STX,101,11,2400:102000,5f200647\n", // Wrong command.
        "STX,100,10,2400:102000,5f200647\n", // Wrong length.
        "STX,100,11,2400:102001,5f200647\n", // Corrupted payload.
        "STX,100,11,2400:102000,5f200646\n", // Corrupted CRC.
        "STX,100,11,2400:102000,\n", "STX,100,11,2400:102000,5g200647\n",
        "STX,100,11,2400:102000,05f200647\n", // CRC too long.
        "STX,100,11,2400:102000,5f200647,extra\n",
        "STX,100,,2400:102000,5f200647\n", "STX,100,256,2400:102000,5f200647\n",
        "STX,100,16,2400:102000,5f200647\n", "STX,100,0,,0\n",
        "STX,100,11,65536:102000,0\n", "STX,100,11,-1:102000,0\n",
        "STX,100,14,2400:604800000,0\n", "STX,100,15,2400:4294967296,0\n",
        "STX,100,7,2400:-1,0\n", "STX,100,2,0:,0\n", "STX,100,2,:0,0\n",
    }) {
        unsigned rejected = 0;
        for (const char* p = line; *p; ++p) {
            auto result = parser.feed(*p);
            assert(result != NeoBoardTime::Result::Accepted);
            rejected += result == NeoBoardTime::Result::Rejected;
        }
        assert(rejected == 1 && !parser.active());
    }
}

void checkIntegration() {
    setup();
    boundary();
    TC before = current();
    auto timerA = TCCR1A, timerB = TCCR1B, timerMask = TIMSK1;
    auto halfBit = halfBitUnits;
    unsigned writes = EEPROM.writes;
    Serial.output.clear();
    input("STX,100,11,2400:102000,");
    assert(!timeSetPending);
    input("5f200647");
    assert(!timeSetPending); // Require a terminator, even with a complete CRC.
    input("\r\n");
    assert(timeSetPending && neoTimeAccepted == 1 && neoTimeRejected == 0);
    assert(equal(current(), before));
    assert(Serial.output == "STX,100,1,1,51de003a\r\n");
    assert(!Serial.blockedWrites);
    assert(equal(requested(), timecodeFromMilliseconds(102000)));
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(102000)));
    assert(OledTimecode::wanted.h == curH && OledTimecode::wanted.m == curM &&
           OledTimecode::wanted.s == curS);

    // A corrupt packet leaves the running clock alone; the next packet recovers.
    input("STX,100,11,2400:102000,00000000\n");
    assert(!timeSetPending && neoTimeRejected == 1);
    input(secondPacket);
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(104000)));
    assert(halfBitUnits == halfBit && EEPROM.writes == writes);
    assert(TCCR1A == timerA && TCCR1B == timerB && TIMSK1 == timerMask);

    // Valid latest packet wins before a boundary; midnight / GPS week rollover.
    input(firstPacket);
    input("STX,100,13,2400:86400000,502e884\n");
    boundary();
    assert(equal(current(), TC{0, 0, 0, 0}));
    input("STX,100,15,65535:604799999,58b9fa9d\n");
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(86399999)));
    input("STX,100,3,0:0,271b0b45\n");
    boundary();
    assert(equal(current(), TC{0, 0, 0, 0}));

    // Timeout arithmetic survives micros() wrap; a fresh line then succeeds.
    simulatedMicros = UINT32_MAX - 100;
    input("STX,100,");
    simulatedMicros += 500001UL;
    serviceSerialInput();
    assert(!neoTimeInput.active() && !timeSetPending && neoTimeRejected == 2);
    input(firstPacket);
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(102000)));

    // EEPROM busy must not stop time reception. U/D also remain queued in order.
    input("u");
    serviceCalibration();
    assert(calibrationStep);
    ready = false;
    input(secondPacket);
    input("d");
    assert(timeSetPending && calibrationQueueCount == 1);
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(104000)));
    ready = true;
    for (unsigned i = 0; i < 2000 && (calibrationStep || calibrationQueueCount); ++i) {
        TIMER1_COMPA_vect(); prepareNextFrame(); serviceCalibration();
    }
    assert(!calibrationStep && !calibrationQueueCount && halfBitUnits == halfBit);
    uint16_t saved;
    assert(readCalibration(saved) && saved == halfBit);

    // Invalid long lines cannot grow memory or execute embedded calibration keys.
    input("STX," + std::string(1000, 'u') + "\n");
    assert(!calibrationQueueCount && !timeSetPending && neoTimeRejected == 3);
    input("T,12,34,56,000\n"); // Existing manual time protocol still works.
    boundary();
    assert(equal(current(), timecodeFromMilliseconds(45296000)));
}

void acknowledgeOled() {
    using OledTimecode::State;
    switch (OledTimecode::state) {
        case State::Start: TWCR.complete(TW_START); break;
        case State::AddressAck: TWCR.complete(TW_MT_SLA_ACK); break;
        case State::DataAck: TWCR.complete(TW_MT_DATA_ACK); break;
        case State::Stop: TWCR.stopComplete(); break;
        default: break;
    }
}

void checkTwoSecondStream() {
    simulatedMillis = simulatedMicros = 0;
    setup();
    uint32_t ticks = 0, nextPacketTicks = 4000000;
    unsigned replyWrites = Serial.writeCalls;
    // Fifteen seconds with packets every two seconds, then five seconds without
    // packets: all BMC frames keep exactly the same duration, including resyncs.
    for (unsigned frame = 0; frame < 600; ++frame) {
        assert(txBitIdx == 0 && !midBitEdgePending);
        TC expected = current();
        if (timeSetPending) expected = requested();
        else if (frameStarted) advanceTC_local(expected);
        TIMER1_COMPA_vect();
        assert(equal(current(), expected));
        uint8_t payload[10]; buildFrame(payload, expected);
        for (uint8_t i = 0; i < 10; ++i) assert(txBuf[i] == payload[i]);
        uint32_t frameTicks = 0;
        do {
            uint16_t edgeTicks = OCR1A + 1;
            ticks += edgeTicks;
            frameTicks += edgeTicks;
            simulatedMicros = ticks / 2;
            simulatedMillis = ticks / 2000;
            if (ticks >= nextPacketTicks && nextPacketTicks < 30000000) {
                // Alternate known fixtures to force visible clock corrections.
                Serial.input += (nextPacketTicks / 4000000) % 2 ? firstPacket : secondPacket;
                nextPacketTicks += 4000000;
            }
            for (uint8_t i = 0; i < 4; ++i) { acknowledgeOled(); loop(); }
            assert(OledTimecode::wantedStatus.gpsTimeRecent == gpsTimeRecent(simulatedMillis));
            assert(OledTimecode::wanted.h == curH && OledTimecode::wanted.m == curM &&
                   OledTimecode::wanted.s == curS);
            if (txBitIdx == 0 && !midBitEdgePending) break;
            TIMER1_COMPA_vect();
        } while (true);
        assert(frameTicks == 160UL * 416);
        assert(framesSent == frame);
    }
    assert(neoTimeAccepted == 7 && !neoTimeRejected);
    assert(Serial.writeCalls == replyWrites + 7 && !Serial.blockedWrites);
    assert(OledTimecode::displayed && OledTimecode::state != OledTimecode::State::Off);
    assert(!OledTimecode::activeStatus.gpsTimeRecent); // Five seconds without packets -> X.
    assert(!TWCR.interruptEnabled);
}

void checkGpsFreshness() {
    simulatedMillis = simulatedMicros = 0;
    setup();
    assert(!gpsTimeRecent(0) && !gpsTimeRecent(3000));
    assert(!OledTimecode::wantedStatus.gpsTimeRecent);
    input(firstPacket); // First valid reception at millis()==0 is still valid.
    assert(gpsTimeRecent(0) && gpsTimeRecent(2999) && !gpsTimeRecent(3000));
    simulatedMillis = 2000;
    input(secondPacket);
    assert(lastGpsTimeMs == 2000 && gpsTimeRecent(4999) && !gpsTimeRecent(5000));

    // Invalid/partial GPS packets and manual clock settings must not refresh G.
    simulatedMillis = 4000;
    input("STX,100,11,2400:102000,00000000\n");
    input("T,12,34,56,000\n");
    input("STX,100,");
    assert(lastGpsTimeMs == 2000 && !gpsTimeRecent(5000));
    simulatedMicros += 500001;
    serviceSerialInput();
    assert(lastGpsTimeMs == 2000);

    simulatedMillis = 5500;
    input(firstPacket); // Recovers without restarting the board.
    assert(gpsTimeRecent(5500));
    simulatedMillis = UINT32_MAX - 1000;
    input(secondPacket);
    assert(gpsTimeRecent(1998) && !gpsTimeRecent(1999)); // millis wrap.
    setup();
    assert(!gpsTimeRecent(simulatedMillis)); // Reboot never shows a stale circle.
}

int main() {
    checkParser();
    checkIntegration();
    checkGpsFreshness();
    checkTwoSecondStream();
    puts("PASS: NeoBoard CRC/ranges, split/timeout/wrap, GPS freshness at boot/2999/3000 ms/recovery, boundary sync, OLED, EEPROM, two-second stream, uninterrupted LTC frames");
}
