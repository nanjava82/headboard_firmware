#include <cassert>
#include <vector>
#define OLED_TIMECODE 1
#include "../src/main.cpp"

using OledTimecode::State;
using OledTimecode::Packet;
using OledTimecode::Time;
std::vector<std::vector<uint8_t>> packets;
std::vector<Time> displayedTimes;
std::vector<uint8_t> bytes;

void step(uint32_t now) {
    State before = OledTimecode::state;
    Packet packet = OledTimecode::packet;
    if (before == State::Start) TWCR.complete(TW_START);
    if (before == State::AddressAck) TWCR.complete(TW_MT_SLA_ACK);
    if (before == State::DataAck) TWCR.complete(TW_MT_DATA_ACK);
    if (before == State::Stop) TWCR.stopComplete();
    OledTimecode::service(now);
    if (before == State::Start && OledTimecode::state == State::AddressAck) {
        assert(TWDR == 0x78);
        bytes.clear();
    }
    if ((before == State::AddressAck || before == State::DataAck) &&
        OledTimecode::state == State::DataAck) bytes.push_back(TWDR);
    if (before == State::DataAck && OledTimecode::state == State::Stop)
        packets.push_back(bytes);
    if (before == State::Stop && packet == Packet::TextData)
        displayedTimes.push_back(OledTimecode::active);
    assert(!TWCR.interruptEnabled);
    assert(OledTimecode::state != State::Off);
}

void settle(uint32_t now) {
    for (unsigned i = 0; i < 2000; ++i) {
        step(now);
        if (OledTimecode::state == State::Idle && OledTimecode::displayed &&
            OledTimecode::wanted.h == OledTimecode::active.h &&
            OledTimecode::wanted.m == OledTimecode::active.m &&
            OledTimecode::wanted.s == OledTimecode::active.s) return;
    }
    assert(false && "OLED transfer did not finish");
}

void faultChecks() {
    // No response at START: bounded timeout, no retries, one error notification.
    OledTimecode::begin(0);
    OledTimecode::service(50);
    OledTimecode::service(74);
    assert(OledTimecode::state == State::Start);
    OledTimecode::service(75);
    assert(OledTimecode::state == State::Off && uint8_t(TWCR) == 0);
    assert(OledTimecode::takeError() == 255 && OledTimecode::takeError() == 0);
    OledTimecode::service(1000);
    assert(OledTimecode::state == State::Off);

    for (uint8_t status : {TW_MT_SLA_NACK, TW_MT_ARB_LOST, 0}) {
        OledTimecode::begin(0);
        step(50); step(50); // START -> address.
        TWCR.complete(status);
        OledTimecode::service(50);
        assert(OledTimecode::state == State::Off);
        assert(OledTimecode::takeError() == (status ? status : 254));
    }
    OledTimecode::begin(0);
    step(50); step(50); step(50); // First command byte in flight.
    TWCR.complete(TW_MT_DATA_NACK);
    OledTimecode::service(50);
    assert(OledTimecode::takeError() == TW_MT_DATA_NACK);

    OledTimecode::begin(0);
    do { step(50); } while (OledTimecode::state != State::Stop);
    OledTimecode::service(75); // STOP bit never clears.
    assert(OledTimecode::state == State::Off && OledTimecode::takeError() == 255);

    OledTimecode::begin(UINT32_MAX - 20);
    OledTimecode::service(28);
    assert(OledTimecode::state == State::PowerWait);
    OledTimecode::service(29);
    assert(OledTimecode::state == State::Start);
    OledTimecode::service(54);
    assert(OledTimecode::takeError() == 255);
}

int main() {
    OledTimecode::begin(0);
    OledTimecode::request(12, 34, 56);
    settle(50);
    assert(TWBR == 72);
    assert(packets.size() == 6);
    assert(packets[2].size() == 513 && packets[2][0] == 0x40);
    for (unsigned i = 1; i < packets[2].size(); ++i) assert(packets[2][i] == 0);
    assert((packets[4] == std::vector<uint8_t>{0, 0x21, 0, 127, 0x22, 0, 3}));
    assert(packets[5].size() == 513 && packets[5][0] == 0x40);
    assert((std::vector<uint8_t>(OledTimecode::digits, OledTimecode::digits + 8) ==
            std::vector<uint8_t>{1, 2, 10, 3, 4, 10, 5, 6}));
    size_t count = packets.size();
    for (unsigned i = 0; i < 100; ++i) step(51);
    assert(packets.size() == count); // No repeated refresh of the same second.

    OledTimecode::request(12, 34, 57);
    do { step(52); } while (OledTimecode::packet != Packet::TextData);
    OledTimecode::request(23, 59, 59);
    OledTimecode::request(0, 0, 0); // Latest request wins, in-flight digits stay frozen.
    assert(OledTimecode::digits[7] == 7 && OledTimecode::active.s == 57);
    settle(52);
    assert(displayedTimes[1].s == 57);
    assert(displayedTimes[2].h == 0 && displayedTimes[2].m == 0 && displayedTimes[2].s == 0);
    assert((packets[packets.size() - 2] == std::vector<uint8_t>{0, 0x21, 0, 127, 0x22, 2, 3}));
    assert(packets.back().size() == 257); // Only the lower half changes each second.

    auto pixel = [](uint8_t x, uint8_t y) {
        return (OledTimecode::pixelByte(uint16_t(y / 8) * 128 + x) >> (y % 8)) & 1;
    };
    assert(pixel(97, 3) && pixel(103, 9)); // Default STOP square.
    assert(pixel(120, 6) && !pixel(117, 3)); // Fixed G circle.
    count = packets.size();
    assert(recToggle() && (PORTD & _BV(PD7)));
    settle(53);
    assert(OledTimecode::activeStatus.recording);
    assert(packets.size() == count + 2);
    assert((packets[count] == std::vector<uint8_t>{0, 0x21, 0, 127, 0x22, 0, 1}));
    assert(!pixel(97, 3) && pixel(98, 3) && pixel(103, 6) && !pixel(103, 3));
    pulseStartMs = 0UL - REC_PULSE_MS;
    servicePulse(); settle(54);
    assert(OledTimecode::activeStatus.recording); // D7 falling edge does not mean STOP.
    pulseEndMs = 0;
    assert(recToggle()); settle(55);
    assert(!OledTimecode::activeStatus.recording && pixel(97, 3));
    OledTimecode::requestStatus(8301, false); // 415.05 ticks.
    settle(56);
    assert(OledTimecode::periodDigits[4] == 4 && OledTimecode::periodDigits[5] == 1 &&
           OledTimecode::periodDigits[6] == 5 && OledTimecode::periodDigits[8] == 0 &&
           OledTimecode::periodDigits[9] == 5);
    faultChecks();

    // Interleave every LTC edge with display traffic across DF and midnight.
    for (TC initial : {TC{0, 0, 59, 28}, TC{23, 59, 59, 28}}) {
        setup();
        packets.clear(); displayedTimes.clear();
        curH = initial.H; curM = initial.M; curS = initial.S; curF = initial.F;
        buildFrame(txBuf, initial);
        TC next = initial; advanceTC_local(next); buildFrame(buildBuf, next);
        TC expected = initial;
        uint32_t ticks = 0;
        for (unsigned frame = 0; frame < 360; ++frame) {
            TIMER1_COMPA_vect();
            assert(framesSent == frame);
            assert(curH == expected.H && curM == expected.M && curS == expected.S && curF == expected.F);
            assert((txBuf[0] & 15) + (txBuf[1] & 3) * 10 == expected.F);
            assert((txBuf[2] & 15) + (txBuf[3] & 7) * 10 == expected.S);
            assert((txBuf[4] & 15) + (txBuf[5] & 7) * 10 == expected.M);
            assert((txBuf[6] & 15) + (txBuf[7] & 3) * 10 == expected.H);
            uint32_t frameTicks = 0;
            do {
                frameTicks += OCR1A + 1; ticks += OCR1A + 1;
                prepareNextFrame();
                step(ticks / 2000); // 2 MHz Timer1; OLED service never advances TC.
                if (txBitIdx == 0 && !midBitEdgePending) break;
                TIMER1_COMPA_vect();
            } while (true);
            assert(frameTicks == 160UL * 416 && framesSent == frame);
            advanceTC_local(expected);
        }
        assert(!displayedTimes.empty() && displayedTimes.size() <= 13);
        bool crossed = false;
        for (Time t : displayedTimes)
            if (t.h == 0 && t.m == (initial.H == 23 ? 0 : 1) && t.s == 0) crossed = true;
        assert(crossed); // Updates even when the new second starts at frame 02.
    }
    assert(!TWCR.interruptEnabled);
    puts("PASS: OLED window/data, once per second, frozen snapshot, latest request, NACK/timeout/wrap, no TWI interrupts, LTC/DF/midnight interleaving");
}
