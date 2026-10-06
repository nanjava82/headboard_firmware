#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <util/twi.h>

// SSD1306 128x32, J12: GND / +5V / A5(SCL) / A4(SDA).
// Exclusive owner of AVR TWI: do not use Wire alongside this driver.
// TWIE stays off. Each service() call advances at most one bus state and never
// waits, disables global interrupts, or touches the LTC timer. Hardware holds
// SCL low between bytes until the main loop can supply the next byte.
namespace OledTimecode {
constexpr uint8_t Address = 0x3C;
constexpr uint8_t Contrast = 0x7F;
constexpr uint8_t TextX = 25, TextY = 16;
constexpr uint8_t TextWidth = 96; // Eight 12x16 cells: HH:MM:SS.
constexpr uint32_t BusTimeoutMs = 25;
constexpr uint8_t TimeoutError = 0xFF;

static const uint8_t Glyphs[][5] PROGMEM = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P (11)
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
};
static const uint8_t Init[] PROGMEM = {
    0x00, // Command control byte.
    0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02,
    0x81, Contrast, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E
};

enum class State : uint8_t { Off, PowerWait, Start, AddressAck, DataAck, Stop, Idle };
enum class Packet : uint8_t { Init, ClearWindow, ClearData, On, TextWindow, TextData };
struct Time { uint8_t h, m, s; };
static State state = State::Off;
static Packet packet;
static Time wanted = {0, 0, 0}, active = {0, 0, 0};
struct Status { uint16_t units; bool recording; };
static Status wantedStatus = {8320, false}, activeStatus = {8320, false};
static uint8_t digits[8];
static uint8_t periodDigits[10]; // Per:416.05, 5x7 text; following space reserved.
static uint8_t firstPage = 0, pageCount = 4;
static bool displayed = false;
static uint8_t errorPending = 0;
static uint16_t position;
static uint32_t lastStepMs;

inline void request(uint8_t h, uint8_t m, uint8_t s) {
    // Main-loop only. Keep the latest request; never change digits mid-transfer.
    wanted = {h, m, s};
}

inline void requestStatus(uint16_t units, bool recording) {
    wantedStatus = {units, recording};
}

inline void requestRecording(bool recording) { wantedStatus.recording = recording; }

inline uint8_t glyphColumn(uint8_t glyph, uint8_t column) {
    return column < 5 ? pgm_read_byte(&Glyphs[glyph][column]) : 0;
}

inline uint8_t pixelByte(uint16_t offset) {
    uint8_t x = offset % 128;
    uint8_t page = offset / 128;
    uint32_t pixels = 0;
    if (page < 2) {
        if (x >= 3 && x < 3 + sizeof(periodDigits) * 6)
            pixels |= uint32_t(glyphColumn(periodDigits[(x - 3) / 6], (x - 3) % 6)) << 3;
        if (x >= 90 && x < 95) pixels |= uint32_t(glyphColumn(19, x - 90)) << 3;
        if (x >= 109 && x < 114) pixels |= uint32_t(glyphColumn(20, x - 109)) << 3;
        // R: filled square at STOP, approximately equilateral right-pointing PLAY.
        if (x >= 97 && x <= 103) {
            for (int8_t y = -3; y <= 3; ++y) {
                uint8_t dy = y < 0 ? -y : y;
                if (!activeStatus.recording || (x >= 98 && (x - 98) * 3 <= (3 - dy) * 5))
                    pixels |= 1UL << (6 + y);
            }
        }
        // G is only a fixed white circle placeholder, not a GNSS lock indication.
        if (x >= 117 && x <= 123) {
            int8_t dx = int8_t(x) - 120;
            for (int8_t y = -3; y <= 3; ++y)
                if (dx * dx + y * y <= 10) pixels |= 1UL << (6 + y);
        }
    } else {
        if (x >= 3 && x < 21) {
            uint8_t label = (x - 3) / 6;
            pixels |= uint32_t(glyphColumn(label == 0 ? 21 : label == 1 ? 22 : 10, (x - 3) % 6)) << 20;
        }
        if (x >= TextX && x < TextX + TextWidth) {
            uint8_t local = x - TextX;
            uint8_t glyph = glyphColumn(digits[local / 12], (local % 12) / 2);
            uint16_t doubled = 0;
            for (uint8_t bit = 0; bit < 7; ++bit)
                if (glyph & (1U << bit)) doubled |= uint16_t(3) << (bit * 2);
            pixels |= uint32_t(doubled) << TextY;
        }
    }
    return uint8_t(pixels >> (page * 8));
}

inline uint16_t packetSize() {
    switch (packet) {
        case Packet::Init: return sizeof(Init);
        case Packet::ClearWindow: case Packet::TextWindow: return 7;
        case Packet::ClearData: return 513;
        case Packet::On: return 2;
        case Packet::TextData: return 1 + uint16_t(pageCount) * 128;
    }
    return 0;
}

inline uint8_t packetByte() {
    if (packet == Packet::Init) return pgm_read_byte(&Init[position]);
    if (packet == Packet::ClearData) return position ? 0x00 : 0x40;
    if (packet == Packet::TextData) return position ? pixelByte(firstPage * 128U + position - 1) : 0x40;
    if (packet == Packet::On) return position ? 0xAF : 0x00;
    bool text = packet == Packet::TextWindow;
    switch (position) {
        case 0: return 0x00;
        case 1: return 0x21; // Column range.
        case 2: return 0;
        case 3: return 127;
        case 4: return 0x22; // Page range.
        case 5: return text ? firstPage : 0;
        default: return text ? firstPage + pageCount - 1 : 3;
    }
}

inline void fail(uint8_t status) {
    errorPending = status ? status : 0xFE; // 0xFE = TWI bus error.
    TWCR = 0; // Release AVR bus control; no recovery loops or retries during LTC.
    state = State::Off; // Reboot to retry the display after checking its wiring.
}

inline uint8_t takeError() {
    uint8_t result = errorPending;
    errorPending = 0;
    return result;
}

inline void startPacket(Packet next, uint32_t now) {
    packet = next;
    position = 0;
    lastStepMs = now;
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA); // TWIE deliberately absent.
    state = State::Start;
}

inline void sendByte(uint32_t now) {
    TWDR = packetByte();
    ++position;
    lastStepMs = now;
    TWCR = _BV(TWINT) | _BV(TWEN);
    state = State::DataAck;
}

inline void begin(uint32_t now) {
    TWCR = 0;
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    TWSR = 0; // Prescaler = 1.
    TWBR = (F_CPU / 100000UL - 16) / 2;
    TWCR = _BV(TWEN);
    wanted = active = {0, 0, 0};
    wantedStatus = activeStatus = {8320, false};
    displayed = false;
    errorPending = 0;
    lastStepMs = now;
    state = State::PowerWait;
}

inline void service(uint32_t now) {
    if (state == State::Off) return;
    if (state == State::PowerWait) {
        if (uint32_t(now - lastStepMs) >= 50) startPacket(Packet::Init, now);
        return;
    }
    if (state == State::Idle) {
        bool timeChanged = !displayed || wanted.h != active.h || wanted.m != active.m || wanted.s != active.s;
        bool statusChanged = !displayed || wantedStatus.units != activeStatus.units ||
                             wantedStatus.recording != activeStatus.recording;
        if (!timeChanged && !statusChanged) return;
        firstPage = statusChanged ? 0 : 2;
        pageCount = timeChanged && statusChanged ? 4 : 2;
        active = wanted;
        activeStatus = wantedStatus;
        digits[0] = active.h / 10; digits[1] = active.h % 10; digits[2] = 10;
        digits[3] = active.m / 10; digits[4] = active.m % 10; digits[5] = 10;
        digits[6] = active.s / 10; digits[7] = active.s % 10;
        uint16_t whole = activeStatus.units / 20;
        uint8_t fraction = (activeStatus.units % 20) * 5;
        periodDigits[0] = 11; periodDigits[1] = 12; periodDigits[2] = 13; periodDigits[3] = 10;
        periodDigits[4] = whole / 100; periodDigits[5] = (whole / 10) % 10; periodDigits[6] = whole % 10;
        periodDigits[7] = 14; periodDigits[8] = fraction / 10; periodDigits[9] = fraction % 10;
        startPacket(Packet::TextWindow, now);
        return;
    }
    if (state == State::Stop) {
        if (TWCR & _BV(TWSTO)) {
            if (uint32_t(now - lastStepMs) >= BusTimeoutMs) fail(TimeoutError);
            return;
        }
        switch (packet) {
            case Packet::Init: startPacket(Packet::ClearWindow, now); break;
            case Packet::ClearWindow: startPacket(Packet::ClearData, now); break;
            case Packet::ClearData: startPacket(Packet::On, now); break;
            case Packet::On: state = State::Idle; break;
            case Packet::TextWindow: startPacket(Packet::TextData, now); break;
            case Packet::TextData: displayed = true; state = State::Idle; break;
        }
        return;
    }
    if (!(TWCR & _BV(TWINT))) {
        if (uint32_t(now - lastStepMs) >= BusTimeoutMs) fail(TimeoutError);
        return;
    }
    uint8_t status = TWSR & 0xF8;
    if (state == State::Start) {
        if (status != TW_START && status != TW_REP_START) { fail(status); return; }
        TWDR = Address << 1;
        TWCR = _BV(TWINT) | _BV(TWEN);
        lastStepMs = now;
        state = State::AddressAck;
    } else if (state == State::AddressAck) {
        if (status != TW_MT_SLA_ACK) { fail(status); return; }
        sendByte(now);
    } else {
        if (status != TW_MT_DATA_ACK) { fail(status); return; }
        if (position < packetSize()) sendByte(now);
        else {
            TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
            lastStepMs = now;
            state = State::Stop;
        }
    }
}
} // namespace OledTimecode
