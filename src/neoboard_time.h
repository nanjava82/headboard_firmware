#pragma once

#include <stdint.h>

// NeoBoard_Rev2/Src/uart4_time_test.c, UART4 at 115200 8N1, every 2000 ms:
// The sender extracts week/ms from the INSPVAX long header (message ID 1465).
// UART4 carries the ASCII time envelope below, not the INSPVAX binary body;
// switching the source log from INSPVA to INSPVAX does not change this format.
// STX,100,<payload length>,<GPS week>:<GPS milliseconds of week>,<CRC hex>\r\n
// CRC covers only the ASCII payload: reflected 0xEDB88320, init 0, no final XOR.
// GPS time is deliberately kept as GPS time (no UTC/KST or leap-second offset).
// Week is range-checked but only time-of-day is carried by our LTC/OLED.
// All parsing/CRC runs in loop(), with at most eight CRC iterations per byte.
namespace NeoBoardTime {
constexpr uint32_t DayMs = 86400000UL;
constexpr uint32_t WeekMs = 604800000UL;
enum class Result : uint8_t { Incomplete, Accepted, Rejected };

class Parser {
    enum class State : uint8_t { Idle, Prefix, Length, Week, Milliseconds, Crc, Discard };
    State state = State::Idle;
    uint8_t prefixIndex = 0, digits = 0, length = 0, payloadCount = 0;
    uint32_t value = 0, crc = 0, receivedCrc = 0;

    void payloadByte(uint8_t c) {
        ++payloadCount;
        crc ^= c;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    }

public:
    uint32_t milliseconds = 0;
    bool active() const { return state != State::Idle; }
    // Retained through the terminating CR/LF, so only identified command 100
    // requests receive a reply. Noise and other command numbers are not ACKed.
    bool timeCommand() const { return prefixIndex == 8; }
    void reset() { state = State::Idle; }

    Result feed(char c) {
        if (c == '\r' || c == '\n') {
            if (!active()) return Result::Incomplete; // CRLF's second terminator.
            bool valid = state == State::Crc && digits && receivedCrc == crc;
            reset();
            return valid ? Result::Accepted : Result::Rejected;
        }
        if (!active()) {
            state = State::Prefix;
            prefixIndex = digits = length = payloadCount = 0;
            value = crc = receivedCrc = milliseconds = 0;
        }
        if (state == State::Prefix) {
            if (c != "STX,100,"[prefixIndex]) state = State::Discard;
            else if (++prefixIndex == 8) state = State::Length;
        } else if (state == State::Length) {
            if (c >= '0' && c <= '9' && ++digits <= 2) {
                length = length * 10 + c - '0';
                if (length > 15) state = State::Discard;
            } else if (c == ',' && digits && length >= 3) {
                state = State::Week;
                digits = 0;
            } else state = State::Discard;
        } else if (state == State::Week || state == State::Milliseconds) {
            bool week = state == State::Week;
            if (c >= '0' && c <= '9' && ++digits <= (week ? 5 : 9)) {
                value = value * 10 + c - '0';
                payloadByte(c);
                if (value > (week ? 65535UL : WeekMs - 1) || payloadCount > length)
                    state = State::Discard;
            } else if (week && c == ':' && digits) {
                payloadByte(c);
                state = State::Milliseconds;
                value = 0;
                digits = 0;
            } else if (!week && c == ',' && digits && payloadCount == length) {
                milliseconds = value;
                state = State::Crc;
                digits = 0;
            } else state = State::Discard;
        } else if (state == State::Crc) {
            uint8_t nibble;
            if (c >= '0' && c <= '9') nibble = c - '0';
            else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
            else { state = State::Discard; return Result::Incomplete; }
            if (++digits > 8) state = State::Discard;
            else receivedCrc = (receivedCrc << 4) | nibble;
        }
        // Discard a damaged/overlong packet through its terminator. No allocation,
        // blocking read, per-error serial write, or growing receive buffer.
        return Result::Incomplete;
    }
};
} // namespace NeoBoardTime
