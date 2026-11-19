#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#define F_CPU 16000000UL
#define _BV(n) (1U << (n))
#define F(s) reinterpret_cast<const __FlashStringHelper*>(s)
struct __FlashStringHelper;
#define ISR(v) void v()
enum {PB1=1, PB5=5, PD7=7, COM1A0=6, CS11=1, WGM12=3,
      OCIE1A=1, OCF1A=1, OCF1B=2, TOV1=0};
enum {OUTPUT=1, INPUT=0, INPUT_PULLUP=2, HIGH=1, A4=18, A5=19};
enum {TWINT=7, TWSTA=5, TWSTO=4, TWEN=2, TWIE=0};
inline uint8_t TWBR, TWSR, TWDR;
struct TwiControlMock {
    uint8_t value = 0;
    bool interruptEnabled = false;
    operator uint8_t() const { return value; }
    void operator=(uint8_t written) {
        interruptEnabled |= bool(written & _BV(TWIE));
        value = written & ~_BV(TWINT); // TWINT is write-one-to-clear.
    }
    void complete(uint8_t status) {
        TWSR = status;
        value = (value & ~_BV(TWSTA)) | _BV(TWINT);
    }
    void stopComplete() { value &= ~_BV(TWSTO); }
};
inline TwiControlMock TWCR;
inline uint8_t PINB, PORTB, PORTD, TCCR1A, TCCR1B, TCCR1C, TIMSK1, TIFR1;
inline uint16_t OCR1A, TCNT1;
inline void noInterrupts() {}
inline void interrupts() {}
inline void pinMode(int,int) {}
inline uint8_t simulatedPins[32] = {};
inline int digitalRead(int pin) { return simulatedPins[pin]; }
inline uint32_t simulatedMillis = 0;
inline unsigned long millis() { return simulatedMillis; }
inline uint32_t simulatedMicros = 0;
inline uint32_t micros() { return simulatedMicros; }
struct SerialMock {
    std::string input;
    std::string output;
    int txSpace = 63;
    unsigned writeCalls = 0, blockedWrites = 0;
    bool drainWrites = true;
    void begin(int) {}
    int available() { return input.size(); }
    char read() { char c = input[0]; input.erase(0, 1); return c; }
    int availableForWrite() { return txSpace; }
    size_t write(const uint8_t* bytes, size_t count) {
        ++writeCalls;
        if (int(count) > txSpace) { ++blockedWrites; return 0; }
        output.append(reinterpret_cast<const char*>(bytes), count);
        if (!drainWrites) txSpace -= count;
        return count;
    }
    void print(const __FlashStringHelper* value) { output += reinterpret_cast<const char*>(value); }
    void print(char value) { output += value; }
    void print(const char* value) { output += value; }
    template<class T> void print(T value) { output += std::to_string(value); }
    template<class T> void println(T value) { print(value); output += '\n'; }
};
inline SerialMock Serial;
