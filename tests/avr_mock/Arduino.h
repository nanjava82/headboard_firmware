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
enum {OUTPUT=1, INPUT=0, HIGH=1};
inline uint8_t PINB, PORTB, PORTD, TCCR1A, TCCR1B, TCCR1C, TIMSK1, TIFR1;
inline uint16_t OCR1A, TCNT1;
inline void noInterrupts() {}
inline void interrupts() {}
inline void pinMode(int,int) {}
inline int digitalRead(int) { return 0; }
inline unsigned long millis() { return 0; }
inline uint32_t simulatedMicros = 0;
inline uint32_t micros() { return simulatedMicros; }
struct SerialMock {
    std::string input;
    void begin(int) {}
    int available() { return input.size(); }
    char read() { char c = input[0]; input.erase(0, 1); return c; }
    template<class T> void print(T) {}
    template<class T> void println(T) {}
};
inline SerialMock Serial;
