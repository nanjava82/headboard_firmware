#include <Arduino.h>
#include "frame_timing.h"

/*
 * LTC Encoder + Single-Camera REC Toggle for Arduino Nano (ATmega328P @ 16 MHz)
 *
 * Pins:
 *   D9  - LTC output (PB1/OC1A), hardware-toggled by Timer1 compare
 *   D7  - release output (PD7), momentary toggle pulse -> camera input via optocoupler
 *   D4  - user record/stop button input (PD4, external pull-down, press = HIGH)
 *
 * Recording model (IMPORTANT):
 *   The Canon E3 remote terminal does NOT use a coded pulse train. A single momentary
 *   contact closure TOGGLES record start/stop. So D7 is pulsed HIGH for REC_PULSE_MS
 *   (optocoupler closes the camera contact), then released LOW. The next accepted
 *   pulse is delayed by REC_GAP_MS after release LOW so the camera inputs can settle.
 *   Each button press = one toggle. We track the intended state in isRecording.
 *
 * REC/STOP timestamp log:
 *   Right after the D7 pulse is fired, we log one line:
 *       "녹화 - HH:MM:SS;FF"   (record started)
 *       "정지 - HH:MM:SS;FF"   (record stopped)
 *   The time printed is the LTC timecode at that instant -- i.e. the SAME timecode
 *   that is being encoded on D9 and recorded by the camera. This lets you line up the
 *   moment you pressed the button against the timecode burned into the recorded video.
 *
 *   Frame note: 29.97DF LTC is represented as a 30-frame address with drop-frame
 *   numbering. The frame field therefore uses 00-29, with 00 and 01 skipped at the
 *   start of most minutes. Logging FF directly is easier to compare with camera
 *   timecode than converting FF to approximate milliseconds.
 *
 *   The serial monitor must be set to UTF-8 to display the Korean labels correctly.
 *
 */

#define FPS              30
#define DROP_FRAME       1

/* 
구형 나노 보드 : 417   (33.40ms)
신형 나노 보드 : 415.5 ~ 416  (33.32ms)
*/
#define HALF_BIT_TICKS   415.5
static_assert(HALF_BIT_TICKS >= 300 && HALF_BIT_TICKS <= 500,
              "Timing supports 300..500 ticks per half bit");

#define BTN_DEBOUNCE_MS  50UL
#define REC_PULSE_MS     800UL
#define REC_GAP_MS       1000UL

#define SERIAL_BAUD      115200
#define TC_REPORT_MS     1000UL

#define LTC_OUT_PIN      9
#define RELEASE_PIN      7
#define BUTTON_PIN       4

#define LTC_LOW()           (PORTB &= ~_BV(PB1))
#define RELEASE_HIGH()      (PORTD |= _BV(PD7))
#define RELEASE_LOW()       (PORTD &= ~_BV(PD7))

#define FRAME_BITS 80
static const uint16_t halfBitTenths = uint16_t(HALF_BIT_TICKS * 10 + 0.5);
volatile uint8_t tickRemainder = 0;

// Distribute fractional timer ticks without adjusting the configured period.
static inline uint16_t fixedTicks(uint8_t halves) {
    uint16_t numerator = halfBitTenths * halves + tickRemainder;
    tickRemainder = numerator % 10U;
    return numerator / 10U;
}

volatile uint8_t frameBufA[10];
volatile uint8_t frameBufB[10];
volatile uint8_t* volatile txBuf = frameBufA;
volatile uint8_t* volatile buildBuf = frameBufB;
volatile bool bufferSwapRequest = false;
volatile uint32_t framesSent = 0;
volatile uint8_t txBitIdx = 0;
volatile bool midBitEdgePending = false;
volatile FrameTiming frameTiming;

struct TC {
    uint8_t H;
    uint8_t M;
    uint8_t S;
    uint8_t F;
};

volatile uint8_t curH = 0;
volatile uint8_t curM = 0;
volatile uint8_t curS = 0;
volatile uint8_t curF = 0;

bool isRecording = false;
bool pulseActive = false;
bool pendingToggle = false;
unsigned long pulseStartMs = 0;
unsigned long pulseEndMs = 0;

bool btnStable = false;
bool btnReading = false;
unsigned long btnChangeMs = 0;


unsigned long lastTCReportMs = 0;
uint32_t lastBuiltFrame = 0;

static inline void setBit(volatile uint8_t* buf, uint8_t bitIdx, uint8_t val) {
    uint8_t byteIdx = bitIdx >> 3;
    uint8_t bitInByte = bitIdx & 7;
    if (val) {
        buf[byteIdx] |= (1 << bitInByte);
    } else {
        buf[byteIdx] &= ~(1 << bitInByte);
    }
}

static inline uint8_t getBit(const volatile uint8_t* buf, uint8_t bitIdx) {
    return (buf[bitIdx >> 3] >> (bitIdx & 7)) & 1;
}

static inline void advanceTC_local(TC& tc) {
    tc.F++;
    if (tc.F >= FPS) {
        tc.F = 0;
        tc.S++;
        if (tc.S >= 60) {
            tc.S = 0;
            tc.M++;
            if (tc.M >= 60) {
                tc.M = 0;
                tc.H++;
                if (tc.H >= 24) {
                    tc.H = 0;
                }
            }
#if DROP_FRAME
            if ((tc.M % 10) != 0) {
    #if FPS == 30
                tc.F = 2;
    #elif FPS == 60
                tc.F = 4;
    #endif
            }
#endif
        }
    }
}

void buildFrame(volatile uint8_t* buf, TC tc) {
    for (uint8_t i = 0; i < 10; i++) {
        buf[i] = 0;
    }

    uint8_t fu = tc.F % 10;
    uint8_t ft = tc.F / 10;
    if (fu & 1) setBit(buf, 0, 1);
    if (fu & 2) setBit(buf, 1, 1);
    if (fu & 4) setBit(buf, 2, 1);
    if (fu & 8) setBit(buf, 3, 1);
    if (ft & 1) setBit(buf, 8, 1);
    if (ft & 2) setBit(buf, 9, 1);
#if FPS == 60
    if (ft & 4) setBit(buf, 10, 1);
#else
    if (DROP_FRAME) setBit(buf, 10, 1);
#endif

    uint8_t su = tc.S % 10;
    uint8_t st = tc.S / 10;
    if (su & 1) setBit(buf, 16, 1);
    if (su & 2) setBit(buf, 17, 1);
    if (su & 4) setBit(buf, 18, 1);
    if (su & 8) setBit(buf, 19, 1);
    if (st & 1) setBit(buf, 24, 1);
    if (st & 2) setBit(buf, 25, 1);
    if (st & 4) setBit(buf, 26, 1);

    uint8_t mu = tc.M % 10;
    uint8_t mt = tc.M / 10;
    if (mu & 1) setBit(buf, 32, 1);
    if (mu & 2) setBit(buf, 33, 1);
    if (mu & 4) setBit(buf, 34, 1);
    if (mu & 8) setBit(buf, 35, 1);
    if (mt & 1) setBit(buf, 40, 1);
    if (mt & 2) setBit(buf, 41, 1);
    if (mt & 4) setBit(buf, 42, 1);

    uint8_t hu = tc.H % 10;
    uint8_t ht = tc.H / 10;
    if (hu & 1) setBit(buf, 48, 1);
    if (hu & 2) setBit(buf, 49, 1);
    if (hu & 4) setBit(buf, 50, 1);
    if (hu & 8) setBit(buf, 51, 1);
    if (ht & 1) setBit(buf, 56, 1);
    if (ht & 2) setBit(buf, 57, 1);

    buf[8] = 0xFC;
    buf[9] = 0xBF;

    uint8_t totalOnes = 0;
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t b = buf[i];
        while (b) {
            b &= (b - 1);
            totalOnes++;
        }
    }
    if (totalOnes & 1) {
        if (FPS == 25) {
            setBit(buf, 59, 1);
        } else {
            setBit(buf, 27, 1);
        }
    }
}

static inline void advanceTC_isr() {
    TC tc = { curH, curM, curS, curF };
    advanceTC_local(tc);
    curH = tc.H;
    curM = tc.M;
    curS = tc.S;
    curF = tc.F;
}

static inline void finishBit_isr() {
    txBitIdx++;
    if (txBitIdx >= FRAME_BITS) {
        txBitIdx = 0;
        framesSent++;

        if (bufferSwapRequest) {
            volatile uint8_t* tmp = txBuf;
            txBuf = buildBuf;
            buildBuf = tmp;
            bufferSwapRequest = false;
            advanceTC_isr();
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    // D9/OC1A is toggled by Timer1 hardware at the compare instant.
    // The ISR only schedules the next edge, so code-path variation does not move D9.
    if (midBitEdgePending) {
        midBitEdgePending = false;
        finishBit_isr();
        OCR1A = fixedTicks(1) - 1;
        return;
    }

    // The prior ISR may advance the frame counter before bit 79 has ended.
    // Observe the actual bit-0 boundary ISR instead; micros includes ISR latency.
    const bool frameStart = (txBitIdx == 0);
    const uint32_t frameStartUs = frameStart ? micros() : 0;
    if (frameStart) {
        if (!frameTiming.primed) tickRemainder = 0;
        frameTiming.record(frameStartUs);
    }
    if (getBit(txBuf, txBitIdx)) {
        midBitEdgePending = true;
        OCR1A = fixedTicks(1) - 1;
    } else {
        finishBit_isr();
        OCR1A = fixedTicks(2) - 1;
    }
}

void prepareNextFrame() {
    uint32_t curSent;
    noInterrupts();
    curSent = framesSent;
    interrupts();

    if (curSent == lastBuiltFrame) {
        return;
    }
    if (bufferSwapRequest) {
        return;
    }

    TC tc;
    noInterrupts();
    tc.H = curH;
    tc.M = curM;
    tc.S = curS;
    tc.F = curF;
    interrupts();

    TC next = tc;
    advanceTC_local(next);

    buildFrame(buildBuf, next);
    bufferSwapRequest = true;
    lastBuiltFrame = curSent;
}

bool recToggle();

bool canStartRecPulse() {
    return !pulseActive && (pulseEndMs == 0 || millis() - pulseEndMs >= REC_GAP_MS);
}

void startRecPulse() {
    RELEASE_HIGH();
    pulseStartMs = millis();
    pulseActive = true;
}

void servicePulse() {
    if (pulseActive && (millis() - pulseStartMs >= REC_PULSE_MS)) {
        RELEASE_LOW();
        pulseEndMs = millis();
        pulseActive = false;
    }

    if (pendingToggle && canStartRecPulse()) {
        pendingToggle = false;
        recToggle();
    }
}

void logRecEvent(bool recording) {
    uint8_t h, m, s, f;
    noInterrupts();
    h = curH;
    m = curM;
    s = curS;
    f = curF;
    interrupts();

    Serial.print(recording ? F("녹화 - ") : F("정지 - "));

    if (h < 10) Serial.print('0');
    Serial.print(h);
    Serial.print(':');
    if (m < 10) Serial.print('0');
    Serial.print(m);
    Serial.print(':');
    if (s < 10) Serial.print('0');
    Serial.print(s);
    Serial.print(DROP_FRAME ? ';' : ':');
    if (f < 10) Serial.print('0');
    Serial.println(f);
}

bool recToggle() {
    if (!canStartRecPulse()) {
        return false;
    }
    isRecording = !isRecording;
    startRecPulse();

    Serial.print(F("REC,"));
    Serial.println(isRecording ? '1' : '0');

    logRecEvent(isRecording);
    return true;
}

void monitorButton() {
    bool raw = (digitalRead(BUTTON_PIN) == HIGH);

    if (raw != btnReading) {
        btnReading = raw;
        btnChangeMs = millis();
    } else if (millis() - btnChangeMs >= BTN_DEBOUNCE_MS) {
        if (btnStable != raw) {
            btnStable = raw;
            if (btnStable) {
                if (!recToggle()) {
                    pendingToggle = true;
                }
            }
        }
    }
}


void printTCLine(TC tc) {
    Serial.print(F("TC,"));
    if (tc.H < 10) Serial.print('0');
    Serial.print(tc.H);
    Serial.print(':');
    if (tc.M < 10) Serial.print('0');
    Serial.print(tc.M);
    Serial.print(':');
    if (tc.S < 10) Serial.print('0');
    Serial.print(tc.S);
    Serial.print(DROP_FRAME ? ';' : ':');
    if (tc.F < 10) Serial.print('0');
    Serial.print(tc.F);
}

void printFrameTiming() {
    uint32_t last, samples;
    uint16_t tenths;
    noInterrupts();
    last = frameTiming.last;
    samples = frameTiming.samples;
    tenths = halfBitTenths;
    interrupts();
    Serial.print(F("     LAST,"));
    if (samples) Serial.print(last);
    else Serial.print(F("WAIT"));
    Serial.print(F(" ("));
    Serial.print(FrameTiming::minimumUs);
    Serial.print('_');
    Serial.print(FrameTiming::maximumUs);
    Serial.print(F(")     HALF_BIT_TICKS,"));
    Serial.print(tenths / 10);
    Serial.print('.');
    Serial.println(tenths % 10);
}


void periodicTCReport() {
#if TC_REPORT_MS > 0
    unsigned long now = millis();
    if (now - lastTCReportMs < TC_REPORT_MS) {
        return;
    }
    lastTCReportMs = now;

    TC tc;
    noInterrupts();
    tc.H = curH;
    tc.M = curM;
    tc.S = curS;
    tc.F = curF;
    interrupts();

    printTCLine(tc);
    printFrameTiming();
#endif
}

void setup() {
    Serial.begin(SERIAL_BAUD);

    pinMode(LTC_OUT_PIN, OUTPUT);
    LTC_LOW();
    pinMode(RELEASE_PIN, OUTPUT);
    RELEASE_LOW();

    pinMode(BUTTON_PIN, INPUT);
    btnReading = (digitalRead(BUTTON_PIN) == HIGH);
    btnStable = btnReading;
    btnChangeMs = millis();

    TC first = { 0, 0, 0, 0 };
    TC second = first;
    advanceTC_local(second);

    curH = first.H;
    curM = first.M;
    curS = first.S;
    curF = first.F;
    buildFrame(txBuf, first);
    buildFrame(buildBuf, second);
    bufferSwapRequest = true;
    lastBuiltFrame = 0;
    txBitIdx = 0;
    midBitEdgePending = false;
    framesSent = 0;

    noInterrupts();
    TCCR1A = _BV(COM1A0);  // hardware toggle OC1A/D9 on every compare match
    TCCR1B = 0;
    TCNT1 = 0;
    tickRemainder = 0;
    OCR1A = fixedTicks(1) - 1;
    TCCR1B |= _BV(WGM12);
    TCCR1B |= _BV(CS11);
    TIMSK1 |= _BV(OCIE1A);
    interrupts();

    Serial.println(F("READY"));
    Serial.print(F("FPS,"));
    Serial.print(FPS);
    Serial.println(DROP_FRAME ? F("DF") : F("NDF"));
}

void loop() {
    prepareNextFrame();   // D9에서 다음에 송출할 타임코드 프레임 준비
    servicePulse();       // D7 녹화 펄스 종료 및 대기 중인 요청 처리
    monitorButton();      // D4 버튼 입력을 확인하여 D7 녹화/정지 요청
    periodicTCReport();   // 1초마다 타임코드와 프레임 측정값 출력
}
