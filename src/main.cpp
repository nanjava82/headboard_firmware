#include <Arduino.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "frame_timing.h"
#include "neoboard_time.h"
#if OLED_TIMECODE
#include "oled_timecode.h"
#endif

/*
 * LTC Encoder + Single-Camera REC Toggle for Arduino Nano (ATmega328P @ 16 MHz)
 *
 * Pins:
 *   D9  - LTC output (PB1/OC1A), hardware-toggled by Timer1 compare
 *   D7  - release output (PD7), momentary toggle pulse -> camera input via optocoupler
 *   D4  - user record/stop button input (PD4, external pull-down, press = HIGH)
 *   D2/D3 - BTN1/BTN2 calibration up/down, one 0.05-tick step per press
 *   A4/A5 - OLED SDA/SCL at J12; HH:MM:SS updated on timecode second changes
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
 *   Serial input: 'u' increases half-bit ticks by 0.05; 'd' decreases by 0.05.
 *   Each change is saved to EEPROM.
 *   T,HH,MM,SS,mmm followed by CR/LF sets time at the next LTC frame boundary.
 *   Input is already in the desired time zone; transport delay is not corrected.
 *   NeoBoard INSPVAX header time: STX,100,length,GPSweek:GPSms,CRC32hex + CR/LF.
 *   Valid GPS time-of-day is applied at the next frame boundary, without changing
 *   Timer1 phase/period. OLED follows that applied time. No UTC/KST conversion.
 *
 */

#define FPS              30
#define DROP_FRAME       1

/* 
구형 나노 보드 : 417   (33.40ms)
신형 나노 보드 : 415.5 ~ 416  (33.32ms)
*/
#define HALF_BIT_TICKS   416.0
static_assert(HALF_BIT_TICKS >= 410 && HALF_BIT_TICKS <= 420,
              "Timing supports 410..420 ticks per half bit");

#define BTN_DEBOUNCE_MS  50UL
#define REC_PULSE_MS     800UL
#define REC_GAP_MS       1000UL

#define SERIAL_BAUD      115200
#define TC_REPORT_MS     1000UL

#define LTC_OUT_PIN      9
#define RELEASE_PIN      7
#define BUTTON_PIN       4
#define CAL_UP_PIN       2
#define CAL_DOWN_PIN     3

#define LTC_LOW()           (PORTB &= ~_BV(PB1))
#define RELEASE_HIGH()      (PORTD |= _BV(PD7))
#define RELEASE_LOW()       (PORTD &= ~_BV(PD7))

#define FRAME_BITS 80
constexpr uint8_t CalibrationScale = 20; // One integer unit = 0.05 timer ticks.
constexpr uint16_t CalibrationMin = 410 * CalibrationScale;
constexpr uint16_t CalibrationMax = 420 * CalibrationScale;
static const uint16_t defaultHalfBitUnits = uint16_t(HALF_BIT_TICKS * CalibrationScale + 0.5);
volatile uint16_t halfBitUnits = defaultHalfBitUnits;
volatile uint16_t requestedHalfBitUnits = defaultHalfBitUnits;
volatile bool timingChangePending = false;
uint8_t calibrationStep = 0;
uint8_t calibrationRecord[6];
volatile uint8_t tickRemainder = 0;

// Distribute fractional timer ticks without adjusting the configured period.
static inline uint16_t fixedTicks(uint8_t halves) {
    uint16_t numerator = halfBitUnits * halves + tickRemainder;
    tickRemainder = numerator % CalibrationScale;
    return numerator / CalibrationScale;
}

volatile uint8_t frameBufA[10];
volatile uint8_t frameBufB[10];
volatile uint8_t* volatile txBuf = frameBufA;
volatile uint8_t* volatile buildBuf = frameBufB;
volatile bool bufferSwapRequest = false;
volatile uint32_t framesSent = 0;
volatile uint32_t readyFrameNumber = 1;
bool frameStarted = false; // Accessed only during setup and in the ISR.
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
volatile bool timeSetPending = false;
volatile uint8_t requestedH, requestedM, requestedS, requestedF;
char timeInput[14];
uint8_t timeInputLength = 0;
bool timeInputInvalid = false;
uint32_t timeInputLastUs = 0;
NeoBoardTime::Parser neoTimeInput;
uint32_t neoTimeAccepted = 0, neoTimeRejected = 0; // Main-loop diagnostic counters.
constexpr uint8_t NeoReplyCapacity = 8;
constexpr uint8_t NeoReplyLength = 22;
bool neoReplies[NeoReplyCapacity];
uint8_t neoReplyHead = 0, neoReplyCount = 0;
constexpr uint32_t GpsTimeTimeoutMs = 3000;
bool gpsTimeReceived = false;
uint32_t lastGpsTimeMs = 0;

bool gpsTimeRecent(uint32_t now) {
    // Packet freshness, not satellite lock: NeoBoard sends only GPS week/ms.
    return gpsTimeReceived && uint32_t(now - lastGpsTimeMs) < GpsTimeTimeoutMs;
}

void queueNeoReply(bool accepted) {
    // serviceSerialInput reserves room before consuming a byte/expiring a packet.
    neoReplies[(neoReplyHead + neoReplyCount) % NeoReplyCapacity] = accepted;
    ++neoReplyCount;
}

void serviceNeoReply() {
    if (!neoReplyCount || Serial.availableForWrite() < NeoReplyLength) return;
    // CRC covers ASCII "1" or "0" only, init 0/no final XOR, as on NeoBoard.
    // Store the two fixed responses in flash; no runtime CRC/formatting needed.
    static const char ok[] PROGMEM = "STX,100,1,1,51de003a\r\n";
    static const char error[] PROGMEM = "STX,100,1,0,26d930ac\r\n";
    static_assert(sizeof(ok) == NeoReplyLength + 1 && sizeof(error) == sizeof(ok),
                  "ACK frame length must match the TX space check");
    const char* source = neoReplies[neoReplyHead] ? ok : error;
    uint8_t frame[NeoReplyLength];
    for (uint8_t i = 0; i < NeoReplyLength; ++i) frame[i] = pgm_read_byte(source + i);
    // Only loop() produces serial output. Reserved space cannot shrink in an ISR,
    // so this complete frame cannot hit HardwareSerial's buffer-full wait loop.
    // Enqueue it together to avoid interleaving ACK bytes with existing log lines.
    Serial.write(frame, sizeof(frame));
    neoReplyHead = (neoReplyHead + 1) % NeoReplyCapacity;
    --neoReplyCount;
}

// Convert elapsed wall time since midnight to the containing 29.97 DF frame.
// Integer arithmetic avoids AVR float precision loss at large times of day.
TC timecodeFromMilliseconds(uint32_t ms) {
    static_assert(FPS == 30 && DROP_FRAME == 1, "Serial clock input requires 29.97 DF");
    uint32_t frame = uint32_t(uint64_t(ms) * 30 / 1001) % 2589408UL;
    uint32_t tenMinuteBlocks = frame / 17982UL;
    uint16_t remainder = frame % 17982UL;
    uint32_t label = frame + tenMinuteBlocks * 18;
    if (remainder >= 2) label += 2 * ((remainder - 2) / 1798);
    TC tc;
    tc.F = label % 30;
    label /= 30;
    tc.S = label % 60;
    label /= 60;
    tc.M = label % 60;
    tc.H = (label / 60) % 24;
    return tc;
}

bool parseClockTime(const char* text, uint8_t length, TC& result) {
    if (length < 9 || length > 14 || text[0] != 'T' || text[1] != ',') return false;
    uint16_t fields[4] = {};
    uint8_t pos = 2;
    for (uint8_t field = 0; field < 4; ++field) {
        uint8_t digits = 0;
        while (pos < length && text[pos] >= '0' && text[pos] <= '9') {
            if (++digits > (field == 3 ? 3 : 2)) return false;
            fields[field] = fields[field] * 10 + text[pos++] - '0';
        }
        if (!digits) return false;
        if (field < 3 && (pos >= length || text[pos++] != ',')) return false;
    }
    if (pos != length) return false;
    uint8_t h = fields[0], m = fields[1], s = fields[2];
    uint16_t ms = fields[3];
    if (h > 23 || m > 59 || s > 59) return false;
    result = timecodeFromMilliseconds(((uint32_t(h) * 60 + m) * 60 + s) * 1000 + ms);
    return true;
}

bool isRecording = false;
bool pulseActive = false;
bool pendingToggle = false;
unsigned long pulseStartMs = 0;
unsigned long pulseEndMs = 0;

bool btnStable = false;
bool btnReading = false;
unsigned long btnChangeMs = 0;

struct CalibrationButton {
    bool reading = false;
    bool stable = false;
    uint32_t changedMs = 0;

    void begin(uint8_t pin, uint32_t now) {
        pinMode(pin, INPUT); // Board has external pull-downs, active HIGH.
        reading = stable = digitalRead(pin) == HIGH;
        changedMs = now;
    }
    bool pressed(uint8_t pin, uint32_t now) {
        bool raw = digitalRead(pin) == HIGH;
        if (raw != reading) { reading = raw; changedMs = now; }
        else if (stable != raw && uint32_t(now - changedMs) >= BTN_DEBOUNCE_MS) {
            stable = raw;
            return stable; // Rearm only after a debounced release; never repeat.
        }
        return false;
    }
};
CalibrationButton calUpButton, calDownButton;
// Preserve button events received while the preceding EEPROM write is in flight.
constexpr uint8_t CalibrationQueueSize = 16;
int8_t calibrationQueue[CalibrationQueueSize];
uint8_t calibrationQueueHead = 0, calibrationQueueCount = 0;

bool queueCalibration(int8_t direction) {
    if (calibrationQueueCount == CalibrationQueueSize) {
        Serial.println(F("ERR,CAL,QUEUE_FULL"));
        return false;
    }
    calibrationQueue[(calibrationQueueHead + calibrationQueueCount) % CalibrationQueueSize] = direction;
    ++calibrationQueueCount;
    return true;
}

void monitorCalibrationButtons() {
    uint32_t now = millis();
    if (calUpButton.pressed(CAL_UP_PIN, now)) queueCalibration(1);
    if (calDownButton.pressed(CAL_DOWN_PIN, now)) queueCalibration(-1);
}


unsigned long lastTCReportMs = 0;

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
    for (uint8_t i = 0; i < 10; ++i) buf[i] = 0;

    // Direct BCD packing keeps the ISR fallback short and bounded.
    buf[0] = tc.F % 10;
    buf[1] = tc.F / 10;
#if FPS != 60
    if (DROP_FRAME) buf[1] |= 0x04;
#endif
    buf[2] = tc.S % 10;
    buf[3] = tc.S / 10;
    buf[4] = tc.M % 10;
    buf[5] = tc.M / 10;
    buf[6] = tc.H % 10;
    buf[7] = tc.H / 10;
    buf[8] = 0xFC;
    buf[9] = 0xBF;

    uint8_t parity = 0;
    for (uint8_t i = 0; i < 10; ++i) parity ^= buf[i];
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    if (parity & 1) setBit(buf, FPS == 25 ? 59 : 27, 1);
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

    // Advance time only at the actual bit-0 boundary, never when scheduling bit 79.
    const bool frameStart = (txBitIdx == 0);
    const uint32_t frameStartUs = frameStart ? micros() : 0;
    if (frameStart) {
        if (timeSetPending) {
            if (frameStarted) ++framesSent;
            frameStarted = true;
            curH = requestedH;
            curM = requestedM;
            curS = requestedS;
            curF = requestedF;
            TC current = {curH, curM, curS, curF};
            buildFrame(txBuf, current);
            bufferSwapRequest = false;
            timeSetPending = false;
        } else if (frameStarted) {
            ++framesSent;
            advanceTC_isr();
            if (bufferSwapRequest && readyFrameNumber == framesSent) {
                volatile uint8_t* tmp = txBuf;
                txBuf = buildBuf;
                buildBuf = tmp;
            } else {
                TC current = {curH, curM, curS, curF};
                buildFrame(txBuf, current);
            }
            bufferSwapRequest = false;
        } else {
            frameStarted = true;
        }
        if (timingChangePending) {
            halfBitUnits = requestedHalfBitUnits;
            tickRemainder = 0;
            timingChangePending = false;
        }
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
    if (bufferSwapRequest) {
        return;
    }

    TC tc;
#if OLED_TIMECODE
    uint16_t displayUnits;
#endif
    volatile uint8_t* target;
    noInterrupts();
    curSent = framesSent;
    target = buildBuf;
    tc.H = curH;
    tc.M = curM;
    tc.S = curS;
    tc.F = curF;
#if OLED_TIMECODE
    displayUnits = halfBitUnits;
#endif
    interrupts();

    TC next = tc;
    advanceTC_local(next);

    buildFrame(target, next);
    // An ISR may have advanced time while this buffer was being built.
    noInterrupts();
    if (framesSent == curSent) {
        readyFrameNumber = curSent + 1;
        bufferSwapRequest = true;
    }
    interrupts();
#if OLED_TIMECODE
    // Reuse the existing coherent snapshot: no new ISR work or critical section.
    // Compare HH:MM:SS, since DF skips frame 00 at most minute boundaries.
    OledTimecode::request(tc.H, tc.M, tc.S);
    OledTimecode::requestStatus(displayUnits, isRecording);
#endif
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
#if OLED_TIMECODE
    OledTimecode::requestRecording(isRecording);
#endif

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

void printHalfBitUnits(uint16_t value) {
    Serial.print(value / CalibrationScale);
    Serial.print('.');
    uint8_t hundredths = (value % CalibrationScale) * 5;
    if (hundredths < 10) Serial.print('0');
    Serial.print(hundredths);
}

void printFrameTiming() {
    uint32_t last, samples;
    uint16_t units;
    noInterrupts();
    last = frameTiming.last;
    samples = frameTiming.samples;
    units = halfBitUnits;
    interrupts();
    Serial.print(F("     LAST,"));
    if (samples) Serial.print(last);
    else Serial.print(F("WAIT"));
    Serial.print(F(" ("));
    Serial.print(FrameTiming::minimumUs);
    Serial.print('_');
    Serial.print(FrameTiming::maximumUs);
    Serial.print(F(")     HALF_BIT_TICKS,"));
    printHalfBitUnits(units);
    Serial.println(F(""));
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

// EEPROM bytes 0..5: commit marker, version, value and its byte complements.
// Version 1 stored tenths; version 2 stores twentieths. Preserve existing tuning.
bool readCalibration(uint16_t& value) {
    uint8_t record[6];
    for (uint8_t i = 0; i < sizeof(record); ++i) record[i] = EEPROM.read(i);
    value = uint16_t(record[2]) | (uint16_t(record[3]) << 8);
    if (record[0] != 0xA5 || record[4] != uint8_t(~record[2]) ||
        record[5] != uint8_t(~record[3])) return false;
    if (record[1] == 1) {
        if (value < 4100 || value > 4200) return false;
        value *= 2; // Legacy value is converted in RAM; next adjustment saves v2.
        return true;
    }
    return record[1] == 2 && value >= CalibrationMin && value <= CalibrationMax;
}

void printCalibration(const __FlashStringHelper* status, uint16_t value) {
    Serial.print(status);
    Serial.print(F(",HALF_BIT_TICKS,"));
    printHalfBitUnits(value);
    Serial.println(F(""));
}

void startCalibrationChange(int8_t direction) {
    uint16_t value;
    noInterrupts();
    value = halfBitUnits;
    interrupts();
    if ((direction > 0 && value >= CalibrationMax) ||
        (direction < 0 && value <= CalibrationMin)) {
        printCalibration(F("ERR,LIMIT"), value);
        return;
    }
    value = direction > 0 ? value + 1 : value - 1;
    calibrationRecord[0] = 0xA5;
    calibrationRecord[1] = 2;
    calibrationRecord[2] = uint8_t(value);
    calibrationRecord[3] = uint8_t(value >> 8);
    calibrationRecord[4] = uint8_t(~calibrationRecord[2]);
    calibrationRecord[5] = uint8_t(~calibrationRecord[3]);
    noInterrupts();
    requestedHalfBitUnits = value;
    timingChangePending = true;
    interrupts();
    calibrationStep = 1; // Start a separate EEPROM save for every accepted step.
    printCalibration(F("RECEIVED"), value);
}

void queueTimecode(TC tc) {
    // Only publish the four label bytes atomically; parsing/conversion stays out
    // of this critical section and out of TIMER1_COMPA_vect.
    noInterrupts();
    requestedH = tc.H;
    requestedM = tc.M;
    requestedS = tc.S;
    requestedF = tc.F;
    timeSetPending = true;
    interrupts();
}

// A complete line is required: malformed or overlong lines never change time.
bool consumeTimeInput(char command) {
    if (command == '\r' || command == '\n') {
        if (timeInputLength || timeInputInvalid) {
            TC tc;
            if (!timeInputInvalid && parseClockTime(timeInput, timeInputLength, tc)) {
                queueTimecode(tc);
                Serial.print(F("SYNC,QUEUED,"));
                printTCLine(tc);
                Serial.println(F(""));
            } else Serial.println(F("ERR,TIME,EXPECTED_T,HH,MM,SS,mmm"));
        }
        timeInputLength = 0;
        timeInputInvalid = false;
        return true;
    }
    if (!timeInputLength && !timeInputInvalid && command != 'T') return false;
    timeInputLastUs = micros();
    if (timeInputLength < sizeof(timeInput)) timeInput[timeInputLength++] = command;
    else timeInputInvalid = true;
    return true;
}

void serviceSerialInput() {
    // Bounded backpressure if TX stalls: never wait or silently overwrite a reply.
    // LTC/OLED/REC continue through loop(); UART RX buffer capacity remains finite.
    if (neoReplyCount == NeoReplyCapacity) return;
    if ((timeInputLength || timeInputInvalid || neoTimeInput.active()) &&
        uint32_t(micros() - timeInputLastUs) > 500000UL) {
        timeInputLength = 0;
        timeInputInvalid = false;
        if (neoTimeInput.active()) {
            if (neoTimeInput.timeCommand()) queueNeoReply(false);
            neoTimeInput.reset();
            ++neoTimeRejected;
        } else Serial.println(F("ERR,TIME,TIMEOUT"));
    }
    if (neoReplyCount == NeoReplyCapacity || !Serial.available()) return;
    char command = Serial.read(); // At most one byte per loop, even during EEPROM writes.
    if (neoTimeInput.active() || (!timeInputLength && !timeInputInvalid && command == 'S')) {
        timeInputLastUs = micros();
        NeoBoardTime::Result result = neoTimeInput.feed(command);
        if (result == NeoBoardTime::Result::Accepted) {
            queueTimecode(timecodeFromMilliseconds(neoTimeInput.milliseconds % NeoBoardTime::DayMs));
            lastGpsTimeMs = millis();
            gpsTimeReceived = true;
            ++neoTimeAccepted;
            queueNeoReply(true);
        } else if (result == NeoBoardTime::Result::Rejected) {
            ++neoTimeRejected;
            if (neoTimeInput.timeCommand()) queueNeoReply(false);
        }
        // Actual TX is serviced separately, only when a whole reply fits.
        return;
    }
    if (consumeTimeInput(command)) return;
    if (command == 'U') command = 'u';
    if (command == 'D') command = 'd';
    if (command == ' ' || command == '\t') return;
    if (command != 'u' && command != 'd') {
        timeInputInvalid = true;
        timeInputLastUs = micros();
        Serial.print(F("ERR,INPUT_BYTE,"));
        Serial.println(uint8_t(command));
        return;
    }
    queueCalibration(command == 'u' ? 1 : -1);
}

void serviceCalibration() {
    if (calibrationStep) {
        if (timingChangePending || !eeprom_is_ready()) return;
        // One byte per loop; invalidate first and commit last to reject partial saves.
        if (calibrationStep == 1) EEPROM.update(0, 0);
        else if (calibrationStep <= 6) {
            uint8_t index = calibrationStep - 1;
            EEPROM.update(index, calibrationRecord[index]);
        } else if (calibrationStep == 7) EEPROM.update(0, 0xA5);
        else {
            uint16_t saved;
            uint16_t expected = uint16_t(calibrationRecord[2]) |
                                (uint16_t(calibrationRecord[3]) << 8);
            bool valid = readCalibration(saved) && saved == expected;
            printCalibration(valid ? F("SAVED") : F("ERR,SAVE"), expected);
            calibrationStep = 0;
            return;
        }
        ++calibrationStep;
        return;
    }

    if (calibrationQueueCount) {
        int8_t direction = calibrationQueue[calibrationQueueHead];
        calibrationQueueHead = (calibrationQueueHead + 1) % CalibrationQueueSize;
        --calibrationQueueCount;
        startCalibrationChange(direction);
        return;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    uint16_t saved;
    bool hasCalibration = readCalibration(saved);
    halfBitUnits = hasCalibration ? saved : defaultHalfBitUnits;
    requestedHalfBitUnits = halfBitUnits;
    calibrationStep = 0;
    timingChangePending = false;
    calibrationQueueHead = calibrationQueueCount = 0;
    printCalibration(hasCalibration ? F("LOADED") : F("DEFAULT"), halfBitUnits);

    pinMode(LTC_OUT_PIN, OUTPUT);
    LTC_LOW();
    pinMode(RELEASE_PIN, OUTPUT);
    RELEASE_LOW();

    pinMode(BUTTON_PIN, INPUT);
    btnReading = (digitalRead(BUTTON_PIN) == HIGH);
    btnStable = btnReading;
    btnChangeMs = millis();
    calUpButton.begin(CAL_UP_PIN, millis());
    calDownButton.begin(CAL_DOWN_PIN, millis());

#if OLED_TIMECODE
    // Initialization and display updates are serviced without TWI interrupts.
    OledTimecode::begin(millis());
    OledTimecode::requestStatus(halfBitUnits, isRecording);
#endif

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
    readyFrameNumber = 1;
    frameStarted = false;
    txBitIdx = 0;
    midBitEdgePending = false;
    framesSent = 0;
    timeSetPending = false;
    timeInputLength = 0;
    timeInputInvalid = false;
    neoTimeInput.reset();
    neoTimeAccepted = neoTimeRejected = 0;
    neoReplyHead = neoReplyCount = 0;
    gpsTimeReceived = false;
    lastGpsTimeMs = 0;

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
    Serial.println(F("CAL,INPUT,U/D,BTN1/D2_UP,BTN2/D3_DOWN,STEP,0.05,RANGE,410.00_420.00"));
    Serial.println(F("TIME,INPUT,T,HH,MM,SS,mmm,END,CR/LF"));
    Serial.println(F("TIME,NEO,STX,100,LEN,WEEK:MS,CRC32,GPS"));
    Serial.print(F("FPS,"));
    Serial.print(FPS);
    Serial.println(DROP_FRAME ? F("DF") : F("NDF"));
}

void loop() {
    prepareNextFrame();   // D9에서 다음에 송출할 타임코드 프레임 준비
    serviceSerialInput(); // One RX byte; CRC/conversion only in foreground, no EEPROM wait.
    serviceNeoReply();    // At most one complete ACK/NACK, only with enough TX space.
    servicePulse();       // D7 녹화 펄스 종료 및 대기 중인 요청 처리
    monitorButton();      // D4 버튼 입력을 확인하여 D7 녹화/정지 요청
    monitorCalibrationButtons(); // D2/D3: one calibration step per debounced press.
    serviceCalibration(); // U/D 및 버튼 주기 조정과 EEPROM 저장 처리
    periodicTCReport();   // 1초마다 타임코드와 프레임 측정값 출력
#if OLED_TIMECODE
    uint32_t displayNow = millis();
    OledTimecode::requestGpsTime(gpsTimeRecent(displayNow));
    OledTimecode::service(displayNow); // One non-blocking I2C step, after LTC preparation.
    uint8_t oledError = OledTimecode::takeError();
    if (oledError) {
        Serial.print(F("OLED,ERR,TWI,"));
        Serial.println(oledError);
    }
#endif
}
