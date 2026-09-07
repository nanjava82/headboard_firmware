#include <Arduino.h>

/*
 * LTC Encoder + 4-Camera Controller for Arduino Nano (ATmega328P @ 16 MHz)
 *
 * Pins:
 *   D9  - LTC output, route through the analog output circuit before devices
 *   D6  - release output, HIGH = recording, drive camera inputs through buffers
 *   D2  - camera 1 LED input, HIGH = recording
 *   D3  - camera 2 LED input, HIGH = recording
 *   D4  - camera 3 LED input, HIGH = recording
 *   D5  - camera 4 LED input, HIGH = recording
 *   D13 - debug LED
 *
 * Serial commands at 115200:
 *   T,HH,MM,SS,FF
 *   R,START
 *   R,STOP
 *   S?
 *   D?
 */

#define FPS              30
#define DROP_FRAME       1
#define HALF_BIT_TICKS   417

#define NUM_CAMERAS      4
#define LED_DEBOUNCE_MS  50UL
#define REC_TIMEOUT_MS   3000UL

#define SERIAL_BAUD      115200
#define TC_REPORT_MS     1000UL

#define LTC_OUT_PIN      9
#define RELEASE_PIN      6
#define DEBUG_LED_PIN    13

const uint8_t LED_PINS[NUM_CAMERAS] = { 2, 3, 4, 5 };

#define LTC_TOGGLE()        (PINB = _BV(PB1))
#define LTC_LOW()           (PORTB &= ~_BV(PB1))
#define DEBUG_LED_TOGGLE()  (PINB = _BV(PB5))
#define RELEASE_HIGH()      (PORTD |= _BV(PD6))
#define RELEASE_LOW()       (PORTD &= ~_BV(PD6))

#define FRAME_BITS 80

volatile uint8_t frameBufA[10];
volatile uint8_t frameBufB[10];
volatile uint8_t* volatile txBuf = frameBufA;
volatile uint8_t* volatile buildBuf = frameBufB;
volatile bool bufferSwapRequest = false;
volatile uint32_t framesSent = 0;
volatile uint32_t underrunCount = 0;
volatile uint8_t txBitIdx = 0;
volatile bool halfBitToggle = false;

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

enum RecState {
    REC_IDLE,
    REC_REQ_START,
    REC_ACTIVE,
    REC_REQ_STOP
};

RecState recState = REC_IDLE;
unsigned long recRequestMs = 0;

bool ledStable[NUM_CAMERAS] = { false, false, false, false };
unsigned long ledChangeMs[NUM_CAMERAS] = { 0, 0, 0, 0 };
bool ledRaw[NUM_CAMERAS] = { false, false, false, false };
bool ledReported[NUM_CAMERAS] = { false, false, false, false };

char rxBuf[32];
uint8_t rxIdx = 0;

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

ISR(TIMER1_COMPA_vect) {
    if (!halfBitToggle) {
        LTC_TOGGLE();
        halfBitToggle = true;
    } else {
        if (getBit(txBuf, txBitIdx)) {
            LTC_TOGGLE();
        }
        halfBitToggle = false;

        txBitIdx++;
        if (txBitIdx >= FRAME_BITS) {
            txBitIdx = 0;
            framesSent++;
            DEBUG_LED_TOGGLE();

            if (bufferSwapRequest) {
                volatile uint8_t* tmp = txBuf;
                txBuf = buildBuf;
                buildBuf = tmp;
                bufferSwapRequest = false;
                advanceTC_isr();
            } else {
                underrunCount++;
            }
        }
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

void printRecLine(const __FlashStringHelper* prefix) {
    Serial.print(prefix);
    for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
        Serial.print(ledStable[i] ? '1' : '0');
        if (i < NUM_CAMERAS - 1) {
            Serial.print(',');
        }
    }
    Serial.println();
}

void monitorCameras() {
    unsigned long now = millis();
    bool anyChange = false;

    for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
        bool curRaw = (digitalRead(LED_PINS[i]) == HIGH);
        if (curRaw != ledRaw[i]) {
            ledRaw[i] = curRaw;
            ledChangeMs[i] = now;
        } else if (now - ledChangeMs[i] >= LED_DEBOUNCE_MS) {
            if (ledStable[i] != curRaw) {
                ledStable[i] = curRaw;
                anyChange = true;
            }
        }
    }

    if (!anyChange) {
        return;
    }

    bool reportChange = false;
    for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
        if (ledReported[i] != ledStable[i]) {
            reportChange = true;
            ledReported[i] = ledStable[i];
        }
    }
    if (reportChange) {
        printRecLine(F("EVT,REC,"));
    }
}

void handleRecording() {
    unsigned long now = millis();

    switch (recState) {
        case REC_IDLE:
        case REC_ACTIVE:
            break;

        case REC_REQ_START: {
            uint8_t onCount = 0;
            for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
                if (ledStable[i]) {
                    onCount++;
                }
            }

            if (onCount == NUM_CAMERAS) {
                recState = REC_ACTIVE;
                Serial.println(F("REC,1,1,1,1"));
            } else if (now - recRequestMs >= REC_TIMEOUT_MS) {
                printRecLine(F("ERR,REC_START_TIMEOUT,"));
                recState = REC_ACTIVE;
            }
            break;
        }

        case REC_REQ_STOP: {
            uint8_t offCount = 0;
            for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
                if (!ledStable[i]) {
                    offCount++;
                }
            }

            if (offCount == NUM_CAMERAS) {
                recState = REC_IDLE;
                Serial.println(F("REC,0,0,0,0"));
            } else if (now - recRequestMs >= REC_TIMEOUT_MS) {
                printRecLine(F("ERR,REC_STOP_TIMEOUT,"));
                recState = REC_IDLE;
            }
            break;
        }
    }
}

void setTimeSafe(uint8_t h, uint8_t m, uint8_t s, uint8_t f) {
    TC newTC = { h, m, s, f };
    TC nextTC = newTC;
    advanceTC_local(nextTC);

    static volatile uint8_t tempBufA[10];
    static volatile uint8_t tempBufB[10];
    buildFrame(tempBufA, newTC);
    buildFrame(tempBufB, nextTC);

    noInterrupts();
    for (uint8_t i = 0; i < 10; i++) {
        txBuf[i] = tempBufA[i];
        buildBuf[i] = tempBufB[i];
    }
    curH = newTC.H;
    curM = newTC.M;
    curS = newTC.S;
    curF = newTC.F;
    txBitIdx = 0;
    halfBitToggle = false;
    bufferSwapRequest = true;
    lastBuiltFrame = framesSent;
    interrupts();
}

void printTCLine(const __FlashStringHelper* prefix, TC tc) {
    Serial.print(prefix);
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
    Serial.println(tc.F);
}

void processSerialCommand(const char* cmd) {
    if (cmd[0] == 'T' && cmd[1] == ',') {
        int h, m, s, f;
        if (sscanf(cmd + 2, "%d,%d,%d,%d", &h, &m, &s, &f) == 4) {
            if (h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60 && f >= 0 && f < FPS) {
                setTimeSafe((uint8_t)h, (uint8_t)m, (uint8_t)s, (uint8_t)f);
                Serial.print(F("OK,T,"));
                Serial.println(cmd + 2);
                return;
            }
        }
        Serial.println(F("ERR,PARSE_T"));
        return;
    }

    if (cmd[0] == 'R' && cmd[1] == ',') {
        if (strcmp(cmd + 2, "START") == 0) {
            RELEASE_HIGH();
            recRequestMs = millis();
            recState = REC_REQ_START;
            Serial.println(F("OK,R,START"));
            return;
        }
        if (strcmp(cmd + 2, "STOP") == 0) {
            RELEASE_LOW();
            recRequestMs = millis();
            recState = REC_REQ_STOP;
            Serial.println(F("OK,R,STOP"));
            return;
        }
        Serial.println(F("ERR,PARSE_R"));
        return;
    }

    if (cmd[0] == 'S' && cmd[1] == '?') {
        TC tc;
        noInterrupts();
        tc.H = curH;
        tc.M = curM;
        tc.S = curS;
        tc.F = curF;
        interrupts();

        printTCLine(F("TC,"), tc);
        printRecLine(F("REC,"));
        return;
    }

    if (cmd[0] == 'D' && cmd[1] == '?') {
        uint32_t ur;
        uint32_t fs;
        noInterrupts();
        ur = underrunCount;
        fs = framesSent;
        interrupts();

        Serial.print(F("STAT,UNDERRUN,"));
        Serial.println(ur);
        Serial.print(F("STAT,FRAMES,"));
        Serial.println(fs);
        return;
    }

    Serial.println(F("ERR,UNKNOWN_CMD"));
}

void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxIdx > 0) {
                rxBuf[rxIdx] = 0;
                processSerialCommand(rxBuf);
                rxIdx = 0;
            }
        } else if (rxIdx < sizeof(rxBuf) - 1) {
            rxBuf[rxIdx++] = c;
        } else {
            rxIdx = 0;
            Serial.println(F("ERR,BUF_OVERFLOW"));
        }
    }
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

    printTCLine(F("TC,"), tc);
#endif
}

void setup() {
    Serial.begin(SERIAL_BAUD);

    pinMode(LTC_OUT_PIN, OUTPUT);
    LTC_LOW();
    pinMode(RELEASE_PIN, OUTPUT);
    RELEASE_LOW();
    pinMode(DEBUG_LED_PIN, OUTPUT);

    for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
        pinMode(LED_PINS[i], INPUT);
    }

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
    halfBitToggle = false;
    framesSent = 0;
    underrunCount = 0;

    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = HALF_BIT_TICKS - 1;
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
    prepareNextFrame();
    processSerial();
    monitorCameras();
    handleRecording();
    periodicTCReport();
}
