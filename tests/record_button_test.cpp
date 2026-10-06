#include <cassert>
#define OLED_TIMECODE 1
#include "../src/main.cpp"

// Drive the real loop with an acknowledged OLED bus and ongoing LTC edges.
void runUntil(uint32_t end) {
    while (simulatedMillis < end) {
        ++simulatedMillis;
        simulatedMicros = simulatedMillis * 1000;
        for (uint8_t i = 0; i < 8; ++i) {
            using OledTimecode::State;
            switch (OledTimecode::state) {
                case State::Start: TWCR.complete(TW_START); break;
                case State::AddressAck: TWCR.complete(TW_MT_SLA_ACK); break;
                case State::DataAck: TWCR.complete(TW_MT_DATA_ACK); break;
                case State::Stop: TWCR.stopComplete(); break;
                default: break;
            }
            TIMER1_COMPA_vect();
            loop();
        }
    }
}

int main() {
    setup();
    runUntil(100);
    assert(!(PORTD & _BV(PD7)) && !isRecording);

    // A short bounce must not trigger recording.
    simulatedPins[BUTTON_PIN] = HIGH;
    runUntil(110);
    simulatedPins[BUTTON_PIN] = 0;
    runUntil(170);
    assert(!(PORTD & _BV(PD7)) && !isRecording);

    // A press reaches D7 after 50 ms, even during a calibration EEPROM save.
    startCalibrationChange(1);
    ready = false;
    simulatedPins[BUTTON_PIN] = HIGH;
    runUntil(220);
    assert(!pulseActive);
    runUntil(221);
    assert(pulseActive && (PORTD & _BV(PD7)) && isRecording);
    assert(Serial.output.find("REC,1\n") != std::string::npos);
    assert(OledTimecode::wantedStatus.recording);
    runUntil(1020);
    assert(PORTD & _BV(PD7));
    runUntil(1021);
    assert(!pulseActive && !(PORTD & _BV(PD7)) && isRecording);
    ready = true;
    runUntil(2200);
    assert(!calibrationStep && halfBitUnits == 8321);
    assert(!pulseActive && isRecording); // Holding does not repeat.

    // Release and press again sends the STOP pulse.
    simulatedPins[BUTTON_PIN] = 0;
    runUntil(2260);
    simulatedPins[BUTTON_PIN] = HIGH;
    runUntil(2311);
    assert(pulseActive && (PORTD & _BV(PD7)) && !isRecording);
    assert(Serial.output.find("REC,0\n") != std::string::npos);
    assert(!OledTimecode::wantedStatus.recording);

    // A new press during a pulse is deferred until the 1000 ms release gap.
    simulatedPins[BUTTON_PIN] = 0;
    runUntil(2371);
    simulatedPins[BUTTON_PIN] = HIGH;
    runUntil(2431);
    assert(pendingToggle && !isRecording);
    runUntil(3111);
    assert(!(PORTD & _BV(PD7)) && pendingToggle);
    runUntil(4110);
    assert(!(PORTD & _BV(PD7)) && !isRecording);
    runUntil(4111);
    assert((PORTD & _BV(PD7)) && isRecording && !pendingToggle);
    assert(!TWCR.interruptEnabled);
    puts("PASS: D4 debounce -> D7 800 ms pulse, hold/release/queued press, OLED and busy EEPROM coexistence");
}
