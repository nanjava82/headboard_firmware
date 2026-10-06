#include <cassert>
#include "../src/main.cpp"

void poll(uint32_t now) {
    simulatedMillis = now;
    monitorCalibrationButtons();
}
void press(uint8_t pin, uint32_t now) {
    simulatedPins[pin] = HIGH;
    poll(now); poll(now + BTN_DEBOUNCE_MS);
}
void release(uint8_t pin, uint32_t now) {
    simulatedPins[pin] = 0;
    poll(now); poll(now + BTN_DEBOUNCE_MS);
}
void drain() {
    for (unsigned i = 0; i < 5000 && (calibrationQueueCount || calibrationStep); ++i) {
        TIMER1_COMPA_vect();
        prepareNextFrame();
        serviceCalibration();
    }
    assert(!calibrationQueueCount && !calibrationStep);
}
size_t saves() {
    size_t count = 0, pos = 0;
    while ((pos = Serial.output.find("SAVED,", pos)) != std::string::npos) { ++count; ++pos; }
    return count;
}

int main() {
    setup();
    // Contact bounce never produces a step.
    simulatedPins[CAL_UP_PIN] = HIGH; poll(1);
    simulatedPins[CAL_UP_PIN] = 0; poll(10);
    simulatedPins[CAL_UP_PIN] = HIGH; poll(20); poll(69);
    assert(calibrationQueueCount == 0);
    poll(70);
    assert(calibrationQueueCount == 1);
    drain();
    assert(halfBitUnits == 8321 && saves() == 1);
    for (uint32_t t = 100; t < 5000; t += 10) poll(t);
    drain();
    assert(halfBitUnits == 8321 && saves() == 1); // Held for five seconds: one step.

    // A bounced release does not rearm the button.
    simulatedPins[CAL_UP_PIN] = 0; poll(5000);
    simulatedPins[CAL_UP_PIN] = HIGH; poll(5010); poll(5100);
    assert(calibrationQueueCount == 0);
    release(CAL_UP_PIN, 5200); press(CAL_UP_PIN, 5300);
    drain();
    assert(halfBitUnits == 8322 && saves() == 2);

    // Both directions remain ordered and individually saved while EEPROM is busy.
    release(CAL_UP_PIN, 5400);
    press(CAL_DOWN_PIN, 5500);
    serviceCalibration(); // Start 416.05 save.
    ready = false;
    press(CAL_UP_PIN, 5600);
    release(CAL_DOWN_PIN, 5700); press(CAL_DOWN_PIN, 5800);
    assert(calibrationQueueCount == 2 && calibrationStep);
    ready = true;
    drain();
    assert(halfBitUnits == 8321 && saves() == 5);
    assert(Serial.output.find("SAVED,HALF_BIT_TICKS,416.05\nRECEIVED,HALF_BIT_TICKS,416.10\nSAVED,HALF_BIT_TICKS,416.10") != std::string::npos);
    uint16_t saved;
    assert(readCalibration(saved) && saved == 8321);

    // Held at boot must be released before a new rising edge is accepted.
    setup();
    poll(10000);
    assert(calibrationQueueCount == 0);
    release(CAL_DOWN_PIN, 10100); press(CAL_DOWN_PIN, 10200);
    drain();
    assert(halfBitUnits == 8320);

    // Debounce remains correct when the millisecond counter wraps.
    release(CAL_UP_PIN, UINT32_MAX - 100);
    simulatedPins[CAL_UP_PIN] = HIGH; poll(UINT32_MAX - 20); poll(28);
    assert(calibrationQueueCount == 0);
    poll(29); drain();
    assert(halfBitUnits == 8321);
    assert(!isRecording && !pulseActive); // Calibration buttons never trigger D7.
    puts("PASS: BTN1/BTN2 edge-only, bounce/hold/release/boot/wrap, queued per-step EEPROM saves, no REC pulse");
}
