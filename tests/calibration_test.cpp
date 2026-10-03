#include <cassert>
#include "../src/main.cpp"

void finishSave() {
    for (unsigned i = 0; calibrationStep && i < 1000; ++i) {
        TIMER1_COMPA_vect();
        prepareNextFrame();
        serviceCalibration();
    }
    assert(calibrationStep == 0);
}

int main() {
    setup();
    assert(halfBitTenths == 4160 && EEPROM.writes == 0);
    Serial.input = "u\r\nd";
    serviceCalibration();
    assert(halfBitTenths == 4160 && requestedHalfBitTenths == 4161);
    txBitIdx = 7;
    TIMER1_COMPA_vect();
    assert(halfBitTenths == 4160);
    finishSave();
    uint16_t saved;
    assert(readCalibration(saved) && saved == 4161);
    serviceCalibration(); // CR
    serviceCalibration(); // LF
    serviceCalibration(); // d
    finishSave();
    assert(readCalibration(saved) && saved == 4160);
    halfBitTenths = 4199;
    Serial.input = "u";
    serviceCalibration();
    finishSave();
    assert(halfBitTenths == 4200);
    unsigned writes = EEPROM.writes;
    Serial.input = "u";
    serviceCalibration();
    assert(halfBitTenths == 4200 && EEPROM.writes == writes);
    halfBitTenths = 4100;
    Serial.input = "d";
    serviceCalibration();
    assert(halfBitTenths == 4100 && EEPROM.writes == writes);
    setup();
    assert(halfBitTenths == 4200); // Reboot restores EEPROM, not the RAM value.
    for (uint16_t invalid : {4099, 4201, 5000}) {
        EEPROM.bytes[2] = uint8_t(invalid);
        EEPROM.bytes[3] = uint8_t(invalid >> 8);
        EEPROM.bytes[4] = uint8_t(~EEPROM.bytes[2]);
        EEPROM.bytes[5] = uint8_t(~EEPROM.bytes[3]);
        setup();
        assert(halfBitTenths == 4160);
    }
    EEPROM.bytes[4] ^= 1;
    setup();
    assert(halfBitTenths == 4160);
    EEPROM.bytes.fill(0xFF);
    Serial.input = "d";
    serviceCalibration();
    txBitIdx = 0;
    midBitEdgePending = false;
    TIMER1_COMPA_vect();
    ready = false;
    serviceCalibration();
    assert(calibrationStep == 1);
    ready = true;
    while (calibrationStep < 8) {
        serviceCalibration();
        if (calibrationStep < 8) assert(!readCalibration(saved));
    }
    serviceCalibration();
    assert(readCalibration(saved) && saved == 4159);
    halfBitTenths = 4155;
    tickRemainder = 0;
    unsigned total = 0;
    for (unsigned i = 0; i < 160; ++i) total += fixedTicks(1);
    assert(total == 66480);
    assert(recToggle() && pulseActive && (PORTD & _BV(PD7)));
    assert(!recToggle());
    pulseStartMs = 0UL - REC_PULSE_MS;
    servicePulse();
    assert(!pulseActive && !(PORTD & _BV(PD7)));
    puts("PASS: defaults, u/d, frame boundary, persistence, corruption, limits, busy EEPROM, partial save, fractional ticks, REC");
}
