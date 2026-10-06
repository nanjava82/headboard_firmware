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
    assert(halfBitUnits == 8320 && EEPROM.writes == 0);
    Serial.input = "u\r\nd";
    serviceCalibration();
    assert(halfBitUnits == 8320 && requestedHalfBitUnits == 8321);
    txBitIdx = 7;
    TIMER1_COMPA_vect();
    assert(halfBitUnits == 8320);
    finishSave();
    uint16_t saved;
    assert(readCalibration(saved) && saved == 8321);
    assert(EEPROM.bytes[1] == 2);
    assert(Serial.output.find("SAVED,HALF_BIT_TICKS,416.05") != std::string::npos);
    serviceCalibration(); // CR
    serviceCalibration(); // LF
    serviceCalibration(); // d
    finishSave();
    assert(readCalibration(saved) && saved == 8320);
    halfBitUnits = 8399;
    Serial.input = "u";
    serviceCalibration();
    finishSave();
    assert(halfBitUnits == 8400);
    unsigned writes = EEPROM.writes;
    Serial.input = "u";
    serviceCalibration();
    assert(halfBitUnits == 8400 && EEPROM.writes == writes);
    halfBitUnits = 8200;
    Serial.input = "d";
    serviceCalibration();
    assert(halfBitUnits == 8200 && EEPROM.writes == writes);
    setup();
    assert(halfBitUnits == 8400); // Reboot restores EEPROM, not the RAM value.
    for (uint16_t invalid : {8199, 8401, 10000}) {
        EEPROM.bytes[2] = uint8_t(invalid);
        EEPROM.bytes[3] = uint8_t(invalid >> 8);
        EEPROM.bytes[4] = uint8_t(~EEPROM.bytes[2]);
        EEPROM.bytes[5] = uint8_t(~EEPROM.bytes[3]);
        setup();
        assert(halfBitUnits == 8320);
    }
    EEPROM.bytes[4] ^= 1;
    setup();
    assert(halfBitUnits == 8320);
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
    assert(readCalibration(saved) && saved == 8319);
    halfBitUnits = 8310;
    tickRemainder = 0;
    unsigned total = 0;
    for (unsigned i = 0; i < 160; ++i) total += fixedTicks(1);
    assert(total == 66480);
    // Exact fractional accumulation for every 0.05 setting and mixed BMC intervals.
    for (uint16_t units = CalibrationMin; units <= CalibrationMax; ++units) {
        halfBitUnits = units;
        tickRemainder = 0;
        uint32_t sum = 0;
        for (unsigned halves = 0; halves < 160;) {
            uint8_t step = halves % 3 == 0 && halves < 159 ? 2 : 1;
            sum += fixedTicks(step);
            halves += step;
            assert(uint32_t(units) * halves - sum * CalibrationScale == tickRemainder);
            assert(tickRemainder < CalibrationScale);
        }
        assert(sum == uint32_t(units) * 8 && tickRemainder == 0);
    }
    // Version-1 EEPROM retains the same physical period and upgrades on next change.
    EEPROM.bytes[0] = 0xA5;
    EEPROM.bytes[1] = 1;
    EEPROM.bytes[2] = uint8_t(4155);
    EEPROM.bytes[3] = uint8_t(4155 >> 8);
    EEPROM.bytes[4] = uint8_t(~EEPROM.bytes[2]);
    EEPROM.bytes[5] = uint8_t(~EEPROM.bytes[3]);
    setup();
    assert(halfBitUnits == 8310);
    Serial.input = "u";
    serviceCalibration(); finishSave();
    assert(readCalibration(saved) && saved == 8311 && EEPROM.bytes[1] == 2);
    setup();
    assert(halfBitUnits == 8311); // 415.55 survives a reboot.
    assert(recToggle() && pulseActive && (PORTD & _BV(PD7)));
    assert(!recToggle());
    pulseStartMs = 0UL - REC_PULSE_MS;
    servicePulse();
    assert(!pulseActive && !(PORTD & _BV(PD7)));
    puts("PASS: 0.05 steps, decimal output, frame boundary, persistence/v1 migration, corruption, limits, busy/partial EEPROM, all fractional settings, REC");
}
