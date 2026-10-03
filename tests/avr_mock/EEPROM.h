#pragma once
#include <array>
#include <stdint.h>
inline bool ready = true;
inline bool eeprom_is_ready() { return ready; }
struct EEPROMMock {
    std::array<uint8_t, 1024> bytes;
    unsigned writes = 0;
    EEPROMMock() { bytes.fill(0xFF); }
    uint8_t read(int address) { return bytes.at(address); }
    void update(int address, uint8_t value) {
        if (bytes.at(address) != value) {
            bytes.at(address) = value;
            ++writes;
        }
    }
};
inline EEPROMMock EEPROM;
