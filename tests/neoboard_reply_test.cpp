#include <cassert>
#include <string>
#define OLED_TIMECODE 1
#include "../src/main.cpp"

const std::string ok = "STX,100,1,1,51de003a\r\n";
const std::string error = "STX,100,1,0,26d930ac\r\n";
const char* valid = "STX,100,11,2400:102000,5f200647\r\n";
const char* badCrc = "STX,100,11,2400:102000,00000000\r\n";

void receive(const std::string& bytes) {
    Serial.input += bytes;
    unsigned limit = Serial.input.size() + 1;
    while (Serial.available() && limit--) serviceSerialInput();
    assert(Serial.input.empty());
}

void sendOne(const std::string& expected) {
    auto before = Serial.output;
    Serial.txSpace = NeoReplyLength;
    serviceNeoReply();
    assert(Serial.output == before + expected);
    assert(!Serial.blockedWrites && Serial.txSpace == 0);
}

int main() {
    setup();
    Serial.output.clear();
    Serial.drainWrites = false;
    assert(ok.size() == NeoReplyLength && error.size() == NeoReplyLength);

    receive("STX,100,11,2400:102000,5f200647");
    assert(!neoReplyCount && !timeSetPending); // No ACK before complete frame.
    receive("\r\n");
    assert(neoReplyCount == 1 && timeSetPending && Serial.output.empty());
    uint16_t period = halfBitUnits;
    auto timerA = TCCR1A, timerB = TCCR1B, mask = TIMSK1;
    for (int space : {0, 1, 21}) {
        Serial.txSpace = space;
        serviceNeoReply();
        assert(neoReplyCount == 1 && Serial.output.empty() && !Serial.writeCalls);
    }
    // LTC applies GPS time and continues even while the reply cannot be sent.
    TIMER1_COMPA_vect();
    TC expected = timecodeFromMilliseconds(102000);
    assert(!timeSetPending && neoReplyCount == 1 && curH == expected.H &&
           curM == expected.M && curS == expected.S && curF == expected.F);
    sendOne(ok);
    assert(!neoReplyCount);
    auto writes = Serial.writeCalls;
    serviceNeoReply();
    assert(Serial.writeCalls == writes); // Exactly one ACK for CRLF.

    auto lastGoodTime = lastGpsTimeMs;
    simulatedMillis = 1000;
    for (const char* bad : {badCrc,
         "STX,100,10,2400:102000,5f200647\n",
         "STX,100,14,2400:604800000,0\n",
         "STX,100,11,2400:102000,nothex\n"}) {
        receive(bad);
        assert(neoReplyCount == 1 && !timeSetPending && lastGpsTimeMs == lastGoodTime);
        sendOne(error);
    }
    receive("STX,101,11,2400:102000,5f200647\n");
    assert(!neoReplyCount); // Other command IDs/noise do not trigger command 100 replies.

    receive("STX,100,");
    simulatedMicros += 500001;
    serviceSerialInput();
    assert(neoReplyCount == 1 && !neoTimeInput.active());
    sendOne(error);
    serviceSerialInput(); receive("\r\n");
    assert(!neoReplyCount); // Timeout produces one NACK, not a repeated alarm.
    receive("STX,10");
    simulatedMicros += 500001;
    serviceSerialInput();
    assert(!neoReplyCount); // Command was not identifiable.

    // Preserve reply ordering and send whole frames around other log lines.
    Serial.output.clear();
    receive(valid); receive(badCrc); receive(valid);
    assert(neoReplyCount == 3);
    sendOne(ok);
    Serial.println("LOG");
    sendOne(error); sendOne(ok);
    assert(Serial.output == ok + "LOG\n" + error + ok);

    // Full response queue applies backpressure without overwriting old responses.
    const std::string validLf = std::string(valid).substr(0, strlen(valid) - 2) + "\n";
    const std::string badLf = std::string(badCrc).substr(0, strlen(badCrc) - 2) + "\n";
    for (unsigned i = 0; i < NeoReplyCapacity; ++i) receive(i % 2 ? badLf : validLf);
    Serial.input = validLf;
    auto accepted = neoTimeAccepted;
    serviceSerialInput();
    assert(Serial.input == validLf && neoReplyCount == NeoReplyCapacity && neoTimeAccepted == accepted);
    sendOne(ok);
    receive("");
    assert(neoReplyCount == NeoReplyCapacity && neoTimeAccepted == accepted + 1);
    for (unsigned i = 1; i < NeoReplyCapacity; ++i) sendOne(i % 2 ? error : ok);
    sendOne(ok);
    assert(!neoReplyCount && !Serial.blockedWrites);
    assert(halfBitUnits == period && TCCR1A == timerA && TCCR1B == timerB && TIMSK1 == mask);

    receive(valid);
    setup();
    assert(!neoReplyCount); // No stale response after reboot.
    puts("PASS: ACK/NACK exact CRC/CRLF, complete frames, timeout/unknown command, TX-space gating, ordered whole-frame replies, queue backpressure, LTC continues");
}
