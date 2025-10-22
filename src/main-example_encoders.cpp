#include <Arduino.h>
#include <Encoder.h>
#include <CrcLib.h>

Encoder enc(CRC_ENCO_A, CRC_ENCO_B);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("Basic Encoder Test:");
}

void loop() {
    Serial.println(enc.read());
}
