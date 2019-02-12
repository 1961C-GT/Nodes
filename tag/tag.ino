/**
 *
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsTag
 *  - give example description
 */
#include <SPI.h>
#include "AsymmetricRanging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

void setup() {
  Serial.begin(115200);
  delay(1000);
  //init the configuration
  AsymmetricRanging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  AsymmetricRanging.attachNewRange(newRange);
  AsymmetricRanging.attachNewDevice(newDevice);
  AsymmetricRanging.attachInactiveDevice(inactiveDevice);
  AsymmetricRanging.attachUnknownMessage(unknownMessage);
  //Enable the filter to smooth the distance
  //AsymmetricRanging.useRangeFilter(true);

  //we start the module as a tag
  AsymmetricRanging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  AsymmetricRanging.loop();
}

void newRange() {
  Serial.print(F("from: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(F("\t Range: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print(F("\t RX power: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
}

void unknownMessage(byte* data[], uint8_t len) {
  // String dataStr = "";
  // // append to string
  // for(int i = 0; i < len; i++) {
  //   dataStr += (char)data[i];
  // }
  String myString = String((char *)data);

  Serial.print(F("Unknown Message: ")); Serial.println(myString);
}

void newDevice(DW1000Device* device) {
  Serial.print(F("ranging init; 1 device added ! -> "));
  Serial.print(F(" short:"));
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print(F("delete inactive device: "));
  Serial.println(device->getShortAddress(), HEX);
}
