/**
 *
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsAnchor
 *  - give example description
 */
#include <SPI.h>
#include "AsymmetricRanging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

volatile unsigned long delaySent = 0;
int16_t sentNum = 0; // todo check int type

void setup() {
  Serial.begin(115200);
  delay(1000);
  //init the configuration
  AsymmetricRanging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  AsymmetricRanging.attachNewRange(newRange);
  AsymmetricRanging.attachBlinkDevice(newBlink);
  AsymmetricRanging.attachInactiveDevice(inactiveDevice);
  AsymmetricRanging.attachSafeTransmit(transmitter);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);

  //we start the module as an anchor
  AsymmetricRanging.startAsAnchor("82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  AsymmetricRanging.loop();
}

void newRange() {
  Serial.print(F("from: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(F("\t Range: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print(F("\t RX power: ")); Serial.print(AsymmetricRanging.getDistantDevice()->getRXPower()); Serial.println(" dBm - ");

  // transmitter();
}


void transmitter(DW1000Device* device) {
  // transmit some data
  Serial.print(F("TRX #")); Serial.println(sentNum);
  DW1000.newTransmit();
  DW1000.setDefaults();
  String msg = F("Hello DW1000, it's #"); msg += sentNum;
  DW1000.setData(msg);
  // delay sending the message for the given amount
  DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.startTransmit();
  delaySent = millis();

  sentNum++;
}

void newBlink(DW1000Device* device) {
  Serial.print(F("blink; 1 device added ! -> "));
  Serial.print(F(" short:"));
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print(F("delete inactive device: "));
  Serial.println(device->getShortAddress(), HEX);
}
