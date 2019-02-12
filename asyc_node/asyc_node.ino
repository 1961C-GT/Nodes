// IMPORTS
#include <SPI.h>
#include <DW1000.h>
#include <M0Timer.h>
#include "constants.h"
// END IMPORTS

// ========= Node IDs ========== //
#define LEN_NODES_LIST 3
constexpr uint16_t nodeList[] = {
  0x71E9, // RED - Base Station
  0x726C, // BLUE
  0xFDA0  // GREEN
};
uint8_t nodeNumber;
boolean isBase;

const uint16_t replyDelayTimeUS = 4000;

State state;

// HELPER FUNCTIONS:
// RX and TX Callback Functions
uint32_t tmpTime;
uint32_t rxTime;
uint32_t txTime;
volatile boolean txFlag = false;
volatile boolean rxFlag = false;
void txHandle() { txTime = micros(); txFlag = true; /*Serial.println('t');*/ }
void rxHandle() { rxTime = micros(); rxFlag = true; /*Serial.println('r');*/ }
boolean checkTx() { if (txFlag) { txFlag = false; return true; } return false; }
boolean checkRx() { if (rxFlag) { rxFlag = false; return true; } return false; }

void (* _TC3Callback)(TcCount16* tc);

// Testing
volatile boolean reset;
volatile boolean led;

DWMODE dwMode;
DW1000Time msgDelay;

void setup() {
  pinMode(LED_PIN, OUTPUT);


  // Start the serial interface
  Serial.begin(115200);
  Serial.println(F("### ASYC Node ###"));

  // #ifdef DEBUG
    while(!Serial);
  // #endif

  // INIT DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println("DW1000 initialized ...");

  DW1000.newConfiguration();
  DW1000.setDefaults();

  // Set our addresses and note what our personal address is
  uint16_t ownAddress = setAddresses(0xDECA); // Want to change to 1961 but the mac address stuff is hardcoded

  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  Serial.println(F("Committed configuration ..."));

  // Figure out what node number we are
  // First see if we are the first address in the node list. If we are, then we
  // are also the base station
  if (nodeList[0] == ownAddress) {
    isBase = true;
    nodeNumber = 0;
  }
  // If we are not the first in the list, then search the nodeList for our
  // address
  else {
    nodeNumber = 255;
    for (uint8_t i = 1; i < LEN_NODES_LIST; i++) {

      // If we have found our adderss, set nodeNumber to i and leave the loop
      if (nodeList[i] == ownAddress) {
        nodeNumber = i;
        break;
      }
    }

    // IF we were unable to identify ourselves in the node list, then kill the
    // program
    if (nodeNumber == 255) {
      while(!Serial);
      Serial.print("Error: Node ID Not in List: "); Serial.println(ownAddress, HEX);
      for (uint8_t i = 0; i < LEN_NODES_LIST; i++) {
        Serial.print(" -- "); Serial.println(nodeList[i], HEX);
      }
      // Kill The Program
      for (;;)
        delay(1000);

    }
  }

  Serial.print("Own Address: "); Serial.print(ownAddress, HEX);
  Serial.print(" -- Node #: "); Serial.print(nodeNumber);
  if (isBase)
    Serial.println(" -- IS BASE");
  else
    Serial.println();

  // Attach DW1000 Handlers
  DW1000.attachSentHandler(txHandle);
  DW1000.attachReceivedHandler(rxHandle);
  DW1000.attachReceiveFailedHandler(rxFail);
  DW1000.attachReceiveTimeoutHandler(rxTimeout);
  DW1000.attachErrorHandler(err);

  // #ifdef DEBUG
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // #endif


  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  Serial.print("Core: "); Serial.println(SystemCoreClock);


  if (isBase) {
    state = { rb_range };
  } else {
    state = { ra_init };
  }


  dwMode = IDLE;

  M0Timer.setTC3SingleUse();

  M0Timer.startTimer(100,3);
  M0Timer.stopTimer(3);

  msgDelay = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);

  delay(10);

  reset = false;
  led = false;

  // M0Timer.attachTC3Handler(tim);
  // M0Timer.startTimer(500.1, 3);

}


// Set the EUI based on this SAMD21G18's unique ID. Also set the
// device address based on the unique ID and set the network ID as given
uint16_t setAddresses(uint16_t net) {
  byte val1, val2, val3, val4, val5, val6, val7, val8;
  byte *ptr = (byte *)0x0080A00C;
  val1 = *ptr; ptr++;
  val2 = *ptr; ptr++;
  val3 = *ptr; ptr++;
  val4 = *ptr; ptr++;

  ptr = (byte *)0x0080A040;
  val5 = *ptr; ptr++;
  val6 = *ptr; ptr++;
  val7 = *ptr; ptr++;
  val8 = *ptr; ptr++;

  byte addr[] = {val4, val3, val2, val1, val8, val7, val6, val5};

  DW1000.setEUI(addr);
  DW1000.setDeviceAddress(val4*256+val3);
  DW1000.setNetworkId(net);

  // // if (val4 == 0x72 && val3 == 0x6C) {
  // if (val4 == 0x71 && val3 == 0xE9) {
  //   Serial.println("Base Station!");
  //   return true;
  // }
  // return false;

  return val4*256+val3; //(uint16_t) val3 | ((uint16_t) val4 << 8);
}

void receiver() {
  // if (dwMode != RX) {
  // 	DW1000.newReceive();
  // 	DW1000.setDefaults();
  // 	// so we don't need to restart the receiver manually
  // 	DW1000.receivePermanently(true);
  // 	DW1000.startReceive();
  //   dwMode = RX;
  //   Serial.println("Entered RX Mode");
  // }
}

// void transmitter() {
//   // if (dwMode != TX) {
//     DW1000.newTransmit();
//     DW1000.setDefaults();
//     dwMode = TX;
//     Serial.println("Entered TX Mode");
//   // }
// }

void rxFail() {
  Serial.println("RX Failure");
}
void rxTimeout() {
  Serial.println("RX Timeout");
}
void err() {
  Serial.println("Sys Fail");
}


Message getMessage() {

  // Create Message Struct
  Message msg;

  // Main Data Buffer
  byte data[LEN_MAC + LEN_DATA];

  // Populate the main data buffer
  DW1000.getData(data, LEN_MAC + LEN_DATA);

  // Get the length of the data coming from the DW1000
  uint8_t dataLength = DW1000.getDataLength();

  // Check if the message is the correct min length. Also confirm that the
  // Check Byte (Byte 0) is 0x9B (unique to our project)
  if (dataLength >= 3 && data[0] == 0x9B) {

    // Get our From bits
    msg.from = data[1] >> 4;

    // Get our Type bits
    msg.type = (Msg_Type) (data[2] & 0x0F);

    // Get our Seq bits
    msg.seq = (data[1] << 4) | (data[2] >> 4);

    // Create a pointer for data (for easy manipulation)
    byte *ptr = (byte *)data;

    // Copy the payload portion of data into msg.data. Notice that this starts
    // at data[LEN_MAC] and steps for dataLength-LEN_MAC bytes. This can be
    // zero bytes if no payload is recieved
    memcpy(msg.data, ptr + LEN_MAC, dataLength - LEN_MAC);

    // Store the length of the payload in msg
    msg.len = dataLength - LEN_MAC;

    // Set this message to valid
    msg.valid = 1;
  }
  return msg;
}

int getMessageLength(Message msg) {
  return LEN_MAC + msg.len;
}

void createMessage(byte data[], Message msg) {

  // Set the Check Byte
  data[0] = 0x9B;

  // Set byte 1
  data[1] = (msg.from << 4) | (msg.seq >> 4);

  // Set byte 2
  data[2] = (msg.seq << 4) | msg.type;

  // Create a pointer for data
  byte *ptr = (byte *)data;

  // Copy the data from msg.data to the byte array starting at its third
  // byte
  memcpy(ptr + 3, msg.data, msg.len);
}

void printMessage(Message msg) {
  Serial.print('['); Serial.print(msg.valid); Serial.print("] ");
  Serial.print("From ("); Serial.print(msg.from);
  Serial.print(") Seq ("); Serial.print(msg.seq);
  Serial.print(") Type ("); Serial.print(MsgTypes[msg.type]);
  Serial.print(") Data {"); DW1000.printPrettyHex(msg.data, msg.len, false);
  Serial.print("} Memory: "); Serial.println(freeMemory());
}



int count = 0;
void loop() {

  // count ++;
  // digitalWrite(LED_PIN, count < 10);

  // if (count > 20) {
  //   count = 0;
  //   #ifdef DEBUG
  //   // Serial.print("Free "); Serial.println(freeMemory());
  //   #endif
  // }

  state.next(&state);

  // delay(10);

  // Serial.println(SINCE(0));
}
