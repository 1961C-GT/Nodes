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

Settings settings;
  // uint8_t n       : 4;  // Number of nodes in the network
  // uint16_t t_rx   : 16; // Buffer time for changing rx/tx mode
  // uint16_t t_b    : 16; // Buffer time between all blocks
  //
  // uint16_t t_r    : 16; // Time between range responses - longer than range_resp message length (~3ms)
  //
  // uint8_t n_com   : 8;  // Number of com frames per cycle
  // uint16_t bits_c : 16; // Number of bits allowed in a com message
  // uint16_t t_cl   : 16; // Time for a single com message - longer than com_msg length
  //
  // uint32_t t_sleep; // Time for the sleep frame
  //
  // // --- Calculated Settings (from given)
  // uint32_t t_br;        // Total time for the each ranging block
  // uint32_t t_fr;        // Total time for the ranging frame (all blocks)
  //
  // uint32_t t_bc;        // Total time for each com block
  // uint32_t t_fc;        // Total time for all com frames (all blocks)


// State
State state;

// HELPER FUNCTIONS:
// RX and TX Callback Functions
volatile uint32_t rxTime;
volatile uint32_t txTime;
volatile boolean txFlag = false;
volatile boolean rxFlag = false;
void txHandle() { txTime = DW1000.getTxTime(); txFlag = true; /*Serial.print('t'); Serial.println(txTime);*/ }
void rxHandle() { rxTime = DW1000.getRxTime(); rxFlag = true; /*Serial.print('r'); Serial.println(rxTime);*/ }
boolean checkTx() { if (txFlag) { txFlag = false; return true; } return false; }
boolean checkRx() { if (rxFlag) { rxFlag = false; return true; } return false; }

void (* _TC3Callback)(TcCount16* tc);

// Timing Values
DW1000Time timePollSent;
DW1000Time * timePollReceived;

// Testing
volatile boolean reset;
volatile boolean led;
volatile boolean blink;

DWMODE dwMode;
DW1000Time msgDelay;
DW1000Time t1;

uint32_t cycleStart;
boolean cycleValid;

// Error Flags
volatile boolean clkErr;
volatile boolean rxFail;
volatile boolean rxTimeout;

uint8_t numClockErrors;

void setup() {

  clkErr = false;
  rxFail = false;
  rxTimeout = false;
  numClockErrors = 0;

  // === Define some default settigs =========================================//
  settings = {
    .mode = DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
    .channel = DW1000.CHANNEL_3,

    .n      = 3,
    .t_rx   = 100,  // Buffer time for changing rx/tx mode // 100
    .t_b    = 5000,  // Buffer time between all blocks // 1000

    .t_r    = 4000,  // Time between range responses - longer than range_resp // 3000
                     //  message length (~3ms)
    .n_com  = 3,     // Number of com frames per cycle
    .bits_c = 16,    // Number of bits allowed in a com message
    .t_cl   = 3000,  // Time for a single com message - longer than com_msg
                     //  length
    .t_sleep = 5000, // Time for the sleep frame
  };

  // Set up our cycle information vars
  uint32_t cycleStart = 0;
  boolean cycleValid = false;

  pinMode(LED_PIN, OUTPUT);

  // Start the serial interface
  Serial.begin(115200);
  // while(!Serial);
  Serial.println(F("### ASYC Node ###"));

  // #ifdef DEBUG
  // if (!isBase)
  //   while(!Serial);
  // #endif

  // INIT DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);


  // Start a new DW1000 Config
  DW1000.newConfiguration();

  // Start with the default settings
  DW1000.setDefaults();

  // Set our addresses and note what our personal address is
  uint16_t ownAddress = setAddresses(0xDECA); // Want to change to 1961 but the mac address stuff is hardcoded

  // Enable our Mode
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

  DW1000.interruptOnRxPreambleDetect(false);
  DW1000.interruptOnTxPreambleSent(false);
  DW1000.interruptOnRxFrameStart(true);
  DW1000.interruptOnTxFrameStart(true);

  // Commit the config to the DW1000
  DW1000.commitConfiguration();


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

  // === Calculate some additional settings based on the current settings === //
  // Calculate the length of a single ranging block. Two times t_rx (one at the
  // start and one at the end), one t_r for each node, and t_b buffer at the end
  settings.t_br = settings.t_rx + settings.t_r + (settings.n * settings.t_r) + settings.t_rx + settings.t_b;

  // Calculate the length of the entire ranging frame. One ranging block for
  // each node in the network
  settings.t_fr = settings.n * settings.t_br;

  // Calculate the time for a single com block. Two times t_rx (one at the
  // start and one at the end), one t_cl (for the message) and t_b buffer at the
  // end
  settings.t_bc = (2 * settings.t_rx) + settings.t_cl + settings.t_b;

  // Calculate the length of the enitre com frame. One com block per node in
  // the network, times the number of times we repeat the frame per cycle
  settings.t_fc = settings.n * settings.t_bc * settings.n_com;

  // The time that this node waits after hearing a RANGE_REQ before sending
  // a RANGE_RES in us
  settings.t_rn = (settings.t_rx + settings.t_r) +  (settings.t_r * nodeNumber);

  DW1000Time timeList[settings.n];
  timePollReceived = timeList;

  // while(!Serial);
  // *(timePollReceived + 0) = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);
  // Serial.println(*(timePollReceived + 0));
  // =========================================================================//



  // if (!isBase)
  //   while(!Serial);


  Serial.println(F("Committed configuration ..."));

  // Print details about our node number
  Serial.print("Own Address:"); Serial.print(ownAddress, HEX);
  Serial.print(" -- Node #:"); Serial.print(nodeNumber);
  if (isBase)
    Serial.println(" -- IS BASE");
  else
    Serial.println();

  // Attach DW1000 Handlers
  DW1000.attachSentHandler(txHandle);
  DW1000.attachReceivedHandler(rxHandle);
  DW1000.attachReceiveFailedHandler(rxFailHandler);
  DW1000.attachReceiveTimeoutHandler(rxTimeoutHandler);
  DW1000.attachErrorHandler(clkErrHandler);


  // Print details about our config
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);

  printSettings(settings);

  // Set up recieve mode
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  // Init global vars
  dwMode = IDLE;
  reset = false;
  led = false;

  // Configure our timers
  // LED Timer
  M0Timer.setup(_LED_TIMER);
  M0Timer.attachTC5Handler(t_blink);
  // M0Timer.start(150, M0Timer.T5);

  // Block Timer
  M0Timer.setup(_BLOCK_TIMER);
  M0Timer.setSingleUse(_BLOCK_TIMER);
  M0Timer.stop(_BLOCK_TIMER); // Not Needed


  // Frame Timer
  M0Timer.setup(_FRAME_TIMER);
  M0Timer.setSingleUse(_FRAME_TIMER);
  M0Timer.stop(_FRAME_TIMER); // Not Needed
    // M0Timer.attachTC3Handler(tec);
  // M0Timer.start(500000,M0Timer.T3, true); // 500ms

  // TODO Modify the M0Timer lib to allow us to "set up" a timer some other
  // way. Starting and stopping it is overkill
  // M0Timer.startTimer(100,3);
  // M0Timer.stopTimer(3);

  // Calculate settings
  msgDelay = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);

  Serial.print("Msg Delay: "); Serial.println(msgDelay);
  Serial.print("Msg Delay: "); Serial.println(msgDelay.getAsMicroSeconds());

  // Set our initial state
  if (isBase) {
    state = { rb_range };
  } else {
    state = { ra_init };
  }

  // Boot up delay
  delay(10);
}


// Set the EUI based on this SAMD21G18's unique ID. Also set the
// device address based on the unique ID and set the network ID as given
uint16_t setAddresses(uint16_t net) {
  // Get the Unique ID of this SAMD21
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

  // Build the EUI Address based on these bytes
  byte addr[] = {val4, val3, val2, val1, val8, val7, val6, val5};

  // Set the EUI Address
  DW1000.setEUI(addr);

  // Set the device address from the upper two bytes
  DW1000.setDeviceAddress(val4*256+val3);

  // set the network ID based on the provided value
  DW1000.setNetworkId(net);

  // Return the device address we set
  return val4*256+val3;
}

void blinkLoop() {
  M0Timer.startms(100, _LED_TIMER);
}

void blinkInit() {
  // M0Timer.start(10, M0Timer.T5);
  M0Timer.stop(_LED_TIMER);
  blink = false;
  digitalWrite(LED_PIN, HIGH);
}

void blinkTx() {
  M0Timer.startms(25, _LED_TIMER);
}

void receiver() {
  // if (dwMode != RX) {
  	DW1000.newReceive();
  	DW1000.setDefaults();
  	// so we don't need to restart the receiver manually
  	DW1000.receivePermanently(true);
  	DW1000.startReceive();
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

// Handlers
void rxFailHandler() {
  rxFail = true;
}
void rxTimeoutHandler() {
  rxTimeout = true;
}
void clkErrHandler() {
  clkErr = true;
}

void t_blink(uint8_t t) {
  blink = true;
  // Serial.print("Rx Time:"); Serial.print(DW1000.getRxTime());
  // Serial.print(" - Tx Time:"); Serial.println(DW1000.getTxTime());
}

// uint32_t last = 0;
// uint32_t goal = 100; //100000;
// #define STEP 100
// void tec(uint8_t t) {
//   goal = 45600;
//   uint32_t now = micros();
//   M0Timer.start(45600, M0Timer.T3, M0Timer._intTime[M0Timer.T3]);
//   Serial.print(goal); Serial.print(','); Serial.println(now - last);// - M0Timer._intTime[M0Timer.T3]);
//   last = now;
//   goal = goal + STEP;
// }

// Main Loop
void loop() {
  state.next(&state);
  // M0Timer.start(100, M0Timer.T5, true);
  if (blink) {
    // Serial.println("b");
    led = !led;
    digitalWrite(LED_PIN, led);
    blink = false;
  }
  if (clkErr){
    clkErr = false;
    numClockErrors++;
    Serial.println("DW1000 Clock Error Detected");
    if (numClockErrors > 3) {
      // Serial.println("Forcing System Soft Reboot");
      // setup();
      // NVIC_SystemReset();
      // return;
    }
  }
  if (rxFail){
    rxFail = false;
    Serial.println("DW1000 Receive Failure Detected");
  }
  if (rxTimeout){
    rxTimeout = false;
    Serial.println("DW1000 Receive Timeout");
  }
}
