// IMPORTS
#include <SPI.h>
#include <DW1000.h>
#include <M0Timer.h>
#include "constants.h"
// END IMPORTS

// ========= Node IDs ========== //
#define LEN_NODES_LIST 2
constexpr uint16_t nodeList[] = {
  0xDC19,
  0x6606,  // NODE 1
  0x726C, // RED - Base Station
  0x71E9, // BLUE
  0xFDA0,  // GREEN
};
// ========= Node IDs ========== //


uint8_t nodeNumber;
boolean isBase;

// Global Settings
Settings settings;

// State control
State state;

// RX and TX Callback Functions
volatile uint32_t rxTimeM0;
volatile uint32_t txTimeM0;
DW1000Time rxTimeDW;
DW1000Time txTimeDW;
volatile boolean txFlag = false;
volatile boolean rxFlag = false;
void rxHandle() {
  rxTimeM0 = DW1000.getRxTime();
  rxFlag = true; /*Serial.print('r'); Serial.println(rxTimeM0);*/
}
void txHandle() {
  txTimeM0 = DW1000.getTxTime();
  txFlag = true; /*Serial.print('t'); Serial.println(txTimeM0);*/
}
boolean checkRx() {
  if (rxFlag) {
    DW1000.getReceiveTimestamp(rxTimeDW);
    rxFlag = false;
    return true;
  } return false;
}
boolean checkTx() {
  if (txFlag) {
    DW1000.getTransmitTimestamp(txTimeDW);
    txFlag = false;
    return true;
  } return false;
}

// Timing Values
DW1000Time timePollSent;
DW1000Time * timePollReceived;
DW1000Time t_r;

// The difference in time between the DW1000 time and the M0 time. Calculated
// using M0 - DW1000(micros). To get DW time from M0 time, use M0 - DWOffset.
// To get M0 time from DW time, use DW + DWOffset.
uint32_t DWOffset;

// Messages
Message rxMessage;
Message txMessage;
Message emptyMessage;

// Testing
volatile boolean led;
volatile boolean blink;

// Cycle Timing
uint32_t cycleStart;
int cycleValid;
uint8_t lastSequenceNumber;
uint8_t rangeRequestFrom;
uint8_t rangeRequestSeq;
boolean transmitAuthorization;

// Timers
boolean frameTimerSet;
boolean blockTimerSet;

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

  // emptyMessage = {
  //   .from    = 0,
  //   .type    = 0,
  //   .seq     = 0,
  //   .len     = 0,
  //   .valid   = 0,
  // };

  // === Define some default settigs =========================================//
  settings = {
    .mode    = DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
    .channel = DW1000.CHANNEL_3,

    .n       = LEN_NODES_LIST,
    .t_rx    = 4000,  // Buffer time for changing rx/tx mode // 1000
    .t_b     = 1000,  // Buffer time between all blocks // 1000

    .t_r     = 4000,  // Time between range responses - longer than range_resp // 3000
    //  message length (~3ms)
    .n_com   = 3,     // Number of com frames per cycle
    .bits_c  = 16,    // Number of bits allowed in a com message
    .t_cl    = 3000,  // Time for a single com message - longer than com_msg
    //  length
    .t_s     = 5000,  // Time for the sleep frame
  };

  // Set up our cycle information vars
  cycleStart = 0;
  cycleValid = 0;
  lastSequenceNumber = 0;

  transmitAuthorization = false;

  pinMode(LED_PIN, OUTPUT);

  // Start the serial interface
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("### ASYC Node ###"));

  // Figure out where we are in the node list
  uint16_t ownAddress = getShortAddress();
  nodeNumber = -1;
  for (uint8_t i = 0; i < LEN_NODES_LIST; ++i) {
    if (nodeList[i] == ownAddress) {
      nodeNumber = i;
      break;
    }
  }

  if (nodeNumber == 0) {
    // If we're the first node in the list, assume we're the base
    isBase = true;
  } else if (nodeNumber == -1) {
    // We're not in the node list, print out some debugging info
    while (!Serial);
    Serial.print("Error: Node ID not in list: "); Serial.println(ownAddress, HEX);
    for (uint8_t i = 0; i < LEN_NODES_LIST; ++i) {
      Serial.print("- "); Serial.println(nodeList[i], HEX);
    }
    // Spin our wheels doing nothing...
    for (;;);
  }

  // INIT DW1000
#ifdef MNSLAC_NODE_M0
  Serial.println("MNSLAC Node Hardware detected");
  DW1000.begin(PIN_IRQ_NODE, PIN_RST_NODE);
  DW1000.select(PIN_SS_NODE);
#else
  Serial.println("Non MNSLAC Node Hardware detected");
  DW1000.begin(PIN_IRQ_BREAD, PIN_RST_BREAD);
  DW1000.select(PIN_SS_BREAD);
#endif


  // Start a new DW1000 Config
  DW1000.newConfiguration();

  // Start with the default settings
  DW1000.setDefaults();

  // Set our addresses and note what our personal address is
  setAddresses(0xDECA); // Want to change to 1961 but the mac address stuff is hardcoded

  // Enable our Mode
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

  DW1000.interruptOnRxPreambleDetect(false);
  DW1000.interruptOnTxPreambleSent(false);
  DW1000.interruptOnRxFrameStart(true);
  DW1000.interruptOnTxFrameStart(true);

  // Commit the config to the DW1000
  DW1000.commitConfiguration();

  // === Calculate some additional settings based on the current settings === //
  // Calculate the length of a single ranging block. One t_rx delay, one t_r
  // delay to account for the intial RANGE_REQUEST message, one t_r for each
  // node, and t_b buffer at the end
  settings.t_br = settings.t_rx + settings.t_r + (settings.n * settings.t_r) + settings.t_b;

  // Calculate the length of the entire ranging frame. One ranging block for
  // each node in the network
  settings.t_fr = settings.n * settings.t_br;

  // Calculate the time for a single com block. Two times t_rx (one at the
  // start and one at the end), one t_cl (for the message) and t_b buffer at the
  // end
  settings.t_bc = settings.t_rx + settings.t_cl + settings.t_b;

  // Calculate the length of the enitre com frame. One com block per node in
  // the network, times the number of times we repeat the frame per cycle
  settings.t_fc = settings.n * settings.t_bc * settings.n_com;

  // The time that this node waits after hearing a RANGE_REQ before sending
  // a RANGE_RES in us
  settings.t_rn = settings.t_r +  (settings.t_r * nodeNumber);

  // Calculate the length of the entire sleep frame
  settings.t_fs = settings.t_rx + settings.t_s + settings.t_b;

  // Calculate the entire cycle length
  settings.t_c  = settings.t_fs + settings.t_fr;

  DW1000Time timeList[settings.n];
  timePollReceived = timeList;

  // while(!Serial);
  // *(timePollReceived + 0) = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);
  // Serial.println(*(timePollReceived + 0));
  // =========================================================================//



  if (isBase)
    while (!Serial);


  Serial.println(F("Committed configuration ..."));

  // Print details about our node number
  Serial.print("Own Address:"); Serial.print(ownAddress, HEX);
  Serial.print(" -- Node #"); Serial.print(nodeNumber);
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
  led = false;

  // Configure our timers
  // LED Timer
  M0Timer.setup(_LED_TIMER);
  M0Timer.attachTC5Handler(t_blink);

  // Block Timer
  M0Timer.setup(_BLOCK_TIMER);
  M0Timer.setSingleUse(_BLOCK_TIMER);
  M0Timer.stop(_BLOCK_TIMER); // Not Needed


  // Frame Timer
  M0Timer.setup(_FRAME_TIMER);
  M0Timer.setSingleUse(_FRAME_TIMER);
  M0Timer.stop(_FRAME_TIMER); // Not Needed

  // Ensure that the timers start stopped and that our flag vars are set to
  // false
  stopTimers();

  // Calculate settings
  t_r = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);

  // Serial.print("Msg Delay: "); Serial.println(t_r);
  // Serial.print("Msg Delay: "); Serial.println(t_r.getAsMicroSeconds());
  //
  // for (int i = 0; i < 32; i++) {
  //   Serial.print("Seq: " ); Serial.print(i); Serial.print(" -- Time: "); Serial.println(getTimeAtSeq(i));
  // }

  DWOffset = 0; // DW1000Time(0, DW1000Time::MICROSECONDS);

  state = { sleep };

  // Set our initial state
  if (isBase) {
    // state = { rb_range };
    cycleValid = 10;
    cycleStart = micros();
    lastSequenceNumber = 255;
    transmitAuthorization = true;
  }

  Serial.println("\033[0;35mThis is a test!");

  // else {
  //   state = { ra_init };
  // }
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
  DW1000.setDeviceAddress(val4 * 256 + val3);

  // set the network ID based on the provided value
  DW1000.setNetworkId(net);

  // Return the device address we set
  return val4 * 256 + val3;
}

uint16_t getShortAddress() {
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

  return val4 * 256 + val3;
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
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
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
}

// Main Loop
void loop() {
  // Execute the next state
  state.next(&state);

  // float voltage = analogRead(A2) * 0.0005900679758;
  // Serial.println(getBattVoltage());

  // M0Timer.start(100, M0Timer.T5, true);
  if (blink) {
    // Serial.println("b");
    led = !led;
    digitalWrite(LED_PIN, led);
    blink = false;
  }
  if (clkErr) {
    clkErr = false;
    numClockErrors++;
    // Serial.println("DW1000 Clock Error Detected");
    if (numClockErrors > 3) {
      // Serial.println("Forcing System Soft Reboot");
      // setup();
      // NVIC_SystemReset();
      // return;
    }
  }
  if (rxFail) {
    rxFail = false;
    Serial.println("DW1000 Receive Failure Detected");
  }
  if (rxTimeout) {
    rxTimeout = false;
    Serial.println("DW1000 Receive Timeout");
  }
}
