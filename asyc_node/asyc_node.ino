// IMPORTS
// #include <SPI.h>
#include <DW1000.h>
#include <M0Timer.h>
#include "constants.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <RTCZero.h>

// END IMPORTS

// ========= Node IDs ========== //
#define LEN_NODES_LIST 3
constexpr uint16_t nodeList[] = {
  0x2243, // BASE 1
  0xDC19, // NODE 3
  0x805C, // NODE 4

  0x6606, // NODE 1
  0x5DCB, // BASE 2
  0xBFAA, // NODE 2


};

constexpr Antenna_Delay antennaDelayList[] {
    SHORT_ANTENNA, // BASE 2

    LONG_ANTENNA, // NODE 1
    LONG_ANTENNA, // NODE 2
    SHORT_ANTENNA, // NODE 3
    LONG_ANTENNA, // NODE 4

    SHORT_ANTENNA, // NODE 1
};
// ========= Node IDs ========== //

uint32_t milliTimer;

uint8_t msg_seq;
uint32_t cycle_counter;

// True if we can send it
uint32_t packet_sendable[LEN_NODES_LIST];

// Packet Buffer Vars
uint8_t buffer_start;
uint8_t buffer_num;
various_msg packet_buffer[MAX_RANGE_MESSAGES];
uint8_t packet_buffer_IDs[MAX_RANGE_MESSAGES];
uint8_t cmd_buffer_len;
various_msg cmd_buffer[MAX_CMD_MESSAGES];


uint8_t nodeNumber;
boolean isBase;

int8_t sleepCounter;
uint16_t sleepTime;
int8_t resetCounter;

// watchdog Settings
volatile boolean __watchdog_comp;
volatile boolean __watchdog_last;

// Global Settings
Settings settings;

// State control
State state;

// LED Vars
boolean led_rst;
uint16_t tlist[] = {0, 0, 0, 0};

// RX and TX Callback Functions
volatile uint32_t rxTimeM0;
volatile uint32_t txTimeM0;
DW1000Time rxTimeDW;
DW1000Time txTimeDW;
volatile boolean txFlag;
volatile boolean rxFlag;
void rxHandle() { rxTimeM0 = DW1000.getRxTime(); rxFlag = true; }
void txHandle() { txTimeM0 = DW1000.getTxTime(); txFlag = true; /*Serial.write((uint8_t)116); Serial.println(txTimeM0);*/ }
boolean checkRx() { if (rxFlag) { DW1000.getReceiveTimestamp(rxTimeDW); rxFlag = false; return true; } return false; }
boolean checkTx() { if (txFlag) { DW1000.getTransmitTimestamp(txTimeDW); txFlag = false; return true; } return false; }

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

// Cycle Timing and authorization
uint32_t cycleStart;
int cycleValid;
int transmitAuthorization;

// Range request information (TODO: Use this for error detection)
uint8_t rangeRequestFrom;
uint8_t rangeRequestSeq;

// Timers
boolean frameTimerSet;
boolean blockTimerSet;

// Error Flags
volatile boolean clkErr;
volatile boolean rxFail;
volatile boolean rxTimeout;
uint8_t numClockErrors;

// Mag/Accel sensor
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
boolean magEnabled;

// Real Time Clock Values
uint8_t min;
uint8_t hour;
uint8_t sec;
uint8_t set_min;
uint8_t set_hour;
uint8_t set_sec;
volatile boolean clockUpdateReceived;

// Real time clock object
RTCZero rtc;

void setup() {

  // Init all global variables
  clkErr = false;
  rxFail = false;
  rxTimeout = false;
  numClockErrors = 0;

  msg_seq = 0;
  cmd_buffer_len = 0;
  buffer_start = 0;
  buffer_num = 0;
  cycle_counter = 0;

  magEnabled = false;
  sleepCounter = -1;
  resetCounter = -1;
  sleepTime = 0;

  rxFlag = false;
  txFlag = false;

  set_min = 0;
  set_hour = 0;
  set_sec = 0;
  clockUpdateReceived = false;

  // Set up our cycle information vars
  cycleStart = 0;
  cycleValid = 0;
  transmitAuthorization = 0;


  DWOffset = 0; // DW1000Time(0, DW1000Time::MICROSECONDS);

  // === Define some default settigs =========================================//
  settings = {
    // .mode    = DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
    // .channel = DW1000.CHANNEL_3,
    //
    // .n       = LEN_NODES_LIST,
    // .t_rx    = 4000,  // Buffer time for changing rx/tx mode // 1000
    // .t_b     = 3000,  // Buffer time between all blocks // 1000
    //
    // .t_r     = 4000,  // Time between range responses - longer than range_resp // 4000
    //                  //  message length (~3ms)
    // .n_com   = 1,     // Number of com frames per cycle
    // .bits_c  = 16,    // Number of bits allowed in a com message
    // .t_cl    = 3000,  // Time for a single com message - longer than com_msg
    //                  //  length
    // .t_s     = 5000,  // Time for the sleep frame

    .mode    = DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
    .channel = DW1000.CHANNEL_3,

    .n       = LEN_NODES_LIST,
    .t_rx    = 4000,  // Buffer time for changing rx/tx mode // 1000
    .t_b     = 10000,  // Buffer time between all blocks // 1000

    .t_r     = 10000,  // Time between range responses - longer than range_resp // 4000
                     //  message length (~3ms)
    .n_com   = 3,     // Number of com frames per cycle
    .bits_c  = 16,    // Number of bits allowed in a com message
    .t_cl    = 4000,  // Time for a single com message - longer than com_msg
                     //  length
    .t_s     = 5000,  // Time for the sleep frame

    .power   = 0,// 0x1F1F1F1FL, // The manual transmit power
  };

  // Start up the real time clock and get the current time
  initRTC();
  updateRTC();

  // Set our LED Pinmodes to output
  pinMode(LED_PIN, OUTPUT);
  pinMode(BOARD_RGB_RED, OUTPUT);
  pinMode(BOARD_RGB_GREEN, OUTPUT);
  pinMode(BOARD_RGB_BLUE, OUTPUT);

  // Start up the LED Timer
  M0Timer.setup(_LED_TIMER);
  M0Timer.attachTC5Handler(t_leds);
  M0Timer.startms(10, _LED_TIMER);

  // Set all of our packets as sendable
  for (int i = 0; i < LEN_NODES_LIST; i++) {
    packet_sendable[i] = 0xFFFFFFFF;
  }

  // Start the serial interface
  Serial.begin(BAUD_RATE);
  // Figure out what node number we are
  // First see if we are the first address in the node list. If we are, then we
  // are also the base station
  uint16_t ownAddress = getShortAddress();
  if (nodeList[0] == ownAddress) {
    isBase = true;
    nodeNumber = 0;

    // Set the RTC time
    rtc.setTime(13, 20, 13);
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
      sprintf(large_buf, "Error:%s Node ID Not in List: %04X", D_CLEAR, ownAddress);
      section(large_buf, C_RED);
      for (uint8_t i = 0; i < LEN_NODES_LIST; i++) {
        sprintf(tiny_buf, "%04X", nodeList[i]);
        pcln(tiny_buf);
      }

      endSection("SYSTEM FAILURE", C_RED);
      // Kill The Program
      for (;;)
        delay(1000);

    }
  }

  // Block until serial is connected if required
  if (BLOCK_SERIAL_BASE && isBase) {
    setLed(LED_RED, MODE_RAMP);
    while(!Serial);
    setLed(LED_RED, MODE_OFF);
  } else if (BLOCK_SERIAL_NODE && !isBase) {
    setLed(LED_RED, MODE_RAMP);
    while(!Serial);
    setLed(LED_RED, MODE_OFF);
  }

  header("ASYC NODE", C_BLACK, BG_YELLOW);
  inc();

  // Print details about our node number
  section("System Boot");
  sprintf(small_buf, "Own Address: %04X", ownAddress); pcln(small_buf);
  sprintf(small_buf, " -> Node #%d", nodeNumber); pcln(small_buf);
  if (isBase) {
    sprintf(small_buf, " -> IS BASE", nodeNumber); pcln(small_buf, C_GREEN);
  }
  pcln("");


  if (isBase) {
    Serial5.begin(BAUD_RATE);
    pcln("Serial5 INIT");
  } else {
    pcln("Serial5 Inactive");
  }

  // Start up the Mag/Accel sensor. If it is not installed, then magEnabled
  // will be false
  magEnabled = mag.begin();
  // Only report an error if we are not the base station
  if (!magEnabled && !isBase) {
    pcln("Mag Sensor Error!", BG_RED_C_WHITE);
  }


// INIT DW1000 with the correct pins
#ifdef MNSLAC_NODE_M0
  pcln("MNSLAC Node Hardware detected");
  DW1000.begin(PIN_IRQ_NODE, PIN_RST_NODE);
  DW1000.select(PIN_SS_NODE);
#else
  pcln("Non MNSLAC Node Hardware detected", C_ORANGE);
  DW1000.begin(PIN_IRQ_BREAD, PIN_RST_BREAD);
  DW1000.select(PIN_SS_BREAD);
#endif

  // Reset the DW1000 incase we were on before
  DW1000.reset();

  // Start a new DW1000 Config
  DW1000.newConfiguration();

  // Start with the default settings
  DW1000.setDefaults();

  // Set our addresses and note what our personal address is
  setAddresses(0xDECA);

  // Enable our Mode
  DW1000.enableMode(settings.mode);

  // Set our antenna delay
  DW1000.setAntennaDelay(antennaDelayList[nodeNumber]);

  // Manually set the power
  if (settings.power != 0)
    DW1000.setManualPower(settings.power);

  // Commit the config to the DW1000
  DW1000.commitConfiguration();

  // Set up interrputs
  DW1000.interruptOnRxPreambleDetect(false);
  DW1000.interruptOnTxPreambleSent(false);
  DW1000.interruptOnRxFrameStart(true);
  DW1000.interruptOnTxFrameStart(true);

  // Attach DW1000 Handlers
  DW1000.attachSentHandler(txHandle);
  DW1000.attachReceivedHandler(rxHandle);
  DW1000.attachReceiveFailedHandler(rxFailHandler);
  DW1000.attachReceiveTimeoutHandler(rxTimeoutHandler);
  DW1000.attachErrorHandler(clkErrHandler);

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
  settings.t_c  = settings.t_fs + settings.t_fr + settings.t_fc;

  DW1000Time timeList[settings.n];
  timePollReceived = timeList;

  t_r = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);
  // =========================================================================//

  // Print details about our config
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  sprintf(medium_buf, "Device ID: %s", msg); pcln(medium_buf);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  sprintf(medium_buf, "Unique ID: %s", msg); pcln(medium_buf);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  sprintf(medium_buf, "Network ID & Device Address: %s", msg); pcln(medium_buf);
  DW1000.getPrintableDeviceMode(msg);
  sprintf(medium_buf, "Device Mode: %s", msg); pcln(medium_buf);
  sprintf(medium_buf, "Antenna Delay: %d", DW1000.getAntennaDelay()); pcln(medium_buf);
  sprintf(medium_buf, "Power Setting: %08X", DW1000.getManualPower()); pcln(medium_buf);
  endSection("Boot Success\n\r", C_GREEN);
  printSettings(settings);

  // Set up recieve mode
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  // Configure our block and frametimers
  // Block Timer
  M0Timer.setup(_BLOCK_TIMER);
  M0Timer.setSingleUse(_BLOCK_TIMER);
  M0Timer.attachTC4Handler(t_block);
  M0Timer.stop(_BLOCK_TIMER); // Not Needed


  // Frame Timer
  M0Timer.setup(_FRAME_TIMER);
  M0Timer.setSingleUse(_FRAME_TIMER);
  M0Timer.attachTC3Handler(t_frame);
  M0Timer.stop(_FRAME_TIMER); // Not Needed

  // Ensure that the timers start stopped and that our flag vars are set to
  // false
  stopTimers();

  // Setup our initial state
  state = { sleep };

  // Start up the watchdog timer
  enableWatchdog();

  // Reset indentation
  rst();
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

// Get the Unique ID of this SAMD21
uint16_t getShortAddress() {
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

  return val4*256+val3;
}

// Update our stored time values from the RTC
void updateRTC() {
  min = rtc.getMinutes();
  hour = rtc.getHours();
  sec = rtc.getSeconds();
}

// Start up the RTC
void initRTC() {
  rtc.begin(); // initialize RTC

  // Set the default time
  rtc.setTime(0, 0, 0);

  // Set the default date
  rtc.setDate(1, 1, 2000);
}

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

// Callback for the block timer
void t_block(uint8_t t)
{
  uint32_t now = micros();
  Serial.print("b");
  Serial.println(now);
}

// Callback for the frame timer
void t_frame(uint8_t t)
{
  uint32_t now = micros();
  Serial.print("f");
  Serial.println(now);
}

// Set the given led to the given mode
void setLed(Led led, Led_Mode mode) {
  switch (mode) {
    // For the MODE OFF and MODE ON cases, we can just set them
    // as needed right away
    case MODE_OFF:
      digitalWrite(led_pin_list[led], LOW);
      led_modes[led] = MODE_NULL;
      break;
    case MODE_ON:
      digitalWrite(led_pin_list[led], HIGH);
      led_modes[led] = MODE_NULL;
      break;
    default:
      // Set the mode of this led accordingly
      led_modes[led] = mode;
      break;
  }
  // Alert the watchdog with the updated flag
  led_updated[led] = true;

}

// Led handler
void t_leds(uint8_t t) {
  // If led_rst, then set the led time counters to 0
  if(led_rst){
    for (int i = 0; i < 4; i++)
      tlist[i] = 0;
  }

  // Go through all of the LEDs
  for (int i = 0; i < 4; i++) {
    // See if the led has been updated by the user
    if (led_updated[i]) {
      // If so, clear the flag
      led_updated[i] = false;

      // Set its time counter to 0
      // tlist[led] = 0;
    }

    // Manage this led
    manage_led((Led) i);
  }
}

// Helper function for setting leds
float led_val;
void manage_led(Led led) {
  uint8_t pin = led_pin_list[led];
  Led_Mode mode = led_modes[led];
  uint16_t t = tlist[led];

  switch(mode){
    case MODE_OFF:
      digitalWrite(pin, LOW);
      led_modes[led] = MODE_NULL;
      break;
    case MODE_ON:
      digitalWrite(pin, HIGH);
      led_modes[led] = MODE_NULL;
      break;
    case MODE_CHIRP:
      digitalWrite(pin, HIGH);
      led_modes[led] = MODE_OFF;
      break;
    case MODE_BLINK:
      if(t == 40){
        digitalWrite(pin, HIGH);
      }else if(t == 80){
        digitalWrite(pin, LOW);
      }else if(t > 80){
        tlist[led] = 0;
      }
      tlist[led]++;
      break;
    case MODE_BLINK_DIM:
      if(t == 40){
        analogWrite(pin, 100);
      }else if(t == 80){
        analogWrite(pin, 0);
      }else if (t > 80){
        tlist[led] = 0;
      }
      tlist[led]++;
      break;
    case MODE_RAMP:
      if(t < 5){
        led_val = t*51;
      }else if(t <= 40){
        led_val = 255-(t-5)*7.285714286;
      }else{
        led_val = 0;
      }
      if(t > 100){
        tlist[led] = 0;
      }
      analogWrite(pin, (int)led_val);

      tlist[led]++;
      break;
    case MODE_DOUBLE_RAMP:
      if(t < 5){
        led_val = t*51;
      }else if(t <= 40){
        led_val = 255-(t-5)*7.285714286;
      }else if(t < 45){
        led_val = t*51;
      }else if(t <= 80){
        led_val = 255-(t-45)*7.285714286;
      }else{
        led_val = 0;
      }
      if(t > 150){
        tlist[led] = 0;
      }
      analogWrite(pin, (int)led_val);
      tlist[led]++;
      break;
  }
}

// Main Loop
void loop() {

  // Execute the next state
  state.next(&state);

  // Check to see if our stall timer has expired
  if (millis() - milliTimer > MAX_STALL_TIME) {
    rst();
    sprintf(medium_buf, "Stall Timer Fired (from loop) %lu", micros()); pcln(medium_buf);
    state.next = sleep;
  }

  // Check if there was a clock error
  if (clkErr){
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

  // Check if there was an RX Failure
  if (rxFail){
    rxFail = false;
    Serial.println("DW1000 Receive Failure Detected");
  }

  // Check if there was an RX Timeout
  if (rxTimeout){
    rxTimeout = false;
    Serial.println("DW1000 Receive Timeout");
  }
}
