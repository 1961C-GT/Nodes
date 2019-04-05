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
#define LEN_NODES_LIST 2
constexpr uint16_t nodeList[] = {
  0x5DCB, // BASE 2

  0x6606, // NODE 1

  0x2243, // BASE 1

  0xBFAA, // NODE 2
  0xDC19, // NODE 3
  0x805C, // NODE 4


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

uint8_t msg_seq = 0;
uint32_t cycle_counter;

// TODO: Remove
uint32_t runningAverage;
uint32_t measurement_counter;

// True if we can send it
uint32_t packet_sendable[LEN_NODES_LIST];

// uint8_t range_array_pointer = 0;
// range_msg range_array[MAX_RANGE_MESSAGES];

uint8_t buffer_start = 0;
uint8_t buffer_num = 0;
various_msg packet_buffer[MAX_RANGE_MESSAGES];
uint8_t packet_buffer_IDs[MAX_RANGE_MESSAGES];

uint8_t ta_msg_seq = 0;
uint8_t cmd_buffer_len = 0;
various_msg cmd_buffer[MAX_CMD_MESSAGES];


uint8_t nodeNumber;
boolean isBase;

boolean softReset;
boolean validTime;

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
volatile boolean txFlag = false;
volatile boolean rxFlag = false;
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
Message emptyMessage;

// Testing
volatile boolean led;
volatile boolean blink;

// Cycle Timing
uint32_t cycleStart;
int cycleValid;
int transmitAuthorization;

uint8_t lastSequenceNumber;
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

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
boolean magEnabled;

uint8_t min;
uint8_t hour;
uint8_t sec;

// Real time clock object
RTCZero rtc;

void setup() {

  clkErr = false;
  rxFail = false;
  rxTimeout = false;
  numClockErrors = 0;

  msg_seq = 0;
  ta_msg_seq = 0;
  cmd_buffer_len = 0;
  buffer_start = 0;
  buffer_num = 0;
  cycle_counter = 0;
  measurement_counter = 0;

  magEnabled = false;

  runningAverage = 0;

  validTime = false;

  initRTC();

  // LED Timer
  M0Timer.setup(_LED_TIMER);
  M0Timer.attachTC5Handler(t_leds);
  M0Timer.startms(10, _LED_TIMER);

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
    .t_b     = 3000,  // Buffer time between all blocks // 1000

    .t_r     = 4000,  // Time between range responses - longer than range_resp // 4000
                     //  message length (~3ms)
    .n_com   = 1,     // Number of com frames per cycle
    .bits_c  = 16,    // Number of bits allowed in a com message
    .t_cl    = 3000,  // Time for a single com message - longer than com_msg
                     //  length
    .t_s     = 5000,  // Time for the sleep frame
  };

  // Set all of our packets as sendable
  for (int i = 0; i < LEN_NODES_LIST; i++) {
    packet_sendable[i] = 0xFFFFFFFF;
  }

  // Set up our cycle information vars
  cycleStart = 0;
  cycleValid = 0;
  transmitAuthorization = 0;
  softReset = false;

  lastSequenceNumber = 0;



  pinMode(LED_PIN, OUTPUT);
  pinMode(BOARD_RGB_RED, OUTPUT);
  pinMode(BOARD_RGB_GREEN, OUTPUT);
  pinMode(BOARD_RGB_BLUE, OUTPUT);

  // digitalWrite(BOARD_RGB_RED, HIGH);
  // digitalWrite(BOARD_RGB_GREEN, HIGH);
  // digitalWrite(BOARD_RGB_BLUE, HIGH);

  // Start the serial interface
  Serial.begin(256000);



  // float r, g, b, t;
  // r = 0; g = 0; b = 0; t = 0;


  // setLed(LED_RED, MODE_RAMP);
  // while(!Serial);
  // setLed(LED_RED, MODE_OFF);

  header("ASYC NODE", C_BLACK, BG_YELLOW);
  inc();
  // header("SLEEP FRAME", C_BLACK, BG_GREEN);
  // header("RANGE FRAME", C_WHITE, BG_RED);


  // pcln("Program Boot", BG_RED_C_WHITE);
  //
  // inc(); pind(); p("This is some "); pc("CoOl StUfF", C_GREEN); pln(" Same here");
  //
  // pind(); sprintf(small_buf, "This is some %sCoOl StUfF%s Same here", C_GREEN, D_CLEAR); Serial.println(small_buf);
  //
  // rst();
  //
  // section("Boot Information");
  // pcln("Loading Data", C_RED);
  // pcln("  -> 0xAbnfn33adas", C_RED);
  // pcln("Being Awesome", C_ORANGE);
  // endSection("SUCCESS", C_GREEN);

  // Figure out what node number we are
  // First see if we are the first address in the node list. If we are, then we
  // are also the base station
  uint16_t ownAddress = getShortAddress();
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

  // #ifdef DEBUG
  if (isBase)
  {
    setLed(LED_RED, MODE_RAMP);
    Serial5.begin(256000);
    while (!Serial);
    // Serial5.println("Base Station Data Serial");
    setLed(LED_RED, MODE_OFF);

    Serial.println(sizeof(various_msg));
    Serial.println(sizeof(cmd_msg));
    Serial.println(sizeof(stats_msg));

    // Set the default time
    rtc.setTime(13, 20, 13);

  } else {
    magEnabled = mag.begin();
    if (!magEnabled) {
      pcln("Mag Sensor Error!", BG_RED_C_WHITE);
    }
  }
// #endif

// INIT DW1000
#ifdef MNSLAC_NODE_M0
  pcln("MNSLAC Node Hardware detected");
  DW1000.begin(PIN_IRQ_NODE, PIN_RST_NODE);
  DW1000.select(PIN_SS_NODE);
#else
  pcln("Non MNSLAC Node Hardware detected", C_ORANGE);
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
  DW1000.enableMode(settings.mode);

  DW1000.interruptOnRxPreambleDetect(false);
  DW1000.interruptOnTxPreambleSent(false);
  DW1000.interruptOnRxFrameStart(true);
  DW1000.interruptOnTxFrameStart(true);

  // Set our antenna delay
  DW1000.setAntennaDelay(antennaDelayList[nodeNumber]);

  DW1000.setManualPower(0x1F1F1F1FL);

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
  settings.t_c  = settings.t_fs + settings.t_fr + settings.t_fc;

  DW1000Time timeList[settings.n];
  timePollReceived = timeList;

  // while(!Serial);
  // *(timePollReceived + 0) = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);
  // Serial.println(*(timePollReceived + 0));
  // =========================================================================//



  // if (isBase)
  //   while(!Serial);

  // Print details about our node number
  section("System Boot");
  sprintf(small_buf, "Own Address: %04X", ownAddress); pcln(small_buf);
  sprintf(small_buf, " -> Node #%d", nodeNumber); pcln(small_buf);
  if (isBase) {
    sprintf(small_buf, " -> IS BASE", nodeNumber); pcln(small_buf, C_GREEN);
  }
  pcln("");

  // Attach DW1000 Handlers
  DW1000.attachSentHandler(txHandle);
  DW1000.attachReceivedHandler(rxHandle);
  DW1000.attachReceiveFailedHandler(rxFailHandler);
  DW1000.attachReceiveTimeoutHandler(rxTimeoutHandler);
  DW1000.attachErrorHandler(clkErrHandler);


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

  // Init global vars
  led = false;

  // Configure our timers

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

  // Calculate settings
  t_r = DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS);

  DWOffset = 0; // DW1000Time(0, DW1000Time::MICROSECONDS);

  state = { sleep };

  // Set our initial state
  if (isBase) {
    // state = { rb_range };
    cycleValid = 10;
    transmitAuthorization = 50;
    cycleStart = micros();
    lastSequenceNumber = 255;
  }

  updateRTC();

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

  return val4*256+val3;
}

void updateRTC() {
  min = rtc.getMinutes();
  hour = rtc.getHours();
  sec = rtc.getSeconds();
}

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

void t_block(uint8_t t)
{
  uint32_t now = micros();
  Serial.print("b");
  Serial.println(now);
}
void t_frame(uint8_t t)
{
  uint32_t now = micros();
  Serial.print("f");
  Serial.println(now);
}

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

  // if(led_chirp_red == true){
  //   digitalWrite(BOARD_RGB_RED, HIGH);
  //   led_chirp_red = false;
  // }else{
  //   tr = manage_led(BOARD_RGB_RED, &led_red, tr);
  // }
  //
  // if(led_chirp_green == true){
  //   digitalWrite(BOARD_RGB_GREEN, HIGH);
  //   led_chirp_green = false;
  // }else{
  //   tg = manage_led(BOARD_RGB_GREEN, &led_green, tg);
  // }
  //
  // if(led_chirp_blue == true){
  //   digitalWrite(BOARD_RGB_BLUE, HIGH);
  //   led_chirp_blue = false;
  // }else{
  //   tb = manage_led(BOARD_RGB_BLUE, &led_blue, tb);
  // }
  //
  // if(led_chirp_aux == true){
  //   digitalWrite(LED_PIN, HIGH);
  //   led_chirp_aux = false;
  // }else{
  //   taux = manage_led(LED_PIN, &led_aux, taux);
  // }
}

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

  // float voltage = analogRead(A2) * 0.0005900679758;
  // Serial.println(getBattVoltage());

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
    // Serial.println("DW1000 Clock Error Detected");
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
