#define DEBUG 50
#define DEBUG0
#define DEBUG1
#define DEBUG2
#define DEBUG3
#define DEBUG4
#define DEBUG5

#define LED_PIN 13

#define LEN_DATA 9
#define LEN_MAC 3

#define TINY_BUFFER_LEN 12
#define SMALL_BUFFER_LEN 80
#define MEDIUM_BUFFER_LEN 120
#define LARGE_BUFFER_LEN 255

char tiny_buf[TINY_BUFFER_LEN];
char small_buf[SMALL_BUFFER_LEN];
char medium_buf[MEDIUM_BUFFER_LEN];
char large_buf[LARGE_BUFFER_LEN];

#define D_BLINK_C_CYAN D_BLINK C_CYAN

#define BG_RED_C_WHITE BG_RED C_WHITE

// PIN DEFS
#define PIN_RST_BREAD 9
#define PIN_IRQ_BREAD 2
#define PIN_SS_BREAD 10

#define PIN_RST_NODE 8
#define PIN_IRQ_NODE 2
#define PIN_SS_NODE 10

#define BOARD_RGB_RED 11
#define BOARD_RGB_GREEN 7
#define BOARD_RGB_BLUE 5

// Coef for calculating the battery voltage from the BATT pin.
// 3.31 / 1024.0 * 2 = 0.00646484375
#define BATT_MEAS_COEFF 0.00646484375

// The minimum time that we are allowed to set a delay callback for. If the
// needed delay is less than this value, then we forget the delay and move to
// the next block or frame anyway
#define MIN_DELAY 500 //150

// The minimum time that we are allowed to set a delay transmit for. If the
// needed delay is less than this value, then we forget the delay and transmit
// as soon as possible instead
#define MIN_TX_DELAY 4000

// System time variables
#define SINCE(value) (micros() - (uint32_t) value)

//// ENUM DEFINITIONS
//   STATE OPTIONS
// enum State {
//   RA_INIT,
//   RA_DECODE,
//   RA_RESP,
//   RB_RANGE,
//   RB_REC_INIT,
//   RB_REC,
//   RB_RANGE_DECODE,
//   R_EXIT,
//   CA_INIT,
//   CA_DECODE,
//   CB_XMIT,
//   C_EXIT
// };


typedef void state_fn(struct State *);
struct State
{
    state_fn * next;
};
struct State;

//   MESSAGE FORMATS
enum Msg_Type {
  ANNOUNCE = 0,
  RANGE_REQ = 2,
  RANGE_RESP = 3,
  RANGE_REPORT = 4,
  COM_MSG = 5,
  SETTINGS = 6
};

//   MESSAGE FORMATS
enum Led_Mode {
  MODE_NULL        = 0,
  MODE_OFF         = 1,
  MODE_ON          = 2,
  MODE_BLINK       = 3,
  MODE_RAMP        = 4,
  MODE_DOUBLE_RAMP = 5,
  MODE_BLINK_DIM   = 6,
  MODE_CHIRP       = 7
};

enum Led {
  LED_AUX   = 0,
  LED_RED   = 1,
  LED_GREEN = 2,
  LED_BLUE  = 3
};

Led_Mode led_modes[] = {
  MODE_OFF,
  MODE_OFF,
  MODE_OFF,
  MODE_OFF
};

boolean led_updated[] = {
  false,
  false,
  false,
  false
};

uint8_t led_pin_list[] = {
  LED_PIN,
  BOARD_RGB_RED,
  BOARD_RGB_GREEN,
  BOARD_RGB_BLUE
};

char *MsgTypes[] = {
  "ANN MSG",
  "NUL MSG",
  "RNG REQ",
  "RNG RES",
  "RNG REP",
  "COM MSG",
  "SETTING"
};

enum Block{
  BLOCK_RANGE_ACCEPT,
  BLOCK_RANGE_REQUEST,
  BLOCK_COM,
  BLOCK_SLEEP,
  BLOCK_NONE
};

enum Frame{
  FRAME_RANGE,
  FRAME_COMS,
  FRAME_SLEEP,
  FRAME_NONE
};

//   COM MESSAGE CONTENTS
enum Com_Type{
  COM_RANGE_REPORT,
  COM_TELEM_REPORT,
  COM_CONFIG,
  COM_SLEEP
};

// Message Structure
struct Message {
  uint8_t from : 4;
  Msg_Type type : 4;
  uint8_t seq  : 8;
  uint8_t len  : 4;
  byte data[LEN_DATA];
  uint8_t valid : 1;
};

// Settings Structure
// all times in us
struct Settings {


  // --- Given Settings
  const byte * mode;         // MODE_LONGDATA_RANGE_LOWPOWER
  byte channel;  // CHANNEL_3 (1-7)
  uint8_t n       : 4;  // Number of nodes in the network
  uint16_t t_rx   : 16; // Buffer time for changing rx/tx mode
  uint16_t t_b    : 16; // Buffer time between all blocks

  uint16_t t_r    : 16; // Time between range responses - longer than range_resp message length (~3ms)

  uint8_t n_com   : 8;  // Number of com frames per cycle
  uint16_t bits_c : 16; // Number of bits allowed in a com message
  uint16_t t_cl   : 16; // Time for a single com message - longer than com_msg length

  uint32_t t_s;         // Time for the sleep frame

  // --- Calculated Settings (from given)
  uint32_t t_br;        // Total time for the each ranging block

  uint32_t t_fr;        // Total time for the ranging frame (all blocks)

  uint32_t t_bc;        // Total time for each com block
  uint32_t t_fc;        // Total time for all com frames (all blocks)

  uint32_t t_rn;        // The amount of time that this node waits after a RANGE_REQ before sending a RANGE_RESP

  uint32_t t_fs;        // total time for the Sleep Frame

  uint32_t t_c;         // Total Cycle Length
};


// Timer Defs
const uint8_t _LED_TIMER = M0Timer.T5;
const uint8_t _BLOCK_TIMER = M0Timer.T4;
const uint8_t _FRAME_TIMER = M0Timer.T3;
