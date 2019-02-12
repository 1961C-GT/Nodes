#define DEBUG
#define LED_PIN 13

#define LEN_DATA 9
#define LEN_MAC 3

// PIN DEFS
#define PIN_RST 9
#define PIN_IRQ 2
#define PIN_SS 10

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


enum DWMODE {
  IDLE,
  RX,
  TX
};

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

char *MsgTypes[] = {
  "ANN MSG",
  "NUL MSG",
  "RNG REQ",
  "RNG RES",
  "RNG REP",
  "COM MSG",
  "SETTING"
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
  byte mode[3];         // MODE_LONGDATA_RANGE_LOWPOWER
  byte channel    : 3;  // CHANNEL_3 (1-7)
  uint8_t n       : 4;  // Number of nodes in the network
  uint16_t t_rx   : 16; // Buffer time for changing rx/tx mode
  uint16_t t_b    : 16; // Buffer time between all blocks

  uint16_t t_r    : 16; // Time between range responses - longer than range_resp message length (~3ms)

  uint8_t n_com   : 8;  // Number of com frames per cycle
  uint16_t bits_c : 16; // Number of bits allowed in a com message
  uint16_t t_cl   : 16; // Time for a single com message - longer than com_msg length

  uint32_t t_sleep; // Time for the sleep frame

  // --- Calculated Settings (from given)
  uint32_t t_br;        // Total time for the each ranging block
  uint32_t t_fr;        // Total time for the ranging frame (all blocks)

  uint32_t t_bc;        // Total time for each com block
  uint32_t t_fc;        // Total time for all com frames (all blocks)

};
