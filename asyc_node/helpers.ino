#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


// Print a message struct to the console
void printMessage(Message msg) {
  Serial.print('['); Serial.print(msg.valid); Serial.print("] ");
  Serial.print("From ("); Serial.print(msg.from);
  Serial.print(") Seq ("); Serial.print(msg.seq);
  Serial.print(") Type ("); Serial.print(MsgTypes[msg.type]);
  Serial.print(") Data {"); DW1000.printPrettyHex(msg.data, msg.len, false);
  Serial.print("} Memory: "); Serial.println(freeMemory());
}

// Get a message from the DW1000 and return its message format
Message getMessage() {

  // Create a data buffer
  byte data[LEN_MAC + LEN_DATA];

  // Populate the main data buffer
  DW1000.getData(data, LEN_MAC + LEN_DATA);

  // Get the length of this message
  uint8_t dataLength = DW1000.getDataLength();

  return getMessage(data, dataLength);
}

// Get a message struct from the given byte array with the given length
Message getMessage(byte data[], uint8_t dataLength) {

  // Create Message Struct
  Message msg;

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

// Get the length of a message object
int getMessageLength(Message msg) {
  return LEN_MAC + msg.len;
}

// Create a message from the given message struct and place the result into
// the given byte buffer
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

void getRangeRecDelay(DW1000Time& t, uint8_t nodeNum, Settings s) {
  if (nodeNum == 0) {
    t.setTime((int32_t) 0);
  } else {
    t.setTime((int32_t) (s.t_rx + s.t_r * nodeNum));
  }
}

uint32_t getRangeRecDelay(uint8_t nodeNum, Settings s) {
  if (nodeNum == 0) {
    return 0;
  } else {
    return (int32_t) (s.t_rx + s.t_r * nodeNum);
  }
}

void printSettings(Settings s) {
  Serial.print("Settings [n:"); Serial.print(s.n);
  Serial.print(", t_rx:"); Serial.print(s.t_rx);
  Serial.print(", t_b:"); Serial.print(s.t_b);
  Serial.print(", t_r:"); Serial.print(s.t_r);
  Serial.print(", t_cl:"); Serial.print(s.t_cl);
  Serial.print(", t_br:"); Serial.print(s.t_br);
  Serial.print(", t_fr:"); Serial.print(s.t_fr);
  Serial.print(", t_bc:"); Serial.print(s.t_bc);
  Serial.print(", t_fc:"); Serial.print(s.t_fc);
  Serial.print(", t_rn:"); Serial.print(s.t_rn);
  Serial.print(", t_sleep:"); Serial.print(s.t_sleep);
  Serial.print(", n_com:"); Serial.print(s.n_com);
  Serial.print(", bits_c:"); Serial.print(s.bits_c);
  Serial.println("]");
}

float runningTotal;
float computeRange(DW1000Time& rec, uint8_t from) {


  // The total time between our poll message being sent and the reply being
  // received.
  DW1000Time total = (rec - timePollSent).wrap();

  // The time that the distant node was suppsed to wait to reply based on their
  // Node Id
  DW1000Time turn = DW1000Time((settings.t_rx + settings.t_r) +  (settings.t_r * from), DW1000Time::MICROSECONDS);

  DW1000Time tof = (total - turn) / 2;

  float distance = tof.getAsMeters();




  // Serial.print("Distance Data: Total("); Serial.print(total.getAsNanoSeconds());
  Serial.print("NodeID: "); Serial.print(from);
  Serial.print("\tTurn("); Serial.print(turn.getAsNanoSeconds());
  Serial.print("), TOF("); Serial.print(tof.getAsNanoSeconds());
  Serial.print("); Sent:"); Serial.print(timePollSent.getAsMicroSeconds());
  Serial.print(", Received:"); Serial.print(rec.getAsMicroSeconds());
  Serial.print(", Dist:"); Serial.println(distance);

  return distance;
}

/*
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

  uint32_t t_sleep; // Time for the sleep frame

  // --- Calculated Settings (from given)
  uint32_t t_br;        // Total time for the each ranging block
  uint32_t t_fr;        // Total time for the ranging frame (all blocks)

  uint32_t t_bc;        // Total time for each com block
  uint32_t t_fc;        // Total time for all com frames (all blocks)

};*/
