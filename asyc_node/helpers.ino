#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#ifdef MNSLAC_NODE_M0
  float getBattVoltage() { return analogRead(BATT) * BATT_MEAS_COEFF; }
#else
  float getBattVoltage() { return 0.0f; }
#endif

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
  Serial.print(", t_s:"); Serial.print(s.t_s);
  Serial.print(", t_fs:"); Serial.print(s.t_fs);
  Serial.print(", t_c:"); Serial.print(s.t_c);
  Serial.print(", n_com:"); Serial.print(s.n_com);
  Serial.print(", bits_c:"); Serial.print(s.bits_c);
  Serial.println("]");
}

float runningTotal;
float computeRange(DW1000Time& rec, uint8_t fromId) {


  // The total time between our poll message being sent and the reply being
  // received.
  DW1000Time total = (rec - timePollSent).wrap();

  // The time that the distant node was suppsed to wait to reply based on their
  // Node Id
  DW1000Time turn = DW1000Time(settings.t_r +  (settings.t_r * fromId), DW1000Time::MICROSECONDS);

  DW1000Time tof = (total - turn) / 2;

  float distance = tof.getAsMeters();


  // Serial.print("Distance Data: Total("); Serial.print(total.getAsNanoSeconds());
  // Serial.print("NodeID: "); Serial.print(fromId);
  // Serial.print("\tTurn("); Serial.print(turn.getAsNanoSeconds());
  // Serial.print("), TOF("); Serial.print(tof.getAsNanoSeconds());
  // Serial.print("); Sent:"); Serial.print(timePollSent.getAsMicroSeconds());
  // Serial.print(", Received:"); Serial.print(rec.getAsMicroSeconds());
  // Serial.print(", Dist:"); Serial.println(distance);

  return distance;
}

// TODO Finish adjust clock
boolean adjustClock(Message msg, uint32_t m0rx, float dwrx, boolean force = false) {

  // Figure out what frame we are in
  Frame curFrame = getFrameFromSeq(msg.seq, settings);
  // Figure out if this is the start of this block
  boolean blockStart = isBlockStart(curFrame, msg.seq, settings);

  // If this message is the start of a block OR the user has set force to true,
  // the we adjust our clocks
  if (blockStart || force) {

    // Reset our DWOffset based on the message we got. Usage of this offset value
    // us explained at its def.
    DWOffset = m0rx - dwrx;

    // From the message, get the exact micros at which that message should have
    // been received
    uint32_t curCycleTime = getTimeAtSeq(msg.seq);

    // At the exact moment that the message was recieved, the cycle was at cycle
    // time curCycleTime. Therefore the start of the cycle occured curCycleTime
    // micros before the message was received in M0 time.
    cycleStart = m0rx - curCycleTime;

    // Because we have just confirmed our cycle, set cycleValid to true
    cycleValid = 5;

    // Return true to denote a clock adjustment
    return true;
  }

  // Return false to denote no clock adjustment
  return false;
}

uint32_t getTimeAtSeq(uint8_t seq) {
  Settings s = settings;

  // Start with the sleep frame time (which is always at the start of the cycle)
  uint32_t ret = s.t_fs;

  // Find the very first sequence number of the COM frame
  uint8_t comFirstSeq = s.n * (s.n + 1);

  // See if we are before the first com seq number
  uint8_t isRangeFrame = seq < comFirstSeq;

  if (isRangeFrame) {
    // Find what block we are in within this frame. This relies on interger
    // truncation as a "floor" operation.
    uint8_t blockNum = seq / (s.n + 1);

    // Find what seq we are within this block.
    uint8_t seqIdx = (seq % (s.n + 1));

    // For each block that we are already past, we add one block length.
    // Remember that the block number starts from zero. This means that if we
    // are in the 0th block, we should add no additional time. If we are the
    // 1st block, then we need only add the time of the 0th block.
    ret += (s.t_br * blockNum);

    // Now within our block, we add the time to get to the given sequence id.
    // Just as with the block numbers, the sequence index starts at zero. If
    // the 0th message comes through, then we only add in the rx time. If the
    // 1st message comes through, we add the rx time plus one range division
    // (t_r)
    ret += (s.t_r * seqIdx) + s.t_rx;
  }
  // If we are not in the range frame, then we are in the com frame
  else {
    // Because we are past the ENTIRE range frame, we can simply add the time
    // to compleate the whole thing now
    ret += s.t_fr;

    // Using the value we calculated earlier for the first seq number of the
    // com frame, we can see how many sequances we are into this frame. As with
    // the Range block number, this block number starts from 0
    uint8_t blockNum = seq - comFirstSeq;

    // Add the time for all the blocks in the COM frame before this one.
    // Because the block number starts at 0, we need only do a simple
    // multiplication to achieve this. Finally we add a single t_rx to account
    // for the fact that each com message is t_rx deep into its block
    ret += (s.t_bc * blockNum) + s.t_rx;
  }
  return ret;
}

// Get the block that the given Node Id should be in, given that the current
// message has the given sequence number.
Block getBlockFromSeq(uint8_t seq, uint8_t nodeId, Settings s) {
  // Figure out what frame we are in
  Frame curFrame = getFrameFromSeq(seq, s);

  // If we are in the range frame, then use the following calculation
  if (curFrame == FRAME_RANGE) {
    // Figure out which block within the range frame we are in
    uint8_t blockNum = seq / (s.n + 1);

    // If our nodeid matches the block number, then it is our turn to be the
    // requester. Otherwise, we should be an accepter.
    if (nodeId == blockNum) {
      return BLOCK_RANGE_REQUEST;
    } else {
      return BLOCK_RANGE_ACCEPT;
    }
  }

  // if we are in the com frame, then we are always in a com block
  else if (curFrame == FRAME_COMS) {
    return BLOCK_COM;
  }

  // if we are in the com frame, then we are always in a com block
  else if (curFrame == FRAME_SLEEP) {
    return BLOCK_SLEEP;
  }

  // If not any others, then assume we are in the sleep block
  else {
    return BLOCK_NONE;
  }
}

// Gets a message's index within a single block
uint8_t getMsgIdx(Frame frame, uint8_t seq, Settings s) {
  // When we are in the range frame, the following formula will result in the
  // message index
  if (frame == FRAME_RANGE) {
    return (seq % (s.n + 1));
  } else {
    return 0;
  }
}

// Return whether or not the given message sequance number is the start of its
// block
boolean isBlockStart(Frame frame, uint8_t seq, Settings s) {
  return getMsgIdx(frame, seq, s) == 0;
}

// Return whether or not the given message sequance number is the end of its
// block
boolean isBlockEnd(Frame frame, uint8_t seq, Settings s) {
  if (frame == FRAME_RANGE) {
    return getMsgIdx(frame, seq, s) == s.n;
  } else {
    return true;
  }
}

// LEFT OFF:
// Setting up get state to work. This will go in the decode states in order to
// help figure out whether or not we need to move to a different state based on
// the message we got. Also might help with moving us between blocks and frames

// Gets state we should move to based on the given sequence number and message
// type. This function will return the state to move to next.
// To be execute if we do not receive the expected message
state_fn * getState(Message msg) {
  Settings s = settings;
  state_fn * nextState;

  // See what we should be doing next sequence
  Frame curFrame = getFrameFromSeq(msg.seq + 1, s);
  Block curBlock = getBlockFromSeq(msg.seq + 1, nodeNumber, s);

  // If this message index is 0, then it is the first in this block
  boolean blockStart = isBlockStart(curFrame, msg.seq + 1, s);

  // If this messsage index is the number of nodes in the network, then it is
  // the last message in this block
  boolean blockEnd = isBlockEnd(curFrame, msg.seq + 1, s);

  if (curFrame == FRAME_RANGE) {

    // Check if we should be in the range accept block
    if (curBlock == BLOCK_RANGE_ACCEPT) {
      switch (msg.type) {
        // If we just got a range request, jump right to the ra_resp state
        case RANGE_REQ:
          nextState = { ra_resp };
          break;
        // For all other message types (should only be RANGE_RESP), simply
        // enter the ra_init state
        default:
          nextState = { ra_init };
          break;
      }
    }

    // If we are suposed to be in the range request block, the move to rb_range
    else if (curBlock == BLOCK_RANGE_REQUEST) {
      nextState = { rb_range };
    }

    else {
      nextState = { ra_init };
    }
  } else if (curFrame == FRAME_COMS) {
    nextState = { sleep };
  } else {
    nextState = { sleep };
  }

  return nextState;
}

// Get the frame the given sequence number falls in, given the current settings
Frame getFrameFromSeq(uint8_t seq, Settings s) {
  // There are n * (n+1) messages required for a full ranging frame. Given that
  // the sequence numbers start from 0, we can get away with using a < operator
  // for this alone (Eg for n=5, the sequences in the Range frame would number
  // from 0 to 29 (accounting for one "blank" seq for the Request node). If our
  // current sequence is 29, that is less than 5*6. If our seq is 30, that is
  // equal to 5*6 and would therefore fail this if statement)
  if (seq < (s.n * (s.n + 1)))
    return FRAME_RANGE;
  else if (seq < (s.n * (s.n + 1) + s.n * s.n_com))
    return FRAME_COMS;
  else
    return FRAME_SLEEP;
}

void stopTimers() {
  M0Timer.stop(_BLOCK_TIMER);
  M0Timer.stop(_FRAME_TIMER);

  blockTimerSet = false;
  frameTimerSet = false;
}

// Get the time from now until the event at cycleDelay micros from cycleStart
// will occur.
long getDelayMicros(uint32_t cycleDelay, uint32_t now) {
  long delay = (long) (cycleStart - now + cycleDelay);
  return delay;
}

// Set the frame timer based on the cycle delay and the offset. The cycle delay
// should be the number of micros between the start of the cycle and when the
// timer should fire. The offset should be the current time
boolean setFrameTimer(uint32_t cycleDelay, uint32_t now, boolean wrapTime = false) {
  // Set the delay based on the cycle start time, the offset (now time), and
  // the callback delay (referenced to the cycle start time)
  long delay = getDelayMicros(cycleDelay, now);

  // Serial.print("Frame Timer Pre: "); Serial.println((long) delay);
  // Serial.print("cycleStart "); Serial.println(cycleStart);
  // Serial.print("now "); Serial.println(now);
  // Serial.print("cycleDelay"); Serial.println(cycleDelay);
  //
  // if (delay < 0) {
  //   Serial.println("Less than 0");
  // } else if (delay == 0) {
  //   Serial.println("Equal to 0");
  // } else if (delay > 0) {
  //   Serial.println("Greater than 0");
  // } else
  //   Serial.println("idk");

  // Serial.println((long)delay);

  // If wrapping is enabled, then we ensure that the delay is greater than
  // min delay by wrapping an ENTIRE cycle time onto it until that is the case.
  // The means that the time will always be the NEXT cycleDelay offset from a
  // cycleStart with a value greater than MIN_DELAY. IF wrapping is disabled
  // and the delay is less than MIN_DELAY, then we cannot set the timer
  if (wrapTime) {
    while (delay < MIN_DELAY) {
      delay += settings.t_c; //settings.t_fs + settings.t_fr + settings.t_fc;
    }
  } else if (delay < MIN_DELAY) {
    // Serial.println("FT False");
    return false;
  }

  // Start the frame timer with the given delay. Set it to calculate an internal
  // offset such that the timer starts as close as possible to right now.
  // TODO Should we use now instead of micros() for the offset?
  M0Timer.start(delay, _FRAME_TIMER, now);
  frameTimerSet = true;

  Serial.print("Frame Timer set: "); Serial.print((long) delay);
  Serial.print(", "); Serial.print(cycleDelay);
  Serial.print(", "); Serial.println(now);

  return true;
}

boolean setBlockTimer(uint32_t cycleDelay, uint32_t now) {
  // Set the delay based on the cycle start time, the offset (now time), and
  // the callback delay (referenced to the cycle start time)
  long delay = getDelayMicros(cycleDelay, now);

  // if (delay < 0) {
  //   Serial.println("Less than 0");
  // } else if (delay == 0) {
  //   Serial.println("Equal to 0");
  // } else if (delay > 0) {
  //   Serial.println("Greater than 0");
  // } else
  //   Serial.println("idk");

  if (delay < MIN_DELAY) {
    return false;
  }

  // Start the block timer with the given delay. Set it to calculate an internal
  // offset such that the timer starts as close as possible to right now.
  // TODO Should we use now instead of micros() for the offset?
  M0Timer.start(delay, _BLOCK_TIMER, micros());
  blockTimerSet = true;

  Serial.print("Block Timer set: "); Serial.print((long) delay);
  Serial.print(", "); Serial.print(cycleDelay);
  Serial.print(", "); Serial.println(now);

  return true;
}

boolean updateTimers(struct State * state, state_fn * frameState, state_fn * blockState, uint32_t frameDelay, uint32_t blockDelay, uint32_t now) {

  // Set the frame timer. IF we are too close to the frame timer's time, then we
  // set our state to the frameState value and return false to alert the user
  boolean prompt = setFrameTimer(frameDelay, now);
  if (!prompt) {
    M0Timer.stop(_BLOCK_TIMER); // TODO Not Needed?
    state->next = frameState;
    return false;
  }

  // Set up the block timer to fire when it is time for us to move to our
  // ranging block. If that is too soon, then move to blockState and return
  // false to alert the user
  prompt = setBlockTimer(blockDelay, now);
  if (!prompt) {
    state->next = blockState;
    return false;
  }

  // If both timers were set correctly, return true
  return true;
}

// Check the block and frame timers to see if they have fired. If they have,
// then set the next state to one of the given states (frame and block). Returns
// true if a timer fired. Returns false if no timer fired
boolean checkTimers(struct State * state, state_fn * frameState, state_fn * blockState) {
  // Frame Timer
  if (M0Timer.getFired(_FRAME_TIMER)) {
    Serial.print("Frame Timer Fired: "); Serial.println(micros());
    state->next = frameState;
    return true;
  }

  if (M0Timer.getFired(_BLOCK_TIMER)) {
    Serial.print("Block Timer Fired: "); Serial.println(micros());
    state->next = blockState;
    return true;
  }

  return false;
}

// void resetTimers(uint8_t seq, uint32_t offsetTime) {
//   switch (getFrameFromSeq(seq, settings)) {
//     case FRAME_RANGE:
//         M0Timer.start(settings.t_br, _BLOCK_TIMER, rxTime);
//       break;
//     case FRAME_COMS:
//         M0Timer.start(settings.t_bc, _BLOCK_TIMER, rxTime);
//       break;
//     case FRAME_SLEEP:
//         M0Timer.stop(_BLOCK_TIMER);
//         M0Timer.start(settings.t_s, _FRAME_TIMER, rxTime);
//       break;
//     default:
//       break;
//   }
// }

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
