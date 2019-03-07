// ========================================================================== //
// STATE: SLEEP
//
// -------------------------------------------------------------------------- //
state_fn sleep, sleep_loop, sleep_decode;
void sleep(struct State * state)
{
  blinkInit();
  // #ifdef DEBUG
    Serial.println(F("[State] ... -> SLEEP"));
  // #endif

  // Make sure that we do not have any block or frame timers set. (which means
  // we will be stuck in sleep until one is set at some point)
  stopTimers();

  // Start RX Mode
  receiver();

  // Set the default next state
  state->next = sleep_loop;

  // If our cycle is valid here at the start of sleep, then go ahead and set our
  // sleep timeout. Otherwise we will have to wait until we can validate it
  // later.
  if (cycleValid > 0 || isBase) {
    Serial.print("[SLEEP] - Cycle valid: "); Serial.println(cycleValid);
    if (isBase)
      cycleStart = micros();

    setFrameTimer(settings.t_fs, cycleStart, true);
  }

  cycleValid --;
  if (cycleValid < 0)
    cycleValid = 0;

  // #ifdef DEBUG
  Serial.println(F("[State] SLEEP -> SLEEP_LOOP"));
  // #endif
}
void sleep_loop(struct State * state)
{
  // See if we have gotten a message
  if (checkRx()) {
    state->next = sleep_decode;
    return;
  }

  // See if it is time to leave the sleep frame. The block state is set to the
  // sleep frame so that if it is set (incorrecly), we will not leave sleep mode
  if (checkTimers(state, range_frame_init, sleep)) {
    cycleStart = micros() - settings.t_fs;
    Serial.println("[SLEEP_LOOP] - Timer Triggered");
    return;
  }

  // If we do not have a frame timer set but we DO have a valid cycle time, then
  // we should go ahead and set our frame timer
  if (!frameTimerSet && cycleValid) {
    Serial.println("[SLEEP_LOOP] - Cycle Now Valid. Setting Frame Timer");
    setFrameTimer(settings.t_fs, micros(), true);
  }
}
void sleep_decode(struct State * state)
{
  // #ifdef DEBUG
  Serial.println(F("[State] SLEEP -> SLEEP_DECODE"));
  // #endif
  blinkInit();

  // Set the defualt next state
  state->next = sleep_loop;

  Serial.print("[SLEEP_DECODE] Rx @: "); Serial.println(rxTimeM0);

  // Get the message from the DW1000 and parse it
  rxMessage = getMessage();
  if (rxMessage.valid) {

    Serial.print("[SLEEP_DECODE] Got Message: "); printMessage(rxMessage);

    // See if we can adjust our clock based on this message
    boolean adjusted = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());

    // If we did adjust our clock, then we should set (or reset) our frame
    // callback
    if (adjusted) {
      Serial.println("[SLEEP_DECODE] Adjusted clock. Setting new frame timer");
      setFrameTimer(settings.t_fs, micros(), true);
    }
  }
}
// ========================================================================== //

// ========================================================================== //
// STATE: RANGE_FRAME_INIT
//
// -------------------------------------------------------------------------- //
state_fn range_frame_init;
void range_frame_init(struct State * state)
{
  // Save the current time before we do any calculations
  uint32_t now = micros();

  Serial.println("[STATE] SLEEP_LOOP -> RANGE_FRAME_INIT");

  // Go ahead and stop any block and frame timers now
  stopTimers();

  if (cycleValid) {

    state->next = ra_init;

    boolean ret = updateTimers(state, sleep, rb_range,
      settings.t_fs + settings.t_fr,
      settings.t_fs + settings.t_br * nodeNumber, now);

    if (!ret) {
      Serial.print("[RANGE_FRAME_INIT] Callback Timer not set. Moving to alternate state (");
      Serial.print(cycleStart); Serial.print(", ");
      Serial.print(micros()); Serial.print(", ");
      Serial.print(settings.t_fs + settings.t_fr); Serial.print(", ");
      Serial.print(settings.t_fs + settings.t_br * nodeNumber); Serial.println(")");
    }

    // Serial.print("Frame Time "); Serial.println(settings.t_fs + settings.t_fr);
    // Serial.print("Block Time "); Serial.println(settings.t_fs + settings.t_br * nodeNumber);

    // // Set up our frame timer for the range frame
    // boolean prompt = setFrameTimer(settings.t_fs + settings.t_fr, now);
    // if (!prompt) {
    //   state->next = sleep;
    //   return;
    // }
    //
    // // Set up the block timer to fire when it is time for us to move to our
    // // ranging block. If that is too soon, then move right to the range
    // // block
    // prompt = setBlockTimer(settings.t_fs + settings.t_br * nodeNumber, now);
    // if (!prompt) {
    //   state->next = rb_range;
    // } else {
    //   state->next = ra_init;
    // }

  } else {
    Serial.println("[RANGE_FRAME_INIT] Clock Not Valid. Returning to sleep");
    // If our cycle is not valid, then we need to wait in sleep instead
    state->next = sleep;
  }
}


// ========================================================================== //


// ========================================================================== //
// STATE: RA_INIT
//
// -------------------------------------------------------------------------- //
state_fn ra_init, ra_init_loop;
void ra_init(struct State * state)
{
  blinkInit();

  // Start RX Mode
  receiver();

  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] ... -> RA_INIT -> RA_INIT_LOOP"));
  #endif

  blinkLoop();
  state->next = ra_init_loop;
}
void ra_init_loop(struct State * state)
{
  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, sleep, rb_range)) {
    Serial.println( "[RA_INIT_LOOP] - Timer Triggered");
    return;
  }

  if (checkRx()) {
    state->next = ra_decode;
    return;
  }

}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_DECODE
//
// -------------------------------------------------------------------------- //
state_fn ra_decode;
void ra_decode(struct State * state)
{
  Serial.println(F("[State] RA_INIT_LOOP -> RA_DECODE"));

  blinkInit();

  // Set the defualt next state
  state->next = ra_init;

  // Message temp value
  rxMessage = getMessage();

  #ifdef DEBUG
    Serial.print("[RA_DECOODE] Rx @: "); Serial.print(rxTimeM0); Serial.print(' ');
    printMessage(rxMessage);
  #endif

  if (rxMessage.valid) {

    // Adjust our clock based on the message we received.
    boolean adj = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());

    switch(rxMessage.type) {
      case RANGE_REQ:
        // If we got a range request, move to range response
        rangeRequestFrom = rxMessage.from;
        rangeRequestSeq = rxMessage.seq;
        state->next = ra_resp;
        Serial.println("[RA_DECOODE] Got Range Request");
        break;
      default:
        Serial.println("[RA_DECOODE] BAD MESSAGE TYPE.");
        // state->next = getState(rxMessage);
        break;
    }

    if (adj) {

      Serial.println("[RA_DECOODE] Adjusting Timers From New Clock Settings");

      // If we got a new clock adjustment, then we should reset our block timer
      // and frame timer to match this more accurate value
      boolean ret = updateTimers(state, sleep, rb_range,
        settings.t_fs + settings.t_fr,
        settings.t_fs + settings.t_br * nodeNumber, micros());

      // If we are late setting this timer, then we should proceed directly to
      // the next block
      if (!ret) {
        Serial.print("[RA_DECOODE] Callback Timer not set. Moving to alternate state (");
        Serial.print(cycleStart); Serial.print(", ");
        Serial.print(micros()); Serial.print(", ");
        Serial.print(settings.t_fs + settings.t_fr); Serial.print(", ");
        Serial.print(settings.t_fs + settings.t_br * nodeNumber); Serial.println(")");
      }
    }
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_RESP
//
// -------------------------------------------------------------------------- //
state_fn ra_resp, ra_resp_loop;
void ra_resp(struct State * state)
{
  blinkTx();

  Serial.println(F("[State] RA_DECODE -> RA_RESP"));

  // Configure for a new transmit
  DW1000.newTransmit();
  DW1000.setDefaults();

  // Set the message to be from us
  txMessage.from = nodeNumber;

  // TODO Maybe compare the calculated block start sequence to the measured
  // one (rangeRequestSeq) in order to ensure we are all on the same page? They
  // should match unless there is some sort of issue
  // uint8_t blockStartSeq = (settings.n + 1 ) * rangeRequestFrom;
  // rangeRequestSeq

  // Apply the sequence number to the message. This is based on our node number
  // as an offset from the seq number of the range request itself.
  txMessage.seq = rangeRequestSeq + nodeNumber + 1;

  // Set the message type
  txMessage.type = RANGE_RESP;

  // Set the message data length to 0 (empty message)
  txMessage.len = 0;

  // get the length of the full message (data + MAC)
  uint8_t len = getMessageLength(txMessage);

  // Create a buffer array to accept the contents of the message
  byte data[len];

  // Create the message and place it in the data buffer
  createMessage(data, txMessage);


  // Schedule the message to send exatly settings.t_r micros from the last
  // rx
  DW1000.setDelayFromRx(t_r); // DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS)

  // Set the data of the message
  DW1000.setData(data, len);

  // Start the message transmission
  DW1000.startTransmit();

  Serial.println(F("[State] RA_RESP -> RA_RESP_LOOP"));

  // Proceed to loop state
  state->next = ra_resp_loop;
}
void ra_resp_loop(struct State * state)
{
  // TODO Do we need these here?
  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  // if (checkTimers(state, sleep, rb_range)) {
  //   return;
  // }

  if(checkTx()) {
    // DW1000.readSystemEventStatusRegister();
    // Serial.print("Late: "); Serial.println(DW1000.isLate());

    // DW1000Time txTime;
    // DW1000.getTransmitTimestamp(txTime);
    //
    // DW1000Time turn = (txTime - timePollSent).wrap();
    // Serial.print("Turn: "); Serial.print(turn.getAsNanoSeconds());
    // Serial.print("ns, Goal: "); Serial.println(t_r.getAsMicroSeconds());
    // Serial.print("ns, TX:"); Serial.print(txTime.getAsMicroSeconds());
    // Serial.print("ns, ");
    receiver();
    state->next = ra_init;
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RB_RANGE
//
// -------------------------------------------------------------------------- //
state_fn rb_range, rb_range_loop;
void rb_range(struct State * state)
{
  #ifdef DEBUG
    Serial.print(F("[State] ... -> RB_RANGE: ")); Serial.println(micros());
  #endif

  // Set this block to end at the end of this node's range block. We do not
  // set the frame timer here because we do not have new and more accurate
  // information here. The block timer is being set to tell us to move back
  // to the other state
  boolean prompt = setBlockTimer(settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

  // If it is time to move on now, then do so
  if (!prompt) {
    Serial.println("[RB_RANGE] Callback Timer not set. Moving to alternate state");
    state->next = ra_init;
    return;
  }

  // Configure out LED
  blinkTx();

  // Configure for transmission
  DW1000.newTransmit();
  DW1000.setDefaults();

  // Set this message to be from us
  txMessage.from = nodeNumber;

  // Set the sequence number to be the start of this node's range request
  // block. This is the specific number associated with this request
  txMessage.seq = (settings.n + 1) * nodeNumber;

  // Set the message type to a range request
  txMessage.type = RANGE_REQ;

  // Set the length of the message data to 0
  txMessage.len = 0;

  // Get the full length of the message (data + MAC)
  uint8_t len = getMessageLength(txMessage);

  // Create a buffer to store the byte form of the message
  byte data[len];

  // Create the message and store it in data
  createMessage(data, txMessage);

  // Set the message data to data
  DW1000.setData(data, len);

  // Set the message delay so that it sends exacty when it should (exactly t_rx
  // after the start of this block)
  int64_t delay = getDelayMicros(settings.t_fs + (settings.t_br * nodeNumber) + settings.t_rx, micros());

  // Only set the delay of the message if our delay value is greater than the
  // MIN_DELAY. Otherwise, we send out message right away (Without settings a
  // delay)
  if (delay > MIN_TX_DELAY) {
    DW1000.setDelay(DW1000Time(delay, DW1000Time::MICROSECONDS));
  }

  // Start the transmission
  DW1000.startTransmit();

  Serial.print("[RB_RANGE] - XMit Delay: " ); Serial.println((long) delay);

  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] RB_RANGE -> RB_RANGE_LOOP"));
  #endif
  state->next = rb_range_loop;
}
void rb_range_loop(struct State * state)
{
  // TODO Do we need these here?
  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  // if (checkTimers(state, sleep, ra_init)) {
  //   return;
  // }

  if(checkTx()) {
    receiver();
    state->next = rb_rec;
    return;
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RB_REC
//
// -------------------------------------------------------------------------- //
state_fn rb_rec, rb_rec_loop;
void rb_rec(struct State * state)
{
  Serial.println("[STATE] RB_RANGE_LOOP -> RB_REC");
  blinkInit();

  // Enter receiver mode right away to catch messages as fast as possible
  receiver();

  // Reset our block timer based on the time at which our transmission sent
  // (which is the NEW block sync)
  boolean adjusted = adjustClock(txMessage, txTimeM0, txTimeDW.getAsMicroSeconds());
  if (!adjusted) {
    Serial.print("[RB_REC] Err Clock Not Adjusted on TX: "); printMessage(txMessage);
  } else {
    Serial.println("[RB_REC] Set new Frame and Block Timers");
    // If we got a new clock adjustment, then we should reset our block timer
    // and frame timer to match this more accurate value
    boolean ret = updateTimers(state, sleep, ra_init,
      settings.t_fs + settings.t_fr,
      settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

    // If we are late setting this timer, then we should proceed directly to
    // the next block
    if (!ret) {
      Serial.print("[RB_REC] Callback Timer not set. Moving to alternate state (");
      Serial.print(cycleStart); Serial.print(", ");
      Serial.print(micros()); Serial.print(", ");
      Serial.print(settings.t_fs + settings.t_fr); Serial.print(", ");
      Serial.print(settings.t_fs + settings.t_br * (nodeNumber + 1)); Serial.println(")");
      return;
    }
  }

  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, sleep, ra_init)) {
    Serial.println( "[RB_REC] Timer Triggered");
    return;
  }

  // Store the time at which we sent this message (for calculating TOF)
  timePollSent = txTimeDW;

  #ifdef DEBUG
    Serial.println(F("[State] RB_REC -> RB_REC_LOOP"));
  #endif

  // Proceed to loop state
  blinkLoop();
  state->next = rb_rec_loop;
}
void rb_rec_loop(struct State * state)
{
  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, sleep, ra_init)) {
    Serial.println( "[RB_REC_LOOP] Timer Triggered");
    return;
  }

  if(checkRx()) {
    state->next = rb_decode;
    return;
  }
}
// ========================================================================== //

// ========================================================================== //
// STATE: RB_DECODE
//
// -------------------------------------------------------------------------- //
state_fn rb_decode;
void rb_decode(struct State * state)
{
  blinkInit();
  #ifdef DEBUG
    Serial.print(F("[State] RB_REC_LOOP -> RB_DECODE: ")); Serial.println(micros());
  #endif

  // Set the defualt next state
  state->next = rb_rec;

  // Load the message into our rxMessage Object
  rxMessage = getMessage();

  Serial.print("[RB_DECODE] Rx @: "); Serial.print(rxTimeM0); Serial.print(' ');
  printMessage(rxMessage);

  // Ensure that the message is valid
  if (rxMessage.valid) {

    // Adjust our clock based on the message we received
    boolean adjusted = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());

    // See what type of message it is
    switch(rxMessage.type){
      case RANGE_RESP:

        if (rxMessage.from < settings.n) {
          DW1000Time rec;
          DW1000.getReceiveTimestamp(rec);
          float dist = computeRange(rec, rxMessage.from);

          Serial.print("[RB_DECODE] Range RESP - NodeID: "); Serial.print(rxMessage.from);
          Serial.print("\tSeq: "); Serial.print(rxMessage.seq);
          Serial.print("\tDistance: "); Serial.println(dist);

        } else {
          Serial.print("[RB_DECODE] Out of Bounds Node ID: "); Serial.println(rxMessage.from);
        }

        break;
      case SETTINGS:
        // TODO
      default:
        Serial.print("[RB_DECODE] BAD MESSAGE TYPE: "); Serial.println(rxMessage.type);
        break;
    }

    // If we adjused our clock, then reset our callback timers
    if (adjusted) {
      Serial.println("[RB_DECODE] - Set new Frame and Block Timers");

      // If we got a new clock adjustment, then we should reset our block timer
      // and frame timer to match this more accurate value
      boolean ret = updateTimers(state, sleep, ra_init,
        settings.t_fs + settings.t_fr,
        settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

      // If we are late setting this timer, then we should proceed directly to
      // the next block
      if (!ret) {
        Serial.print("[RB_DECODE] Callback Timer not set. Moving to alternate state (");
        Serial.print(cycleStart); Serial.print(", ");
        Serial.print(micros()); Serial.print(", ");
        Serial.print(settings.t_fs + settings.t_fr); Serial.print(", ");
        Serial.print(settings.t_fs + settings.t_br * (nodeNumber + 1)); Serial.println(")");
        return;
      }
    }
  }

  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  // if (checkTimers(state, sleep, ra_init)) {
  //   Serial.println( "[RB_DECODE] Timer Triggered");
  //   return;
  // }
}
// ========================================================================== //
